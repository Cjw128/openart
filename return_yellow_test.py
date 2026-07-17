# ======================================================================
# return_yellow_test.py - OpenART return-to-garage yellow-line preview
# ======================================================================
# Run directly from OpenART/OpenMV IDE. The script mirrors the 0x07 return
# detector in main.py/minimain.py and draws the selected lines on the frame.
#
# Overlay:
#   cyan box         center vertical ROI (searched top to bottom)
#   magenta box      lower horizontal ROI (searched right to left)
#   thin red line    stop threshold x=200
#   orange/green     selected horizontal return line (pending/valid)
#   blue/red line    selected vertical stop line (before/after threshold)
#   blue strips      blue-cloth boundary sampling ROIs
#   yellow line      active blue-cloth cut line
#
# Set CAMERA_ROLE for the camera being tested. UART output is disabled by
# default so this preview cannot unexpectedly stop a moving car.
# ======================================================================

import gc
import sensor
import time


CAMERA_ROLE = "master"       # Only selects the camera threshold; both roles stop independently.
ENABLE_UART_OUTPUT = False   # True sends the same 7-byte 0xC8 packet as the runtime.

CALIBRATION_FILE = "/sd/color_thr.txt"
SOFTWARE_HMIRROR = True
WB_GAINS = (101.00, 64.00, 97.00)
DEFAULT_EXPOSURE_US = 1200
DEFAULT_BLUE_GROUND_THRESHOLD = [(25, 62, -3, 57, -96, 127)]

MASTER_RETURN_YELLOW_THRESHOLD = [(59, 94, -71, 37, -26, 113)]
SLAVE_RETURN_YELLOW_THRESHOLD = [(59, 94, -71, 37, -26, 113)]

RETURN_YELLOW_ROI = (150, 30, 20, 210)
RETURN_YELLOW_MIN_PIXELS = 5
RETURN_YELLOW_MIN_AREA = 5
RETURN_YELLOW_STABLE_FRAMES = 1
RETURN_YELLOW_STABLE_DELTA = 3

RETURN_STOP_ROI = (0, 200, 320, 20)
RETURN_STOP_X_THRESHOLD = 200
RETURN_STOP_MIN_PIXELS = 5
RETURN_STOP_MIN_AREA = 5
RETURN_STOP_HORIZONTAL_GUARD = 3
RETURN_STOP_MIN_BLOB_H = 8
RETURN_STOP_MAX_WIDTH_HEIGHT_X100 = 300

ENABLE_DYNAMIC_CUT = True
CUT_BLOB_MIN_H = 12
CUT_BLOB_BOTTOM_MARGIN = 25
CUT_GAP_BRIDGE = 10
CUT_STRIP_XS = (10, 85, 160, 235, 310)
CUT_MIN_VALID_STRIPS = 3
CUT_SINGLE_STRIP_MAX_LEAD = 40
CUT_MIN_VALID_X_SPAN = 180
CUT_MAX_STEP_UP = 5
CUT_MAX_STEP_DOWN = 16
CUT_STRIP_HALF_W = 2
CUT_SCAN_Y_MIN = 0
CUT_SCAN_Y_MAX = 140
CUT_STRIP_ROIS = [
    (x - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN,
     CUT_STRIP_HALF_W * 2 + 1, CUT_SCAN_Y_MAX - CUT_SCAN_Y_MIN)
    for x in CUT_STRIP_XS
]
CUT_UPDATE_INTERVAL = 2
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

RETURN_YELLOW_PACKET_ID = 0xC8
RETURN_STATUS_Y_VALID = 0x01
RETURN_STATUS_STOP = 0x02

ROI_Y_COLOR = (0, 255, 255)
ROI_STOP_COLOR = (255, 0, 255)
THRESHOLD_COLOR = (255, 0, 0)
PENDING_LINE_COLOR = (255, 128, 0)
VALID_LINE_COLOR = (0, 255, 0)
TRACK_STOP_COLOR = (0, 128, 255)
TRIGGER_STOP_COLOR = (255, 0, 0)
TEXT_COLOR = (255, 255, 255)
CUT_SAMPLE_COLOR = (0, 0, 255)
CUT_LINE_COLOR = (255, 255, 0)

if CAMERA_ROLE == "slave":
    RETURN_YELLOW_THRESHOLD = SLAVE_RETURN_YELLOW_THRESHOLD
else:
    CAMERA_ROLE = "master"
    RETURN_YELLOW_THRESHOLD = MASTER_RETURN_YELLOW_THRESHOLD

return_yellow_last_y = -1
return_yellow_stable_count = 0
return_yellow_detected = False
return_yellow_y = 0
return_stop_x = -1
return_stop_requested = False
dynamic_cut_y = 0
dynamic_cut_valid = False
dynamic_cut_miss_count = 0


def snapshot_frame():
    img = sensor.snapshot()
    if SOFTWARE_HMIRROR:
        img = img.replace(hmirror=True)
    return img


def average_ground_threshold(ground_rows):
    ground = ground_rows.get("ground")
    ground2 = ground_rows.get("ground2")
    if ground and ground2:
        averaged = []
        for i in range(6):
            averaged.append((ground[i] + ground2[i]) // 2)
        return tuple(averaged)
    return ground if ground else ground2


def parse_int_values(parts):
    values = []
    for value in parts:
        values.append(int(value))
    return tuple(values)


def load_test_calibration(path=CALIBRATION_FILE):
    exposure = None
    ground_rows = {}
    try:
        with open(path, "r") as calibration_file:
            for line in calibration_file:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("exposure_us="):
                    try:
                        value = int(line.split("=", 1)[1])
                        exposure = value if value > 0 else None
                    except Exception:
                        exposure = None
                    continue
                if line.startswith("ground=") or line.startswith("ground2="):
                    try:
                        name, raw_values = line.split("=", 1)
                        values = parse_int_values(raw_values.split(","))
                        if len(values) == 6:
                            ground_rows[name] = values
                    except Exception:
                        pass
    except Exception:
        return None, None
    return exposure, average_ground_threshold(ground_rows)


def clamp_int(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def pick_blue_top_y(blobs):
    if not blobs:
        return None
    top_y = None
    for blob in blobs:
        if blob.h() < CUT_BLOB_MIN_H:
            continue
        if blob.y() + blob.h() < CUT_SCAN_Y_MAX - CUT_BLOB_BOTTOM_MARGIN:
            continue
        if top_y is None or blob.y() < top_y:
            top_y = blob.y()
    if top_y is None:
        return None

    bridged_top = None
    for blob in blobs:
        if blob.h() < CUT_BLOB_MIN_H:
            continue
        bottom = blob.y() + blob.h()
        if bottom <= top_y and top_y - bottom <= CUT_GAP_BRIDGE and blob.y() < top_y:
            if bridged_top is None or blob.y() < bridged_top:
                bridged_top = blob.y()
    return bridged_top if bridged_top is not None else top_y


def update_dynamic_cut(img, frame_count):
    global dynamic_cut_y, dynamic_cut_valid
    global dynamic_cut_miss_count

    if (not ENABLE_DYNAMIC_CUT or
            frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    top_ys = []
    strip_xs = []
    for i in range(len(CUT_STRIP_ROIS)):
        try:
            blobs = img.find_blobs(
                BLUE_GROUND_THRESHOLD, roi=CUT_STRIP_ROIS[i],
                pixels_threshold=CUT_MIN_PIXELS,
                area_threshold=CUT_MIN_AREA, merge=True)
        except Exception:
            blobs = None
        top_y = pick_blue_top_y(blobs)
        if top_y is not None:
            top_ys.append(top_y)
            strip_xs.append(CUT_STRIP_XS[i])

    valid_strips = len(top_ys)
    if (valid_strips >= CUT_MIN_VALID_STRIPS and
            max(strip_xs) - min(strip_xs) < CUT_MIN_VALID_X_SPAN):
        valid_strips = 0

    top_y_pick = None
    if valid_strips >= 1:
        top_ys.sort()
        pick_index = (valid_strips - 1) * 2 // 3
        top_y_pick = top_ys[pick_index]
        if (valid_strips < 3 and valid_strips >= 2 and
                top_ys[1] - top_ys[0] > CUT_SINGLE_STRIP_MAX_LEAD):
            top_y_pick = top_ys[1]

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_y = top_y_pick
            dynamic_cut_valid = True
        else:
            delta = top_y_pick - dynamic_cut_y
            if delta < -CUT_MAX_STEP_UP:
                top_y_pick = dynamic_cut_y - CUT_MAX_STEP_UP
            elif delta > CUT_MAX_STEP_DOWN:
                top_y_pick = dynamic_cut_y + CUT_MAX_STEP_DOWN
            dynamic_cut_y = int(
                CUT_EMA_ALPHA * top_y_pick +
                (1.0 - CUT_EMA_ALPHA) * dynamic_cut_y)
        dynamic_cut_y = clamp_int(dynamic_cut_y, 0, CUT_SCAN_Y_MAX)
    else:
        dynamic_cut_miss_count += 1
        if dynamic_cut_miss_count > CUT_MAX_MISS:
            dynamic_cut_valid = False
            dynamic_cut_y = 0

def yellow_blob_below_cut(blob):
    if not ENABLE_DYNAMIC_CUT or not dynamic_cut_valid:
        return True
    return blob.y() + blob.h() >= dynamic_cut_y + CUT_BLOB_DELTA


def detect_return_yellow_line(img):
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_YELLOW_ROI,
            pixels_threshold=RETURN_YELLOW_MIN_PIXELS,
            area_threshold=RETURN_YELLOW_MIN_AREA, merge=True)
    except Exception:
        return None, None
    if not blobs:
        return None, None

    # Top-to-bottom search: record the first yellow blob without shape checks.
    best_y = None
    best_top = 241
    best_pixels = -1
    best_blob = None
    for blob in blobs:
        if not yellow_blob_below_cut(blob):
            continue
        top = blob.y()
        y = blob.cy()
        pixels = blob.pixels()
        if (best_y is None or top < best_top or
                (top == best_top and pixels > best_pixels)):
            best_y = y
            best_top = top
            best_pixels = pixels
            best_blob = blob
    return best_y, best_blob


def return_line_overlaps_stop_roi(y):
    if y is None:
        return False
    roi_y = RETURN_STOP_ROI[1]
    roi_bottom = roi_y + RETURN_STOP_ROI[3] - 1
    return (y >= roi_y - RETURN_STOP_HORIZONTAL_GUARD and
            y <= roi_bottom + RETURN_STOP_HORIZONTAL_GUARD)


def detect_return_stop_line(img, return_y=None):
    if return_line_overlaps_stop_roi(return_y):
        return None, None
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_STOP_ROI,
            pixels_threshold=RETURN_STOP_MIN_PIXELS,
            area_threshold=RETURN_STOP_MIN_AREA, merge=True)
    except Exception:
        return None, None
    if not blobs:
        return None, None

    # Right-to-left search: record the first yellow blob without shape checks.
    best_x = None
    best_pixels = -1
    best_blob = None
    for blob in blobs:
        if not yellow_blob_below_cut(blob):
            continue
        w = blob.w()
        h = blob.h()
        if (h < RETURN_STOP_MIN_BLOB_H or
                w * 100 > h * RETURN_STOP_MAX_WIDTH_HEIGHT_X100):
            continue
        x = blob.cx()
        pixels = blob.pixels()
        if (best_x is None or x > best_x or
                (x == best_x and pixels > best_pixels)):
            best_x = x
            best_pixels = pixels
            best_blob = blob
    return best_x, best_blob


def update_return_state(raw_y, raw_stop_x):
    global return_yellow_last_y, return_yellow_stable_count
    global return_yellow_detected, return_yellow_y
    global return_stop_x, return_stop_requested

    return_stop_x = raw_stop_x if raw_stop_x is not None else -1
    if raw_stop_x is not None and raw_stop_x > RETURN_STOP_X_THRESHOLD:
        return_stop_requested = True

    if raw_y is None:
        return_yellow_last_y = -1
        return_yellow_stable_count = 0
        return_yellow_detected = False
        return_yellow_y = 0
    else:
        if (return_yellow_last_y >= 0 and
                abs(raw_y - return_yellow_last_y) <= RETURN_YELLOW_STABLE_DELTA):
            return_yellow_stable_count += 1
        else:
            return_yellow_stable_count = 1
        return_yellow_last_y = raw_y
        if return_yellow_stable_count >= RETURN_YELLOW_STABLE_FRAMES:
            return_yellow_detected = True
            return_yellow_y = raw_y
        else:
            return_yellow_detected = False
            return_yellow_y = 0

    status = RETURN_STATUS_STOP if return_stop_requested else 0x00
    if return_yellow_detected:
        status |= RETURN_STATUS_Y_VALID
    return status


_tx_buf = bytearray(7)
_tx_buf[0] = 0xAA
_tx_buf[1] = 0x55
_tx_buf[2] = RETURN_YELLOW_PACKET_ID


def send_return_packet(status):
    if uart is None:
        return
    y = return_yellow_y if status & RETURN_STATUS_Y_VALID else 0
    y = clamp_int(int(y), 0, 239)
    _tx_buf[3] = status & 0xFF
    _tx_buf[4] = y & 0xFF
    _tx_buf[5] = (y >> 8) & 0xFF
    _tx_buf[6] = (_tx_buf[2] + _tx_buf[3] +
                  _tx_buf[4] + _tx_buf[5]) & 0xFF
    uart.write(_tx_buf)


def draw_detection(img, raw_y, y_blob, raw_stop_x, stop_blob, status):
    for roi in CUT_STRIP_ROIS:
        img.draw_rectangle(roi, color=CUT_SAMPLE_COLOR, thickness=1)
    if dynamic_cut_valid:
        img.draw_line(0, dynamic_cut_y, 319, dynamic_cut_y,
                      color=CUT_LINE_COLOR, thickness=2)
    img.draw_rectangle(RETURN_YELLOW_ROI, color=ROI_Y_COLOR, thickness=1)
    img.draw_rectangle(RETURN_STOP_ROI, color=ROI_STOP_COLOR, thickness=1)
    img.draw_line(RETURN_STOP_X_THRESHOLD, 0, RETURN_STOP_X_THRESHOLD, 239,
                  color=THRESHOLD_COLOR, thickness=1)

    if raw_y is not None:
        line_color = (VALID_LINE_COLOR if status & RETURN_STATUS_Y_VALID
                      else PENDING_LINE_COLOR)
        img.draw_line(0, raw_y, 319, raw_y, color=line_color, thickness=2)
        if y_blob is not None:
            img.draw_rectangle(y_blob.rect(), color=line_color, thickness=2)
            img.draw_cross(y_blob.cx(), y_blob.cy(),
                           color=line_color, size=5, thickness=1)

    if raw_stop_x is not None:
        stop_color = (TRIGGER_STOP_COLOR
                      if raw_stop_x > RETURN_STOP_X_THRESHOLD
                      else TRACK_STOP_COLOR)
        img.draw_line(raw_stop_x, 0, raw_stop_x, 239,
                      color=stop_color, thickness=3)
        if stop_blob is not None:
            img.draw_rectangle(stop_blob.rect(), color=stop_color, thickness=2)
            img.draw_cross(stop_blob.cx(), stop_blob.cy(),
                           color=stop_color, size=5, thickness=1)

    y_text = "-" if raw_y is None else str(raw_y)
    if return_line_overlaps_stop_roi(raw_y):
        x_text = "guard"
    else:
        x_text = "-" if raw_stop_x is None else str(raw_stop_x)
    stable_display = return_yellow_stable_count
    if stable_display > RETURN_YELLOW_STABLE_FRAMES:
        stable_display = RETURN_YELLOW_STABLE_FRAMES
    img.draw_string(2, 2, "{} Y:{} {}/{}".format(
        CAMERA_ROLE, y_text, stable_display, RETURN_YELLOW_STABLE_FRAMES),
        color=TEXT_COLOR, scale=1)
    img.draw_string(2, 14, "X:{} status:{:02X}".format(x_text, status),
                    color=TEXT_COLOR, scale=1)
    cut_text = str(dynamic_cut_y) if dynamic_cut_valid else "-"
    img.draw_string(2, 26, "cut:{}".format(cut_text),
                    color=TEXT_COLOR, scale=1)
    if status & RETURN_STATUS_STOP:
        img.draw_string(245, 2, "STOP", color=TRIGGER_STOP_COLOR, scale=1)


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(True)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
loaded_exposure_us, loaded_ground_threshold = load_test_calibration()
if loaded_exposure_us is None:
    exposure_us = DEFAULT_EXPOSURE_US
    exposure_source = "default"
else:
    exposure_us = loaded_exposure_us
    exposure_source = CALIBRATION_FILE
if loaded_ground_threshold is None:
    BLUE_GROUND_THRESHOLD = DEFAULT_BLUE_GROUND_THRESHOLD
    ground_source = "default"
else:
    BLUE_GROUND_THRESHOLD = [loaded_ground_threshold]
    ground_source = CALIBRATION_FILE
sensor.set_auto_exposure(False, exposure_us=exposure_us)
sensor.skip_frames(time=500)

uart = None
if ENABLE_UART_OUTPUT:
    from machine import UART
    uart = UART(12, baudrate=115200)

clock = time.clock()
frame_count = 0
last_print = time.ticks_ms()

print("[return_test] role={} threshold={}".format(
    CAMERA_ROLE, RETURN_YELLOW_THRESHOLD[0]))
print("[return_test] exposure_us={} source={}".format(
    exposure_us, exposure_source))
print("[return_test] blue_ground={} source={}".format(
    BLUE_GROUND_THRESHOLD[0], ground_source))
print("[return_test] vertical strip: top->bottom, x=150..169, y=30..239")
print("[return_test] horizontal strip: right->left, y=200..219")
print("[return_test] stop when cx>{}; uart={}".format(
    RETURN_STOP_X_THRESHOLD, ENABLE_UART_OUTPUT))

while True:
    clock.tick()
    frame_count += 1
    img = snapshot_frame()
    update_dynamic_cut(img, frame_count)

    raw_y, y_blob = detect_return_yellow_line(img)
    raw_stop_x, stop_blob = detect_return_stop_line(img, raw_y)
    status = update_return_state(raw_y, raw_stop_x)
    send_return_packet(status)
    draw_detection(img, raw_y, y_blob, raw_stop_x, stop_blob, status)

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 500:
        last_print = now
        print("[return_test] frame={} cut={} raw_y={} stable={}/{} y={} x={} stop={} status=0x{:02X} fps={:.1f}".format(
            frame_count, dynamic_cut_y if dynamic_cut_valid else None,
            raw_y, return_yellow_stable_count,
            RETURN_YELLOW_STABLE_FRAMES, return_yellow_y, raw_stop_x,
            return_stop_requested, status, clock.fps()))

    if frame_count % 10 == 0:
        gc.collect()
