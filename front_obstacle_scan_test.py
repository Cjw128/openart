# ======================================================================
# front_obstacle_scan_test.py - OpenART front color-obstacle preview
# ======================================================================
# Run this from OpenART/OpenMV IDE. It uses the same color thresholds and
# basic filters as the runtime pre-carry 0x06 scan, then draws and prints
# candidate front obstacles.
#
# Output:
#   mask bit0-bit4 => color ID 1-5
#   only blobs with pixels > FRONT_SCAN_MIN_PIXELS are reported
# ======================================================================

import sensor, time

CALIBRATION_FILE = '/sd/color_thr.txt'

SOFTWARE_HMIRROR = True
RUNTIME_LENS_CORR = False
WB_GAINS = (101.00, 64.00, 97.00)
EXPOSURE_US = 1200

COLOR_SEARCH_ORDER = [1, 2, 3, 4, 5]
all_color_thresholds = [
    (34, 100, -41, 4, -72, -22),
    (10, 80, 22, 122, -17, 93),
    (50, 100, -128, -27, 20, 127),
    (21, 52, -77, 25, 6, 99),
    (51, 100, -5, 5, -38, 18),
]

DRAW_COLORS = {
    1: (0, 170, 255),
    2: (255, 0, 0),
    3: (0, 255, 0),
    4: (160, 96, 32),
    5: (255, 255, 255),
}

DETECT_Y_MIN = 8
DETECT_Y_MAX = 150
DETECT_ROI = (0, DETECT_Y_MIN, 320, DETECT_Y_MAX - DETECT_Y_MIN)
COLOR_MIN_PIXELS = 70
COLOR_MIN_AREA = 100
TENNIS_MIN_PIXELS = 80
TENNIS_MIN_AREA = 80
NEAR_NOISE_Y_MIN = 170
NEAR_NOISE_BOX_AREA = 400
COLOR_BLOB_LIMITS = (
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
)
MULTICOLOR_MIN_PIXELS = min(COLOR_MIN_PIXELS, TENNIS_MIN_PIXELS)
MULTICOLOR_MIN_AREA = min(COLOR_MIN_AREA, TENNIS_MIN_AREA)
FRONT_SCAN_MIN_PIXELS = 60
IGNORE_BOTTOMMOST_CARRY_BLOB = True
SEARCH_ROI_DRAW_COLOR = (255, 255, 0)

ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = [(0, 55, -30, 45, -90, -7)]
CUT_BLOB_MIN_H = 12  # Top-down: first continuous dark-blue run must be at least 12 px high.
CUT_LEFT_X = 0
CUT_RIGHT_X = 320
CUT_STRIP_XS = (10, 85, 160, 235, 310)
CUT_MIN_VALID_STRIPS = 2
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
CUT_ROI_Y_OFFSET = -10
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

dynamic_cut_left_y = DETECT_Y_MIN
dynamic_cut_right_y = DETECT_Y_MIN
dynamic_cut_valid = False
dynamic_cut_miss_count = 0
dynamic_detect_roi = DETECT_ROI


def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def load_calibrated_params(path=CALIBRATION_FILE):
    try:
        rows = {}
        exposure = None
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('exposure_us='):
                    try:
                        exposure = int(line.split('=', 1)[1])
                    except Exception:
                        exposure = None
                    continue
                if line.startswith('ground=') or line.startswith('ground2='):
                    continue
                parts = line.split(',')
                if len(parts) == 7:
                    slot = int(parts[0])
                    values = tuple(int(p) for p in parts[1:])
                elif len(parts) == 6:
                    slot = len(rows) + 1
                    values = tuple(int(p) for p in parts)
                else:
                    continue
                if 1 <= slot <= 5 and len(values) == 6:
                    rows[slot] = values
        if len(rows) == 5:
            return [rows[i] for i in range(1, 6)], exposure, 'loaded'
        return None, None, 'incomplete'
    except Exception:
        pass
    return None, None, 'missing_or_invalid'


def snapshot_frame(apply_lens_corr=False):
    img = sensor.snapshot()
    if apply_lens_corr:
        img = img.lens_corr(2)
    if SOFTWARE_HMIRROR:
        img = img.replace(hmirror=True)
    return img


def pick_top_y_from_strip(blobs):
    # Pick the smallest valid y, equivalent to scanning each strip from top to bottom.
    if not blobs:
        return None
    top_y = None
    for b in blobs:
        if b.h() < CUT_BLOB_MIN_H:
            continue
        if top_y is None or b.y() < top_y:
            top_y = b.y()
    return top_y


def update_dynamic_cut(img, frame_count):
    global dynamic_cut_left_y, dynamic_cut_right_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi

    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    top_y_sum = 0
    valid_strips = 0
    for roi in CUT_STRIP_ROIS:
        try:
            blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                                   pixels_threshold=CUT_MIN_PIXELS,
                                   area_threshold=CUT_MIN_AREA, merge=True)
        except Exception:
            blobs = None
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            valid_strips += 1
            top_y_sum += ty

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        top_y_average = top_y_sum // valid_strips
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = top_y_average
            dynamic_cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            dynamic_cut_left_y = int(a * top_y_average + (1.0 - a) * dynamic_cut_left_y)
        dynamic_cut_left_y = clamp_int(dynamic_cut_left_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
        dynamic_cut_right_y = dynamic_cut_left_y
    else:
        dynamic_cut_miss_count += 1
        if dynamic_cut_miss_count > CUT_MAX_MISS:
            dynamic_cut_valid = False
            dynamic_cut_left_y = DETECT_Y_MIN
            dynamic_cut_right_y = DETECT_Y_MIN

    if dynamic_cut_valid:
        y_base = clamp_int(dynamic_cut_left_y + CUT_ROI_Y_OFFSET,
                           DETECT_Y_MIN, DETECT_Y_MAX - 1)
    else:
        y_base = DETECT_Y_MIN
    dynamic_detect_roi = (0, y_base, 320, DETECT_Y_MAX - y_base)


def cut_line_y_at_x(x):
    return dynamic_cut_left_y


def valid_front_scan_blob(blob, color_id, pixels_threshold_override=0):
    # Match 0x06: colored obstacles are valid regardless of target aspect ratio.
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    box_area = w * h
    if blob.y() > NEAR_NOISE_Y_MIN and box_area < NEAR_NOISE_BOX_AREA:
        return False
    pixels_threshold, area_threshold = COLOR_BLOB_LIMITS[color_id - 1]
    if pixels_threshold_override > 0:
        pixels_threshold = pixels_threshold_override
    if blob.pixels() < pixels_threshold or box_area < area_threshold:
        return False
    density_minimum = (0.25 if color_id == 3 or color_id == 4 or color_id == 5
                       else 0.40)
    return blob.density() >= density_minimum


def color_id_from_blob_code(code):
    if code <= 0 or code & (code - 1):
        return 0
    color_id = 1
    while code > 1:
        code >>= 1
        color_id += 1
    return color_id if color_id <= len(all_color_thresholds) else 0


def find_front_obstacles(img):
    candidates = []
    mask = 0
    roi = dynamic_detect_roi
    try:
        blobs = img.find_blobs(all_color_thresholds, roi=roi,
                               pixels_threshold=FRONT_SCAN_MIN_PIXELS,
                               area_threshold=MULTICOLOR_MIN_AREA,
                               merge=False)
    except Exception:
        blobs = None
    if blobs:
        for b in blobs:
            color_id = color_id_from_blob_code(b.code())
            if color_id <= 0:
                continue
            if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
                # A target may cross the red line; discard it only when fully above.
                if b.y() + b.h() < cut_line_y_at_x(b.cx()) + CUT_BLOB_DELTA:
                    continue
            if b.pixels() <= FRONT_SCAN_MIN_PIXELS:
                continue
            if not valid_front_scan_blob(
                    b, color_id, FRONT_SCAN_MIN_PIXELS + 1):
                continue
            candidates.append((color_id, b))

    ignored = None
    if IGNORE_BOTTOMMOST_CARRY_BLOB and candidates:
        for color_id, b in candidates:
            if ignored is None:
                ignored = (color_id, b)
                continue
            _, ib = ignored
            b_bottom = b.y() + b.h()
            ib_bottom = ib.y() + ib.h()
            if b_bottom > ib_bottom:
                ignored = (color_id, b)
            elif b_bottom == ib_bottom and b.cy() > ib.cy():
                ignored = (color_id, b)

    best_by_color = {}
    for color_id, b in candidates:
        if ignored is not None and color_id == ignored[0] and b is ignored[1]:
            continue
        best = best_by_color.get(color_id)
        if best is None or b.pixels() > best.pixels():
            best_by_color[color_id] = b

    result = []
    for color_id in COLOR_SEARCH_ORDER:
        best = best_by_color.get(color_id)
        if best is not None:
            mask |= 1 << (color_id - 1)
            result.append((color_id, best))
    return mask, result, ignored


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(True)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
sensor.skip_frames(time=500)

_loaded, _exp, _source = load_calibrated_params()
if _loaded:
    all_color_thresholds = _loaded
    if _exp is not None:
        sensor.set_auto_exposure(False, exposure_us=_exp)
    print('[front_test] loaded /sd/color_thr.txt exposure={}'.format(_exp))
else:
    print('[front_test] using built-in thresholds ({})'.format(_source))

clock = time.clock()
frame_count = 0
last_print = time.ticks_ms()

print('[front_test] pixels>{}, roi_y={}..{}, dynamic_cut={}, ignore_bottommost={}'.format(
    FRONT_SCAN_MIN_PIXELS, DETECT_Y_MIN, DETECT_Y_MAX - 1,
    ENABLE_DYNAMIC_CUT, IGNORE_BOTTOMMOST_CARRY_BLOB))

while True:
    clock.tick()
    frame_count += 1
    img = snapshot_frame(apply_lens_corr=RUNTIME_LENS_CORR)
    update_dynamic_cut(img, frame_count)

    mask, obstacles, ignored = find_front_obstacles(img)

    img.draw_rectangle(dynamic_detect_roi, color=SEARCH_ROI_DRAW_COLOR, thickness=1)

    if dynamic_cut_valid:
        img.draw_line(CUT_LEFT_X, dynamic_cut_left_y, CUT_RIGHT_X, dynamic_cut_right_y,
                      color=(255, 0, 0), thickness=2)

    for color_id, blob in obstacles:
        color = DRAW_COLORS.get(color_id, (255, 255, 255))
        img.draw_rectangle(blob.rect(), color=color, thickness=2)
        img.draw_cross(blob.cx(), blob.cy(), color=color, size=6, thickness=1)
        img.draw_string(blob.x(), max(0, blob.y() - 12),
                        'id{} p{}'.format(color_id, blob.pixels()),
                        color=color, scale=1)

    if ignored is not None:
        color_id, blob = ignored
        img.draw_rectangle(blob.rect(), color=(128, 128, 128), thickness=1)
        img.draw_cross(blob.cx(), blob.cy(), color=(128, 128, 128), size=6, thickness=1)
        img.draw_string(blob.x(), min(228, blob.y() + blob.h() + 2),
                        'carry id{}'.format(color_id),
                        color=(128, 128, 128), scale=1)

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 500:
        last_print = now
        print('[front_test] frame={} mask=0x{:02x} count={} fps={:.1f}'.format(
            frame_count, mask, len(obstacles), clock.fps()))
        if ignored is not None:
            color_id, blob = ignored
            print('  ignored_carry id={} rect=({}, {}, {}, {}) pixels={} density={:.2f}'.format(
                color_id, blob.x(), blob.y(), blob.w(), blob.h(),
                blob.pixels(), blob.density()))
        for color_id, blob in obstacles:
            print('  id={} rect=({}, {}, {}, {}) pixels={} density={:.2f}'.format(
                color_id, blob.x(), blob.y(), blob.w(), blob.h(),
                blob.pixels(), blob.density()))
