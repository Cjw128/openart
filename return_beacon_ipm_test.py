import sensor
import time
from machine import UART


# Standalone OpenART test script for return-to-depot beacon detection.
# It uses color blob detection and the same front-view IPM calibration style
# as openart/main.py, then reports beacon world coordinates.


UART_ENABLED = True
UART_ID = 12
UART_BAUD = 115200
BEACON_ID = 1

WIDTH = 320
HEIGHT = 240
CENTER_X = WIDTH // 2

BEACON_THRESHOLD = [(79, 95, 5, 65, -54, 73)]

DETECT_Y_MIN = 20
DETECT_ROI = (0, DETECT_Y_MIN, WIDTH, HEIGHT - DETECT_Y_MIN)

MIN_PIXELS = 100
MIN_AREA = 100
MERGE_BLOBS = True

ENABLE_ASPECT_RATIO_FILTER = True
MIN_ASPECT_RATIO = 0.10
MAX_ASPECT_RATIO = 3.50
MIN_DENSITY = 0.20

ENABLE_LOCAL_TRACK_ROI = True
TRACK_MARGIN_X_RATIO = 0.50
TRACK_MARGIN_Y_RATIO = 0.50
TRACK_MARGIN_X_MIN = 12
TRACK_MARGIN_Y_MIN = 12
TRACK_MARGIN_X_MAX = 90
TRACK_MARGIN_Y_MAX = 70
TRACK_MIN_ROI_W = 16
TRACK_MIN_ROI_H = 16
TRACK_MAX_LOST = 10

WB_FIXED = True
WB_GAINS = (101.00, 64.00, 97.00)
EXPOSURE_US = 1000
GAIN_DB = 0
LENS_CORR_STRENGTH = 2


CALIB_PIXEL = [
    [85, 240],
    [267, 240],
    [125, 129],
    [219, 129],
]

CALIB_WORLD = [
    [-7.5, 7.5],
    [7.5, 7.5],
    [-7.5, 22.5],
    [7.5, 22.5],
]


def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def mat_solve_8x8(A, B):
    n = 8
    m = [[A[i][j] for j in range(n)] for i in range(n)]
    b = [B[i] for i in range(n)]

    for col in range(n):
        max_val = abs(m[col][col])
        max_row = col
        for row in range(col + 1, n):
            if abs(m[row][col]) > max_val:
                max_val = abs(m[row][col])
                max_row = row

        if max_row != col:
            m[col], m[max_row] = m[max_row], m[col]
            b[col], b[max_row] = b[max_row], b[col]

        if abs(m[col][col]) < 1e-10:
            return None

        for row in range(col + 1, n):
            factor = m[row][col] / m[col][col]
            for j in range(col, n):
                m[row][j] -= factor * m[col][j]
            b[row] -= factor * b[col]

    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = b[i]
        for j in range(i + 1, n):
            x[i] -= m[i][j] * x[j]
        x[i] /= m[i][i]
    return x


def calc_homography(pixels, world):
    A = []
    B = []
    for i in range(4):
        px = float(pixels[i][0])
        py = float(pixels[i][1])
        wx = float(world[i][0])
        wy = float(world[i][1])
        A.append([px, py, 1, 0, 0, 0, -wx * px, -wx * py])
        A.append([0, 0, 0, px, py, 1, -wy * px, -wy * py])
        B.append(wx)
        B.append(wy)

    h = mat_solve_8x8(A, B)
    if h is None:
        return None

    return [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0],
    ]


def pixel_to_world(px, py, H):
    px = float(px)
    py = float(py)
    w = H[2][0] * px + H[2][1] * py + H[2][2]
    if abs(w) < 1e-10:
        return (0.0, 0.0)
    X = (H[0][0] * px + H[0][1] * py + H[0][2]) / w
    Y = (H[1][0] * px + H[1][1] * py + H[1][2]) / w
    return (X, Y)


def box_to_world(x, y, w, h, H):
    if H is None:
        return (0.0, 0.0)

    corners = [
        (x, y),
        (x + w, y),
        (x, y + h),
        (x + w, y + h),
    ]
    wx_sum = 0.0
    wy_sum = 0.0
    for px, py in corners:
        wx, wy = pixel_to_world(px, py, H)
        wx_sum += wx
        wy_sum += wy
    return (wx_sum / 4.0, wy_sum / 4.0)


def clamp_roi_to_frame(x, y, w, h):
    y_min_limit = DETECT_Y_MIN
    x0 = clamp_int(x, 0, WIDTH - 1)
    y0 = clamp_int(y, y_min_limit, HEIGHT - 1)
    x1 = clamp_int(x + w, x0 + 1, WIDTH)
    y1 = clamp_int(y + h, y0 + 1, HEIGHT)

    if (x1 - x0) < TRACK_MIN_ROI_W:
        if x0 + TRACK_MIN_ROI_W <= WIDTH:
            x1 = x0 + TRACK_MIN_ROI_W
        else:
            x0 = WIDTH - TRACK_MIN_ROI_W
            x1 = WIDTH

    if (y1 - y0) < TRACK_MIN_ROI_H:
        if y0 + TRACK_MIN_ROI_H <= HEIGHT:
            y1 = y0 + TRACK_MIN_ROI_H
        else:
            y0 = HEIGHT - TRACK_MIN_ROI_H
            y1 = HEIGHT
        if y0 < y_min_limit:
            y0 = y_min_limit
            y1 = min(HEIGHT, y0 + TRACK_MIN_ROI_H)

    return (x0, y0, x1 - x0, y1 - y0)


def make_roi_from_box(box):
    if (not ENABLE_LOCAL_TRACK_ROI) or box is None:
        return DETECT_ROI

    x, y, w, h = box
    mx = clamp_int(int(w * TRACK_MARGIN_X_RATIO), TRACK_MARGIN_X_MIN, TRACK_MARGIN_X_MAX)
    my = clamp_int(int(h * TRACK_MARGIN_Y_RATIO), TRACK_MARGIN_Y_MIN, TRACK_MARGIN_Y_MAX)
    return clamp_roi_to_frame(x - mx, y - my, w + mx * 2, h + my * 2)


def valid_beacon_blob(blob):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False

    if ENABLE_ASPECT_RATIO_FILTER:
        aspect = w / h
        if aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
            return False

    if blob.density() < MIN_DENSITY:
        return False

    return True


def center_dist2(blob, box):
    bx = box[0] + box[2] // 2
    by = box[1] + box[3] // 2
    dx = blob.cx() - bx
    dy = blob.cy() - by
    return dx * dx + dy * dy


def find_beacon_blob(img, last_box):
    roi = make_roi_from_box(last_box)
    blobs = img.find_blobs(
        BEACON_THRESHOLD,
        roi=roi,
        pixels_threshold=MIN_PIXELS,
        area_threshold=MIN_AREA,
        merge=MERGE_BLOBS,
    )

    if not blobs:
        return None

    candidates = []
    for blob in blobs:
        if valid_beacon_blob(blob):
            candidates.append(blob)

    if not candidates:
        return None

    if last_box is not None:
        return max(candidates, key=lambda b: b.pixels() - center_dist2(b, last_box) // 20)
    return max(candidates, key=lambda b: b.pixels())


def send_world_data(uart, beacon_id, wx_mm, wy_mm, pixel_w):
    data = bytearray(13)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = beacon_id & 0xFF
    data[3] = wx_mm & 0xFF
    data[4] = (wx_mm >> 8) & 0xFF
    data[5] = wy_mm & 0xFF
    data[6] = (wy_mm >> 8) & 0xFF
    data[7] = pixel_w & 0xFF
    data[8] = (pixel_w >> 8) & 0xFF
    data[9] = 0x00
    data[10] = 0x00
    data[11] = 0x00
    data[12] = sum(data[2:12]) & 0xFF
    uart.write(data)


def send_no_target(uart):
    data = bytearray(13)
    data[0] = 0xAA
    data[1] = 0x55
    data[12] = sum(data[2:12]) & 0xFF
    uart.write(data)


def configure_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_framerate(60)

    if WB_FIXED:
        sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
        sensor.skip_frames(time=500)
    else:
        sensor.set_auto_whitebal(True)
        sensor.skip_frames(time=1500)
        sensor.set_auto_whitebal(False)

    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
    sensor.skip_frames(time=300)


H_PIX2WORLD = calc_homography(CALIB_PIXEL, CALIB_WORLD)

configure_sensor()
uart = UART(UART_ID, baudrate=UART_BAUD) if UART_ENABLED else None
clock = time.clock()

last_box = None
lost_frames = 0
frame_id = 0
last_print_ms = time.ticks_ms()

print("=" * 50)
print("Return beacon IPM test")
print("threshold =", BEACON_THRESHOLD)
print("roi       =", DETECT_ROI)
print("uart      = UART{} {}".format(UART_ID, UART_BAUD) if UART_ENABLED else "uart      = disabled")
print("H status  =", "OK" if H_PIX2WORLD else "ERROR")
print("=" * 50)

while True:
    clock.tick()
    frame_id += 1

    img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)
    blob = find_beacon_blob(img, last_box)

    if blob is not None and H_PIX2WORLD is not None:
        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()
        last_box = (x, y, w, h)
        lost_frames = 0

        world_x, world_y = box_to_world(x, y, w, h, H_PIX2WORLD)
        center_x, center_y = pixel_to_world(cx, cy, H_PIX2WORLD)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)

        if UART_ENABLED:
            send_world_data(uart, BEACON_ID, wx_mm, wy_mm, w)

        img.draw_rectangle(blob.rect(), color=(0, 255, 255), thickness=2)
        img.draw_cross(cx, cy, color=(255, 0, 0), size=8, thickness=2)
        img.draw_string(
            x,
            max(0, y - 16),
            "w=({:.1f},{:.1f})cm".format(world_x, world_y),
            color=(0, 255, 255),
            scale=1,
        )

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 200:
            print(
                "[{}] beacon wx={:.1f} wy={:.1f}cm center=({:.1f},{:.1f})cm "
                "pixel=({},{}) box=({},{},{},{}) pixels={} density={:.2f} fps={:.1f}".format(
                    frame_id,
                    world_x,
                    world_y,
                    center_x,
                    center_y,
                    cx,
                    cy,
                    x,
                    y,
                    w,
                    h,
                    blob.pixels(),
                    blob.density(),
                    clock.fps(),
                )
            )
            last_print_ms = now
    else:
        lost_frames += 1
        if lost_frames > TRACK_MAX_LOST:
            last_box = None
        if UART_ENABLED:
            send_no_target(uart)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            print("[{}] beacon none fps={:.1f}".format(frame_id, clock.fps()))
            last_print_ms = now
