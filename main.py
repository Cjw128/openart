# ======================================================================
# OpenART Plus front camera - multi-target object detection
# ======================================================================


import sensor, image, time, math, gc
from machine import UART
try:
    from machine import WDT
except Exception:
    WDT = None
from yellow_crossline_ipm import create_crossline_ipm

# ======================================================================
# Mode selection
# ======================================================================
CALIBRATION_MODE = False# True = IPM calibration mode, False = normal detection
BIRDVIEW_DEBUG = False     # True = show bird-view image in IDE, False = fast detection
IS_SLAVE_CAR = False       # False=master OpenART(UART12), True=slave OpenART(UART2)
SLAVE_MODE = IS_SLAVE_CAR  # True: color is controlled by host 0x03 command
SOFTWARE_HMIRROR = True   # Hardware keeps vflip; software only adds hmirror to avoid full-frame flip tearing.
RUNTIME_LENS_CORR = False  # Test switch: disable per-frame lens_corr to isolate frame-buffer pressure.
ENABLE_WATCHDOG = True
WATCHDOG_TIMEOUT_MS = 8000
ENABLE_SD_LOG = True
LOG_PATH = '/sd/watchdog.log'
LOG_INTERVAL_MS = 1000
LOG_FIRST_FRAMES = 10

wdt = None
last_log_ms = 0

def log_checkpoint(stage, frame=-1, force=False):
    global last_log_ms
    if not ENABLE_SD_LOG:
        return
    now = time.ticks_ms()
    if not force and time.ticks_diff(now, last_log_ms) < LOG_INTERVAL_MS:
        return
    last_log_ms = now
    try:
        free = gc.mem_free() if hasattr(gc, 'mem_free') else -1
    except Exception:
        free = -1
    try:
        with open(LOG_PATH, 'a') as f:
            f.write('{} ms frame={} stage={} free={}\n'.format(now, frame, stage, free))
    except Exception:
        pass

def init_watchdog():
    global wdt
    if not ENABLE_WATCHDOG or WDT is None:
        return
    try:
        wdt = WDT(timeout=WATCHDOG_TIMEOUT_MS)
    except Exception:
        wdt = None

def feed_watchdog():
    if wdt is None:
        return
    try:
        wdt.feed()
    except Exception:
        pass

log_checkpoint('boot', force=True)
init_watchdog()

# ======================================================================
# Hardware initialization
# ======================================================================

# Camera initialization
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.set_framerate(60)
# Firmware may only keep one hardware flip; keep vertical in hardware and add hmirror in snapshot_frame().
sensor.set_hmirror(False)
sensor.set_vflip(True)

def snapshot_frame(apply_lens_corr=False):
    img = sensor.snapshot()
    if apply_lens_corr:
        img = img.lens_corr(2)
    if SOFTWARE_HMIRROR:
        img = img.replace(hmirror=True)
    return img

# White balance configuration
# True = fixed gains for competition, False = auto-converge then lock for tuning
WB_FIXED = True
# Fixed white balance gains (R_db, G_db, B_db)
# Fill these after running wb_calibrate.py under competition lighting.
WB_GAINS = (101.00, 64.00, 97.00) # Fixed white balance

if WB_FIXED:
    sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
    sensor.skip_frames(time=500)
else:
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)

# ======================================================================
# Startup brightness calibration
# ======================================================================

TARGET_BRIGHTNESS = 45.0
EXPOSURE_MIN = 100
EXPOSURE_MAX = 4500
EXPOSURE_INIT = 1000
GAIN_INIT = 0
CALIBRATION_DELAY = 50
CALIBRATION_SETTLE_MS = 200
MAX_EXPOSURE_STEP_UP = 1.6
MAX_EXPOSURE_STEP_DOWN = 0.6

def set_exposure(exposure_us):
    sensor.set_auto_exposure(False, exposure_us=exposure_us)

def measure_brightness(roi=None):
    img = snapshot_frame()
    if roi:
        stats = img.get_statistics(roi=roi)
    else:
        stats = img.get_statistics()
    return stats.l_mean()

def measure_brightness_stable(samples=5, roi=None):
    total = 0.0
    for _ in range(samples):
        total += measure_brightness(roi)
        time.sleep_ms(CALIBRATION_DELAY)
    return total / samples

def calculate_exposure_adjustment(current_brightness, target_brightness, current_exposure):
    if current_brightness <= 0:
        return current_exposure
    ratio = target_brightness / current_brightness
    if ratio > MAX_EXPOSURE_STEP_UP:
        ratio = MAX_EXPOSURE_STEP_UP
    elif ratio < MAX_EXPOSURE_STEP_DOWN:
        ratio = MAX_EXPOSURE_STEP_DOWN
    new_exposure = int(current_exposure * ratio)
    return max(EXPOSURE_MIN, min(EXPOSURE_MAX, new_exposure))

def calibrate_brightness_startup(target=TARGET_BRIGHTNESS, samples=5, roi=None, max_iterations=3):

    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_INIT)
    sensor.set_auto_gain(False, gain_db=GAIN_INIT)

    exposure = EXPOSURE_INIT
    set_exposure(exposure)
    time.sleep_ms(CALIBRATION_SETTLE_MS)

    for _ in range(max_iterations):
        current_brightness = measure_brightness_stable(samples=samples, roi=roi)
        error = abs(target - current_brightness)
        if error < 3.0:
            break
        exposure = calculate_exposure_adjustment(current_brightness, target, exposure)
        set_exposure(exposure)
        time.sleep_ms(CALIBRATION_SETTLE_MS)

    if not WB_FIXED:
        sensor.skip_frames(time=1500)
        sensor.set_auto_whitebal(False)
    return exposure

# Fixed exposure
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)

# UART initialization (UART12, 115200 baud)
if IS_SLAVE_CAR:
    uart = UART(2, baudrate=115200)
else:
    uart = UART(12, baudrate=115200)

# FPS timer
clock = time.clock()

# ======================================================================
# LAB color thresholds - dynamic multi-color detection
# ======================================================================
# LAB format: (L_min, L_max, A_min, A_max, B_min, B_max)
# L: brightness (0-100)
# A: red-green axis (positive=red, negative=green)
# B: yellow-blue axis (positive=yellow, negative=blue)

# Supported color thresholds (fallback defaults; overridden by /sd/params.txt if present)
all_color_thresholds = [
    (34, 100, -41, 5, -72, -17),    # Color 1: light-blue bag
    (10, 80, 22, 122, -17, 93),     # Color 2: red bag
    (50, 100, -128, -27, 20, 127),  # Color 3: tennis ball
    (20, 55, -1, 30, 0, 50),        # Color 4: brown teddy bear; tune on field
    (53, 100, -10, 11, -11, 8)      # Color 5: white teddy bear; model-only in runtime
]

def _load_thresholds(path='/sd/params.txt'):
    try:
        result = []
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split(',')
                if len(parts) != 6:
                    continue
                result.append(tuple(int(p) for p in parts))
        if len(result) == 5:
            return result
    except Exception:
        pass
    return None

_loaded = _load_thresholds()
if _loaded:
    all_color_thresholds = _loaded

COLOR_SEARCH_ORDER = [1, 2, 3, 4, 5]

COLOR_LOST_FRAMES = 5
COLOR_TRACK_MARGIN = 45
COLOR_MIN_PIXELS = 100
COLOR_MIN_AREA = 100
TENNIS_COLOR_ID = 3
TENNIS_MIN_PIXELS = 45
TENNIS_MIN_AREA = 45

# True: use TFLite model for white bear. False: detect all targets by LAB color blobs.
USE_WHITE_BEAR_MODEL = True
WHITE_BEAR_COLOR_ID = 5
MODEL_PATH = '/sd/dataset_25000_blur_0.30.tflite'
MODEL_INPUT_SCALE = 0.75
MODEL_SCORE_THRESHOLD = 0.40
MODEL_FALLBACK_SCORE_THRESHOLD = 0.25
MODEL_LABEL_BEAR = 0
model_tf = None
model_net = None
model_ready = False
model_status_last_print = 0

# Blue floor threshold, example values that must be tuned on field.
# Focus on the B channel; blue is usually below -20.
# BLUE_GROUND_THRESHOLD = (10, 50, -20, 50, -77, -25)

# Dynamic active-threshold state
red_thresholds = all_color_thresholds  # Initial search uses all colors
active_threshold = None                 # Currently locked single-color threshold
active_color_id = 0                     # Locked color ID, 0 means unlocked

# Target-lost counters
lost_frame_count = 0                    # Consecutive frames without target
MAX_LOST_FRAMES = 30                    # Maximum lost frames, about 0.5s at 60 FPS
stable_detect_count = 0                 # Stable detection counter for color locking
STABLE_FRAMES_REQUIRED = 5              # Frames required before locking a color

# ======================================================================
# Recognition parameters
# ======================================================================
MIN_PIXELS = 30        # Blob pixel threshold
MIN_AREA = 80          # Minimum bounding-box area
MERGE_DISTANCE = False      # Merge threshold matches at the same position
DETECT_Y_MIN = 8           # Ignore image rows above this Y value
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)

# ======================================================================
# Dynamic cut line (based on blue-ground strips on left/right)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = [(32, 57, -52, 76, -108, -28)]
CUT_LEFT_X = 10
CUT_RIGHT_X = 310
CUT_STRIP_HALF_W = 2
CUT_SCAN_Y_MIN = 0
CUT_SCAN_Y_MAX = 140
CUT_UPDATE_INTERVAL = 2
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_Y_MARGIN = 6
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

# Local ROI tracking (lock-target fast path)
ENABLE_LOCAL_TRACK_ROI = True
TRACK_MARGIN_X_RATIO = 0.50
TRACK_MARGIN_Y_RATIO = 0.50
TRACK_MARGIN_X_MIN = 12
TRACK_MARGIN_Y_MIN = 12
TRACK_MARGIN_X_MAX = 90
TRACK_MARGIN_Y_MAX = 70
TRACK_MIN_ROI_W = 16
TRACK_MIN_ROI_H = 16
TRACK_MAX_JUMP_PX = 90
TRACK_AREA_CHANGE_MAX_PERCENT = 60

# Aspect-ratio filter to reduce false positives
ENABLE_ASPECT_RATIO_FILTER = True   # Enable aspect-ratio filtering
MIN_ASPECT_RATIO = 0.3            # Minimum width/height ratio
MAX_ASPECT_RATIO = 2.3             # Maximum width/height ratio

MIN_ROUNDNESS = 0.4                 # Roundness threshold; ball high, line very low
MIN_DENSITY = 0.6

last_target_cx = -1
last_target_cy = -1
dynamic_cut_left_y = DETECT_Y_MIN
dynamic_cut_right_y = DETECT_Y_MIN
dynamic_cut_valid = False
dynamic_cut_miss_count = 0
dynamic_detect_roi = DETECT_ROI
local_track_rect = None
last_tracked_pixels = -1
track_force_global_next = False
track_local_miss_count = 0
target_color_id = 0
color_track_active = False
color_track_box = None
color_track_color_id = 0
color_lost_count = 0
_cmd_rx_buf = bytearray()
crossline_angle_enabled = False
crossline_angle_result = None
# ======================================================================
# Yellow line detection parameters
# ======================================================================
yellow_threshold = [(56, 100, -31, 127, 46, 116)]    # Yellow LAB threshold
YELLOW_ROI_LEFT = (0, 80, 70, 160)       # Left vertical strip, extended to image bottom
YELLOW_ROI_RIGHT = (250, 80, 70, 160)    # Right vertical strip, extended to image bottom
YELLOW_DETECT_INTERVAL = 2              # Detect yellow line every N frames
YELLOW_ENTER_PIXELS = 10                # Pixel threshold for first yellow-line hit
YELLOW_KEEP_PIXELS = 3                  # Lower hold threshold after line is seen
YELLOW_SCAN_STRIP_H = 12                # Carry mode scans yellow ROI from bottom to top by strips
# ======================================================================
# Orange obstacle detection
# ======================================================================
obstacle_threshold = [(56, 96, 16, 127, 9, 127)]
OBSTACLE_ROI = (0, 80, 320, 160)
OBSTACLE_PATH_X_MIN = 110
OBSTACLE_PATH_X_MAX = 210
OBSTACLE_MIN_PIXELS = 80
OBSTACLE_MIN_AREA = 80

OBSTACLE_NONE = 0x00
OBSTACLE_MOVE_RIGHT = 0x01
OBSTACLE_MOVE_LEFT = 0x02
OBSTACLE_BLOCKED = 0x03
OBSTACLE_TARGET_OVERLAP_PIXELS = 200

yellow_boundary_y = 0       # Yellow boundary Y in image pixels
yellow_boundary_left_y = 0  # Left yellow-line center Y
yellow_boundary_right_y = 0 # Right yellow-line center Y
yellow_boundary_wy = 0.0    # Yellow boundary Y in world coordinates, cm
yellow_detected = False     # Whether yellow line is visible
yellow_tracking = False      # Hysteresis state after first yellow-line hit
yellow_lost_count = 0       # Consecutive yellow-line lost counter
YELLOW_LOST_THRESHOLD = 3   # Lost detections required before crossed-line decision
yellow_seen_in_carry = False # Whether yellow line was confirmed in carry mode
YELLOW_RECENT_DETECTIONS = 5 # Recent yellow-line latch window for carry occlusion
yellow_recent_count = 0

# State machine
MODE_SEARCH = 0
MODE_CARRY = 1
MODE_WAIT_TURN = 2
MODE_RETURN = 3

openart_mode = MODE_SEARCH   # 0=search, 1=carry, 2=wait for turn, 3=return

# Position flags
POS_NO_BOUNDARY = 0x00
POS_RIGHT_SIDE  = 0x01
POS_CROSSED     = 0x02

# Return-to-depot beacon detection. Uses the same IPM and UART packet as normal targets.
RETURN_BEACON_ID = 0x06
BEACON_THRESHOLD = [(52, 100, 4, 91, -14, 127)]
BEACON_DETECT_Y_MIN = 20
BEACON_DETECT_ROI = (0, BEACON_DETECT_Y_MIN, 320, 240 - BEACON_DETECT_Y_MIN)
BEACON_MIN_PIXELS = 40
BEACON_MIN_AREA = 40
BEACON_MERGE_BLOBS = True
BEACON_MIN_ASPECT_RATIO = 0.01
BEACON_MAX_ASPECT_RATIO = 20.00
BEACON_MIN_DENSITY = 0.10
BEACON_TRACK_MAX_LOST = 10
beacon_last_box = None
beacon_lost_frames = 0

def code_to_color_id(code):
    """将find_blobs返回的位掩码code映射为颜色ID(1/2/3...)。"""
    if code <= 0:
        return 0
    for i in range(len(all_color_thresholds)):
        if code & (1 << i):
            return i + 1
    return 0

def reset_target_tracking_state():
    """清空上一轮搬运留下的目标锁定状态，下一帧从全局重新找场地中央目标。"""
    global active_threshold, active_color_id, red_thresholds
    global lost_frame_count, stable_detect_count
    global local_track_rect, last_tracked_pixels, track_force_global_next, track_local_miss_count
    global last_target_cx, last_target_cy
    global target_color_id
    global color_track_active, color_track_box, color_track_color_id, color_lost_count

    active_threshold = None
    active_color_id = 0
    red_thresholds = all_color_thresholds
    lost_frame_count = 0
    stable_detect_count = 0
    local_track_rect = None
    last_tracked_pixels = -1
    track_force_global_next = False
    track_local_miss_count = 0
    last_target_cx = -1
    last_target_cy = -1
    target_color_id = 0
    color_track_active = False
    color_track_box = None
    color_track_color_id = 0
    color_lost_count = 0

def reset_yellow_state():
    """清空黄线状态，避免新一轮任务继承上一轮的边界/滞回。"""
    global yellow_lost_count, yellow_seen_in_carry, yellow_tracking, yellow_detected
    global yellow_recent_count
    global yellow_boundary_y, yellow_boundary_left_y, yellow_boundary_right_y, yellow_boundary_wy

    yellow_lost_count = 0
    yellow_seen_in_carry = False
    yellow_tracking = False
    yellow_detected = False
    yellow_recent_count = 0
    yellow_boundary_y = 0
    yellow_boundary_left_y = 0
    yellow_boundary_right_y = 0
    yellow_boundary_wy = 0.0

def reset_beacon_state():
    global beacon_last_box, beacon_lost_frames
    beacon_last_box = None
    beacon_lost_frames = 0

# ======================================================================
# Homography transform for inverse perspective mapping
# ======================================================================

def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def cut_line_y_at_x(x):
    dx = CUT_RIGHT_X - CUT_LEFT_X
    if dx == 0:
        return dynamic_cut_left_y
    return int(dynamic_cut_left_y + (dynamic_cut_right_y - dynamic_cut_left_y) * (x - CUT_LEFT_X) / dx)

def pick_top_y_from_strip(blobs):
    if not blobs:
        return None
    top_y = 240
    for b in blobs:
        by = b.y()
        if by < top_y:
            top_y = by
    return top_y

def update_dynamic_cut(img, frame_count):
    global dynamic_cut_left_y, dynamic_cut_right_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi

    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    strip_h = CUT_SCAN_Y_MAX - CUT_SCAN_Y_MIN
    left_roi = (CUT_LEFT_X - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN, CUT_STRIP_HALF_W * 2 + 1, strip_h)
    right_roi = (CUT_RIGHT_X - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN, CUT_STRIP_HALF_W * 2 + 1, strip_h)

    left_blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=left_roi,
                                pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
    right_blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=right_roi,
                                 pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)

    left_y_new = pick_top_y_from_strip(left_blobs)
    right_y_new = pick_top_y_from_strip(right_blobs)

    if left_y_new is not None and right_y_new is not None:
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = left_y_new
            dynamic_cut_right_y = right_y_new
            dynamic_cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            dynamic_cut_left_y = int(a * left_y_new + (1.0 - a) * dynamic_cut_left_y)
            dynamic_cut_right_y = int(a * right_y_new + (1.0 - a) * dynamic_cut_right_y)

        dynamic_cut_left_y = clamp_int(dynamic_cut_left_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
        dynamic_cut_right_y = clamp_int(dynamic_cut_right_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
    else:
        dynamic_cut_miss_count += 1
        if dynamic_cut_miss_count > CUT_MAX_MISS:
            dynamic_cut_valid = False
            dynamic_cut_left_y = DETECT_Y_MIN
            dynamic_cut_right_y = DETECT_Y_MIN

    if dynamic_cut_valid:
        y_base = min(dynamic_cut_left_y, dynamic_cut_right_y) - CUT_Y_MARGIN
        y_base = clamp_int(y_base, DETECT_Y_MIN, 239)
    else:
        y_base = DETECT_Y_MIN
    dynamic_detect_roi = (0, y_base, 320, 240 - y_base)

def clamp_roi_to_frame(x, y, w, h, y_min_limit):
    # Strict ROI clamping to avoid firmware ValueError/assert.
    y_min_limit = clamp_int(y_min_limit, 0, 239)
    if y_min_limit > 240 - TRACK_MIN_ROI_H:
        y_min_limit = 240 - TRACK_MIN_ROI_H

    x0 = clamp_int(x, 0, 319)
    y0 = clamp_int(y, y_min_limit, 239)
    x1 = clamp_int(x + w, x0 + 1, 320)
    y1 = clamp_int(y + h, y0 + 1, 240)

    if (x1 - x0) < TRACK_MIN_ROI_W:
        if x0 + TRACK_MIN_ROI_W <= 320:
            x1 = x0 + TRACK_MIN_ROI_W
        else:
            x0 = 320 - TRACK_MIN_ROI_W
            x1 = 320

    if (y1 - y0) < TRACK_MIN_ROI_H:
        if y0 + TRACK_MIN_ROI_H <= 240:
            y1 = y0 + TRACK_MIN_ROI_H
        else:
            y0 = 240 - TRACK_MIN_ROI_H
            y1 = 240
        if y0 < y_min_limit:
            y0 = y_min_limit
            y1 = min(240, y0 + TRACK_MIN_ROI_H)

    return (x0, y0, x1 - x0, y1 - y0)

def build_local_track_roi(base_roi):
    if local_track_rect is None:
        return base_roi

    rx, ry, rw, rh = local_track_rect
    mx = clamp_int(int(rw * TRACK_MARGIN_X_RATIO), TRACK_MARGIN_X_MIN, TRACK_MARGIN_X_MAX)
    my = clamp_int(int(rh * TRACK_MARGIN_Y_RATIO), TRACK_MARGIN_Y_MIN, TRACK_MARGIN_Y_MAX)

    x = rx - mx
    y = ry - my
    w = rw + mx * 2
    h = rh + my * 2

    y_floor = base_roi[1]
    if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
        y_floor = max(y_floor, min(dynamic_cut_left_y, dynamic_cut_right_y) - CUT_Y_MARGIN)

    return clamp_roi_to_frame(x, y, w, h, y_floor)

def box_iou(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    ax2 = ax1 + aw
    ay2 = ay1 + ah
    bx2 = bx1 + bw
    by2 = by1 + bh
    inter_w = min(ax2, bx2) - max(ax1, bx1)
    inter_h = min(ay2, by2) - max(ay1, by1)
    if inter_w <= 0 or inter_h <= 0:
        return 0.0
    inter = inter_w * inter_h
    union = aw * ah + bw * bh - inter
    if union <= 0:
        return 0.0
    return inter / union

def center_dist2(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    acx = ax1 + aw // 2
    acy = ay1 + ah // 2
    bcx = bx1 + bw // 2
    bcy = by1 + bh // 2
    dx = acx - bcx
    dy = acy - bcy
    return dx * dx + dy * dy

def make_roi_from_box(box):
    if not box:
        return dynamic_detect_roi
    x, y, w, h = box
    x0 = clamp_int(x - COLOR_TRACK_MARGIN, 0, 319)
    y0 = clamp_int(y - COLOR_TRACK_MARGIN, DETECT_Y_MIN, 239)
    x1 = clamp_int(x + w + COLOR_TRACK_MARGIN, x0 + 1, 320)
    y1 = clamp_int(y + h + COLOR_TRACK_MARGIN, y0 + 1, 240)
    return (x0, y0, x1 - x0, y1 - y0)

def threshold_items_for_color():
    items = []
    for color_id in COLOR_SEARCH_ORDER:
        if color_id < 1 or color_id > len(all_color_thresholds):
            continue
        if USE_WHITE_BEAR_MODEL and color_id == WHITE_BEAR_COLOR_ID:
            continue
        if target_color_id > 0 and color_id != target_color_id:
            continue
        items.append((color_id, all_color_thresholds[color_id - 1]))
    return items

def valid_color_blob(blob, color_id):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    aspect = w / h
    if color_id == 3:
        if aspect < 0.45 or aspect > 1.85:
            return False
        if blob.density() < 0.35:
            return False
    elif color_id == 4 or color_id == 5:
        if aspect < 0.30 or aspect > 2.50:
            return False
        if blob.pixels() < 120:
            return False
    else:
        if aspect < 0.60 or aspect > 1.80:
            return False
        if blob.density() < 0.50:
            return False
    return True

def color_blob_thresholds(color_id):
    if color_id == TENNIS_COLOR_ID:
        return (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA)
    return (COLOR_MIN_PIXELS, COLOR_MIN_AREA)

def pick_initial_color_candidate(candidates):
    # Prefer the object whose bounding-box bottom is closest to y=240.
    return min(candidates, key=lambda item: 240 - (item[1].y() + item[1].h()))

def find_color_target(img, last_box):
    items = threshold_items_for_color()
    if not items:
        return None
    roi = make_roi_from_box(last_box)
    candidates = []
    for color_id, threshold in items:
        color_candidates = []
        pixels_threshold, area_threshold = color_blob_thresholds(color_id)
        try:
            blobs = img.find_blobs([threshold], roi=roi,
                                   pixels_threshold=pixels_threshold,
                                   area_threshold=area_threshold,
                                   merge=True)
        except Exception:
            blobs = None
        if not blobs:
            continue
        for blob in blobs:
            if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
                if blob.cy() < cut_line_y_at_x(blob.cx()) + CUT_BLOB_DELTA:
                    continue
            if valid_color_blob(blob, color_id):
                item = (color_id, blob)
                candidates.append(item)
                color_candidates.append(item)
    if not candidates:
        return None
    if last_box:
        def score_item(item):
            b = item[1]
            b_box = (b.x(), b.y(), b.w(), b.h())
            return b.pixels() - center_dist2(b_box, last_box) // 20
        return max(candidates, key=score_item)
    return pick_initial_color_candidate(candidates)

def load_white_bear_model():
    global model_tf, model_net, model_ready
    if not USE_WHITE_BEAR_MODEL:
        model_tf = None
        model_net = None
        model_ready = False
        log_checkpoint('model_disabled', force=True)
        return
    log_checkpoint('model_load_start', force=True)
    try:
        import tf
        gc.collect()
        model_tf = tf
        model_net = tf.load(MODEL_PATH)
        model_ready = True
        log_checkpoint('model_load_done', force=True)
    except Exception as e:
        model_tf = None
        model_net = None
        model_ready = False
        log_checkpoint('model_load_fail', force=True)

def find_white_bear_model_target(img, last_box):
    global model_status_last_print
    if not USE_WHITE_BEAR_MODEL:
        return None
    if not model_ready or model_tf is None or model_net is None:
        now = time.ticks_ms()
        if time.ticks_diff(now, model_status_last_print) >= 1000:
            model_status_last_print = now
        return None

    detect_img = img
    if MODEL_INPUT_SCALE != 1.0:
        try:
            detect_img = img.copy(MODEL_INPUT_SCALE, 1)
        except Exception:
            return None

    best = None
    best_score = -1.0
    try:
        for obj in model_net.detect([detect_img], thresholds=[(MODEL_FALLBACK_SCORE_THRESHOLD, 1.0)]):
            label = obj.label()
            score = obj.score()
            if label != MODEL_LABEL_BEAR or score < MODEL_FALLBACK_SCORE_THRESHOLD:
                continue

            rx, ry, rw, rh = obj.rect()
            # Scale coordinates back to original image space if detect_img was downscaled.
            if detect_img is not img:
                rx = int(rx / MODEL_INPUT_SCALE)
                rw = int(rw / MODEL_INPUT_SCALE)
            if rw <= 0 or rh <= 0:
                continue

            if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
                if (ry + rh // 2) < cut_line_y_at_x(rx + rw // 2) + CUT_BLOB_DELTA:
                    continue

            cand_box = (rx, ry, rw, rh)
            rank = score
            if last_box:
                rank += box_iou(cand_box, last_box) * 0.4
                rank -= center_dist2(cand_box, last_box) / 200000.0
            if rank > best_score:
                best_score = rank
                best = (WHITE_BEAR_COLOR_ID, rx, ry, rw, rh, score)
    except Exception as e:
        best = None

    if detect_img is not img:
        detect_img = None
        gc.collect()
    return best

def beacon_roi_from_box(box):
    if (not ENABLE_LOCAL_TRACK_ROI) or box is None:
        return BEACON_DETECT_ROI

    x, y, w, h = box
    mx = clamp_int(int(w * TRACK_MARGIN_X_RATIO), TRACK_MARGIN_X_MIN, TRACK_MARGIN_X_MAX)
    my = clamp_int(int(h * TRACK_MARGIN_Y_RATIO), TRACK_MARGIN_Y_MIN, TRACK_MARGIN_Y_MAX)
    return clamp_roi_to_frame(x - mx, y - my, w + mx * 2, h + my * 2, BEACON_DETECT_Y_MIN)

def valid_beacon_blob(blob):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False

    aspect = w / h
    if aspect < BEACON_MIN_ASPECT_RATIO or aspect > BEACON_MAX_ASPECT_RATIO:
        return False
    if blob.density() < BEACON_MIN_DENSITY:
        return False
    return True

def beacon_center_dist2(blob, box):
    bx = box[0] + box[2] // 2
    by = box[1] + box[3] // 2
    dx = blob.cx() - bx
    dy = blob.cy() - by
    return dx * dx + dy * dy

def find_beacon_blob(img, last_box):
    roi = beacon_roi_from_box(last_box)
    blobs = img.find_blobs(BEACON_THRESHOLD, roi=roi,
                           pixels_threshold=BEACON_MIN_PIXELS,
                           area_threshold=BEACON_MIN_AREA,
                           merge=BEACON_MERGE_BLOBS)
    if not blobs:
        return None

    candidates = []
    for blob in blobs:
        if valid_beacon_blob(blob):
            candidates.append(blob)
    if not candidates:
        return None

    if last_box is not None:
        return max(candidates, key=lambda b: b.pixels() - beacon_center_dist2(b, last_box) // 20)
    return max(candidates, key=lambda b: b.pixels())

def box_to_world(x, y, w, h):
    if H_pix2world is None:
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
        wx, wy = pixel_to_world(px, py, H_pix2world)
        wx_sum += wx
        wy_sum += wy
    return (wx_sum / 4.0, wy_sum / 4.0)

def mat_solve_8x8(A, B):
    n = 8
    m = [[A[i][j] for j in range(n)] for i in range(n)]
    b = [B[i] for i in range(n)]
    for col in range(n):
        max_val, max_row = abs(m[col][col]), col
        for row in range(col + 1, n):
            if abs(m[row][col]) > max_val:
                max_val, max_row = abs(m[row][col]), row
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
    A, B = [], []
    for i in range(4):
        px, py = float(pixels[i][0]), float(pixels[i][1])
        wx, wy = float(world[i][0]), float(world[i][1])
        A.append([px, py, 1, 0, 0, 0, -wx * px, -wx * py])
        A.append([0, 0, 0, px, py, 1, -wy * px, -wy * py])
        B.extend([wx, wy])
    h = mat_solve_8x8(A, B)
    if h is None:
        return None
    return [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], 1.0]
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

def world_to_pixel(X, Y, H):
    w = H[2][0] * X + H[2][1] * Y + H[2][2]
    if abs(w) < 1e-10:
        return (-1, -1)
    u = (H[0][0] * X + H[0][1] * Y + H[0][2]) / w
    v = (H[1][0] * X + H[1][1] * Y + H[1][2]) / w
    return (int(u), int(v))

# ======================================================================
# IPM calibration data; current Plus camera setup is calibrated at 22 cm above ground.
# Re-run calibration mode if the camera height or pitch changes.
# ======================================================================
CALIB_PIXEL = [
    [90, 240],     # Point 0: near left
    [236, 240],    # Point 1: near right
    [121, 149],    # Point 2: far left
    [210, 149],    # Point 3: far right
]

# Ground-clearance 22 cm calibration parameters
CALIB_WORLD = [
    [-8, 6],   # Point 0: left 7.5 cm, forward 7.5 cm
    [7, 6],    # Point 1: right 7.5 cm, forward 7.5 cm
    [-8, 21],  # Point 2: left 7.5 cm, forward 22.5 cm
    [8, 21],   # Point 3: right 7.5 cm, forward 22.5 cm
]

# Compute homography matrix
H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)
H_world2pix = calc_homography(CALIB_WORLD, CALIB_PIXEL)
crossline_ipm = create_crossline_ipm(uart_enabled=False)
crossline_ipm.set_debug_draw(False)
crossline_ipm.H_pix2world = H_pix2world
crossline_ipm.yellow_threshold = yellow_threshold
load_white_bear_model()

# ======================================================================
# Bird-view configuration for debug mode
# ======================================================================
BIRD_W = 80
BIRD_H = 80
X_MIN = -15.0    # Left 15 cm
X_MAX = 15.0     # Right 15 cm
Y_MIN = 5.0      # Near 5 cm
Y_MAX = 35.0     # Far 35 cm
SX = (X_MAX - X_MIN) / BIRD_W
SY = (Y_MAX - Y_MIN) / BIRD_H

# ======================================================================
# Distance estimation parameters based on target width
# ======================================================================
# Distance formula: distance = (real_width * focal_length) / pixel_width

# Real target widths in mm
TARGET_REAL_WIDTH = [
    70.0,   # Color 1: bag, 7 cm
    70.0,   # Color 2: bag, 7 cm
    67.0,   # Color 3: tennis ball, about 6.7 cm diameter
    120.0,  # Color 4: brown teddy bear, about 12 cm body width
    120.0   # Color 5: white teddy bear, about 12 cm body width
]

# Camera calibration parameter from field testing
FOCAL_LENGTH = 167.5  # Focal length parameter; tune after calibration

# Valid distance range in mm
MIN_DETECT_DISTANCE = 50    # Minimum detection distance
MAX_DETECT_DISTANCE = 2000  # Maximum detection distance

def calculate_distance(pixel_width, color_id=1):
    """根据目标像素宽度和颜色ID计算距离"""
    if pixel_width <= 0:
        return -1
    real_width = TARGET_REAL_WIDTH[color_id - 1] if 1 <= color_id <= len(TARGET_REAL_WIDTH) else 70.0
    distance = (real_width * FOCAL_LENGTH) / pixel_width
    if distance < MIN_DETECT_DISTANCE or distance > MAX_DETECT_DISTANCE:
        return -1
    return int(distance)

def detect_obstacle(img):
    blobs = img.find_blobs(obstacle_threshold, roi=OBSTACLE_ROI,
                           pixels_threshold=OBSTACLE_MIN_PIXELS,
                           area_threshold=OBSTACLE_MIN_AREA,
                           merge=True)
    if not blobs:
        return (OBSTACLE_NONE, None)

    in_path = []
    for blob in blobs:
        left = blob.x()
        right = blob.x() + blob.w()
        img.draw_rectangle(blob.rect(), color=(255, 128, 0), thickness=2)
        if not (right < OBSTACLE_PATH_X_MIN or left > OBSTACLE_PATH_X_MAX):
            in_path.append(blob)

    if not in_path:
        return (OBSTACLE_NONE, blobs)

    if len(in_path) > 1:
        return (OBSTACLE_BLOCKED, blobs)

    blob = in_path[0]
    left = blob.x()
    right = blob.x() + blob.w()
    overlap_left = max(left, OBSTACLE_PATH_X_MIN)
    overlap_right = min(right, OBSTACLE_PATH_X_MAX)
    overlap_center = (overlap_left + overlap_right) // 2

    if overlap_center < 160:
        return (OBSTACLE_MOVE_RIGHT, blobs)
    return (OBSTACLE_MOVE_LEFT, blobs)

def box_hits_obstacle(box, obstacle_blobs):
    if not obstacle_blobs:
        return False
    x, y, w, h = box
    x2 = x + w
    y2 = y + h
    for blob in obstacle_blobs:
        bx = blob.x()
        by = blob.y()
        bx2 = bx + blob.w()
        by2 = by + blob.h()
        inter_w = min(x2, bx2) - max(x, bx)
        inter_h = min(y2, by2) - max(y, by)
        if inter_w > 0 and inter_h > 0 and inter_w * inter_h >= OBSTACLE_TARGET_OVERLAP_PIXELS:
            return True
    return False

# ======================================================================
# UART protocol
# ======================================================================
# Packet format, 14 bytes:
# [0-1]   Header: 0xAA 0x55
# [2]     Color ID, 0=no target, 1=light blue, 2=red, 3+=reserved
# [3-4]   Center X, low byte then high byte
# [5-6]   Center Y, low byte then high byte
# [7-8]   Width, low byte then high byte
# [9-10]  Height, low byte then high byte
# [11-12] Distance in mm, low byte then high byte
# [13]    Checksum, sum of data bytes masked to 0xFF

def calculate_checksum(data):
    """计算校验和"""
    return sum(data) & 0xFF

def send_target_data(color_id, cx, cy, w, h, distance):
    """
    发送沙包坐标数据到RT1021主控

    Args:
        color_id: 颜色ID (1=颜色1, 2=颜色2, ...)
        cx: 中心X坐标 (0-320)
        cy: 中心Y坐标 (0-240)
        w: 宽度
        h: 高度
        distance: 距离 (mm)
    """
    data = bytearray(14)

    # Header
    data[0] = 0xAA
    data[1] = 0x55

    # Color ID
    data[2] = color_id

    # Coordinates and size, little-endian
    data[3] = cx & 0xFF          # X low byte
    data[4] = (cx >> 8) & 0xFF   # X high byte
    data[5] = cy & 0xFF          # Y low byte
    data[6] = (cy >> 8) & 0xFF   # Y high byte
    data[7] = w & 0xFF           # Width low byte
    data[8] = (w >> 8) & 0xFF    # Width high byte
    data[9] = h & 0xFF           # Height low byte
    data[10] = (h >> 8) & 0xFF   # Height high byte

    # Distance, little-endian
    if distance < 0:
        distance = 0
    data[11] = distance & 0xFF          # Distance low byte
    data[12] = (distance >> 8) & 0xFF   # Distance high byte

    # Checksum over data bytes 2 through 12
    data[13] = calculate_checksum(data[2:13])

    # Send packet
    uart.write(data)

def send_no_target():
    """发送无目标数据"""
    data = bytearray(14)
    data[0] = 0xAA
    data[1] = 0x55
    # data[2-12] remain zero, so color ID 0 means no target
    data[13] = 0  # Checksum is also zero
    uart.write(data)

# ======================================================================
# UART protocol, world-coordinate packet
# ======================================================================
# [0-1]  Header: 0xAA 0x55
# [2]    Color ID, 0=no target, 1=light blue, 2=red, 3=ball, 4=brown bear, 5=white bear
# [3-4]  World X, int16 little-endian, mm*10, left positive
# [5-6]  World Y, int16 little-endian, mm*10, forward positive
# [7-8]  Pixel width, uint16 little-endian
# [9]    Checksum, sum(data[2:9]) & 0xFF

def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                    angle_flag=0x00, angle_cdeg=0):
    # World packet v2, 16 bytes:
    # [12] angle_flag: bit0=angle enabled, bit1=angle valid
    # [13-14] crossline angle, int16 little-endian, degree * 100
    # [15] checksum = sum(data[2:15]) & 0xFF
    """发送世界坐标数据包 (12字节, 含黄线信息)
    [0-1]  帧头 0xAA 0x55
    [2]    颜色ID
    [3-4]  世界X (mm, int16, 小端序)
    [5-6]  世界Y (mm, int16, 小端序)
    [7-8]  像素宽度 (uint16)
    [9]    黄线标志 0x00/0x01
    [10]   位置关系 0x00/0x01/0x02
    [11]   校验和 (data[2:11])
    """
    data = bytearray(16)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = color_id & 0xFF
    data[3] = wx_mm & 0xFF
    data[4] = (wx_mm >> 8) & 0xFF
    data[5] = wy_mm & 0xFF
    data[6] = (wy_mm >> 8) & 0xFF
    data[7] = pw & 0xFF
    data[8] = (pw >> 8) & 0xFF
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[11] = obstacle_flag & 0xFF
    data[12] = angle_flag & 0xFF
    data[13] = angle_cdeg & 0xFF
    data[14] = (angle_cdeg >> 8) & 0xFF
    data[15] = sum(data[2:15]) & 0xFF
    uart.write(data)

def send_world_no_target(yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                         angle_flag=0x00, angle_cdeg=0):
    # Same 16-byte packet layout as send_world_data(), with color_id/position fields zero.
    """发送无目标数据包 (12字节)"""
    data = bytearray(16)
    data[0] = 0xAA
    data[1] = 0x55
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[11] = obstacle_flag & 0xFF
    data[12] = angle_flag & 0xFF
    data[13] = angle_cdeg & 0xFF
    data[14] = (angle_cdeg >> 8) & 0xFF
    data[15] = sum(data[2:15]) & 0xFF
    uart.write(data)

def process_return_beacon_frame(img):
    global beacon_last_box, beacon_lost_frames, last_print_time

    obstacle_flag, obstacle_blobs = detect_obstacle(img)
    angle_flag, angle_cdeg = get_crossline_angle_fields()

    if H_pix2world is None:
        send_world_no_target(False, POS_NO_BOUNDARY, obstacle_flag, angle_flag, angle_cdeg)
        return

    blob = find_beacon_blob(img, beacon_last_box)
    if blob is not None:
        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()

        beacon_last_box = (x, y, w, h)
        beacon_lost_frames = 0

        world_x, world_y = box_to_world(x, y, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(RETURN_BEACON_ID, wx_mm, wy_mm, w, False, POS_NO_BOUNDARY,
                        obstacle_flag, angle_flag, angle_cdeg)

        img.draw_rectangle(blob.rect(), color=(0, 255, 255), thickness=2)
        img.draw_cross(cx, cy, color=(255, 0, 0), size=8, thickness=2)
        img.draw_string(x, max(0, y - 16),
                        "return ({:.1f},{:.1f})cm".format(world_x, world_y),
                        color=(0, 255, 255), scale=1)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 300:
            last_print_time = now
    else:
        beacon_lost_frames += 1
        if beacon_lost_frames > BEACON_TRACK_MAX_LOST:
            beacon_last_box = None
        send_world_no_target(False, POS_NO_BOUNDARY, obstacle_flag, angle_flag, angle_cdeg)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 500:
            last_print_time = now

def receive_command_from_host():
    """接收RT1021主机命令"""
    global active_threshold, active_color_id, red_thresholds
    global lost_frame_count, stable_detect_count, openart_mode
    global local_track_rect, last_tracked_pixels, track_force_global_next, track_local_miss_count
    global target_color_id
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    global _cmd_rx_buf, crossline_angle_enabled, crossline_angle_result
    global yellow_seen_in_carry, yellow_tracking, yellow_detected, yellow_recent_count

    if uart.any():
        chunk = uart.read(uart.any())
        if chunk:
            _cmd_rx_buf.extend(chunk)
    if len(_cmd_rx_buf) > 64:
        _cmd_rx_buf = _cmd_rx_buf[-32:]

    while len(_cmd_rx_buf) >= 4:
        idx = -1
        for i in range(len(_cmd_rx_buf) - 1):
            if _cmd_rx_buf[i] == 0xAA and _cmd_rx_buf[i + 1] == 0x55:
                idx = i
                break
        if idx < 0:
            _cmd_rx_buf = bytearray()
            return (0, 0)
        if idx > 0:
            _cmd_rx_buf = _cmd_rx_buf[idx:]
        if len(_cmd_rx_buf) < 4:
            return (0, 0)

        command = _cmd_rx_buf[2]
        if command == 0x03 or command == 0x04:
            if len(_cmd_rx_buf) < 5:
                return (0, 0)
            param = _cmd_rx_buf[3]
            checksum_recv = _cmd_rx_buf[4]
            checksum_calc = (command + param) & 0xFF
            frame_len = 5
        else:
            param = 0
            checksum_recv = _cmd_rx_buf[3]
            checksum_calc = command & 0xFF
            frame_len = 4

        if checksum_calc != checksum_recv:
            _cmd_rx_buf = _cmd_rx_buf[2:]
            continue

        _cmd_rx_buf = _cmd_rx_buf[frame_len:]

        if command == 0x03:  # SET_TARGET_COLOR
            if 1 <= param <= len(all_color_thresholds):
                target_color_id = param
                active_color_id = param
                if USE_WHITE_BEAR_MODEL and param == WHITE_BEAR_COLOR_ID:
                    active_threshold = None
                    red_thresholds = all_color_thresholds
                else:
                    active_threshold = [all_color_thresholds[param - 1]]
                    red_thresholds = active_threshold
                lost_frame_count = 0
                stable_detect_count = 0
                color_track_active = False
                color_track_box = None
                color_track_color_id = 0
                color_lost_count = 0
                local_track_rect = None
                last_tracked_pixels = -1
                track_force_global_next = False
                track_local_miss_count = 0
        elif command == 0x01:  # Enter carry mode
            openart_mode = MODE_CARRY
            reset_yellow_state()
            reset_beacon_state()
        elif command == 0x04:  # SET_CROSSLINE_ANGLE_ENABLE
            crossline_angle_enabled = (param == 1)
            if not crossline_angle_enabled:
                crossline_angle_result = None
        elif command == 0x05:  # ENTER_RETURN_MODE
            openart_mode = MODE_RETURN
            reset_target_tracking_state()
            reset_yellow_state()
            reset_beacon_state()
            crossline_angle_enabled = False
            crossline_angle_result = None
        elif command == 0x00 or command == 0x02:  # Reset to search mode or turn completed
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
            reset_beacon_state()
            crossline_angle_enabled = False
            crossline_angle_result = None
        return (command, param)
    return (0, 0)

def current_pos_flag(frame_count):
    global yellow_lost_count, yellow_seen_in_carry, openart_mode
    if openart_mode == MODE_CARRY:
        if yellow_detected:
            yellow_seen_in_carry = True
            yellow_lost_count = 0
            return POS_NO_BOUNDARY
        if yellow_seen_in_carry and (frame_count % YELLOW_DETECT_INTERVAL == 0):
            yellow_lost_count += 1
            if yellow_lost_count >= YELLOW_LOST_THRESHOLD:
                openart_mode = MODE_WAIT_TURN
                return POS_CROSSED
        return POS_NO_BOUNDARY
    if openart_mode == MODE_WAIT_TURN:
        return POS_CROSSED
    if openart_mode == MODE_SEARCH:
        yellow_lost_count = 0
        if yellow_detected:
            return POS_RIGHT_SIDE
    return POS_NO_BOUNDARY

def find_yellow_blob_bottom_up(img, roi, pixels_threshold):
    x, y, w, h = roi
    y_end = y + h
    strip_h = YELLOW_SCAN_STRIP_H
    scan_y = y_end - strip_h
    while scan_y >= y:
        current_h = min(strip_h, y_end - scan_y)
        strip_roi = (x, scan_y, w, current_h)
        blobs = img.find_blobs(yellow_threshold, roi=strip_roi,
                               pixels_threshold=pixels_threshold,
                               area_threshold=20, merge=True)
        if blobs:
            return max(blobs, key=lambda b: b.pixels())
        scan_y -= strip_h
    if scan_y + strip_h > y:
        current_h = scan_y + strip_h - y
        strip_roi = (x, y, w, current_h)
        blobs = img.find_blobs(yellow_threshold, roi=strip_roi,
                               pixels_threshold=pixels_threshold,
                               area_threshold=20, merge=True)
        if blobs:
            return max(blobs, key=lambda b: b.pixels())
    return None

def update_yellow_detection(img, frame_count):
    global yellow_tracking, yellow_detected
    global yellow_recent_count
    global yellow_boundary_y, yellow_boundary_left_y, yellow_boundary_right_y, yellow_boundary_wy

    if frame_count % YELLOW_DETECT_INTERVAL != 0:
        return

    yellow_pixels_threshold = YELLOW_KEEP_PIXELS if yellow_tracking else YELLOW_ENTER_PIXELS

    if openart_mode == MODE_CARRY:
        left_blob = find_yellow_blob_bottom_up(img, YELLOW_ROI_LEFT, yellow_pixels_threshold)
        right_blob = find_yellow_blob_bottom_up(img, YELLOW_ROI_RIGHT, yellow_pixels_threshold)
        yellow_blobs_left = [left_blob] if left_blob else None
        yellow_blobs_right = [right_blob] if right_blob else None
    else:
        yellow_blobs_left = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_LEFT,
                                           pixels_threshold=yellow_pixels_threshold,
                                           area_threshold=20, merge=True)
        yellow_blobs_right = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_RIGHT,
                                            pixels_threshold=yellow_pixels_threshold,
                                            area_threshold=20, merge=True)

    raw_yellow_seen = (yellow_blobs_left and yellow_blobs_right)

    # Use a higher threshold for initial detection, then a lower hold threshold.
    if raw_yellow_seen:
        yellow_tracking = True
        yellow_detected = True
        yellow_recent_count = YELLOW_RECENT_DETECTIONS
    else:
        yellow_detected = False
        if yellow_recent_count > 0:
            yellow_recent_count -= 1
        if openart_mode == MODE_SEARCH:
            yellow_tracking = False

    if yellow_detected:
        if openart_mode != MODE_CARRY:
            left_blob = max(yellow_blobs_left, key=lambda b: b.pixels())
            right_blob = max(yellow_blobs_right, key=lambda b: b.pixels())
        yellow_boundary_left_y = left_blob.cy()
        yellow_boundary_right_y = right_blob.cy()
        yellow_boundary_y = (yellow_boundary_left_y + yellow_boundary_right_y) // 2

        if H_pix2world:
            _, yellow_boundary_wy = pixel_to_world(160, yellow_boundary_y, H_pix2world)
    elif openart_mode == MODE_SEARCH:
        yellow_boundary_y = 0
        yellow_boundary_left_y = 0
        yellow_boundary_right_y = 0
        yellow_boundary_wy = 0.0

def get_crossline_angle_fields():
    if not crossline_angle_enabled or crossline_angle_result is None:
        return (0x00, 0)

    flag = 0x01
    if crossline_angle_result["valid"]:
        flag |= 0x02
    return (flag, crossline_angle_result["angle_cdeg"])

# ======================================================================
# Main loop
# ======================================================================
frame_count = 0
detect_count = 0
last_print_time = time.ticks_ms()

# ======================================================================
# Calibration mode, enabled when CALIBRATION_MODE is True
# ======================================================================
if CALIBRATION_MODE:
    _calib_pts = []
    _calib_stable = 0
    _calib_last = [0, 0]
    _calib_wait_remove = False
    _STABLE_NEED = 20
    _MOVE_THR = 10
    _calib_t = time.ticks_ms()


    # Use only the first two reliable bag thresholds for calibration.
    _calib_thresholds = all_color_thresholds[:2]

    while len(_calib_pts) < 4:
        clock.tick()
        img = snapshot_frame()

        for _gx in range(0, 321, 40):
            img.draw_line(_gx, 0, _gx, 240, color=(64, 64, 64))
        for _gy in range(0, 241, 40):
            img.draw_line(0, _gy, 320, _gy, color=(64, 64, 64))
        img.draw_line(160, 0, 160, 240, color=(0, 128, 0))
        img.draw_line(0, 120, 320, 120, color=(0, 128, 0))

        for _ci in range(len(_calib_pts)):
            _cp = _calib_pts[_ci]
            img.draw_circle(_cp[0], _cp[1], 6, color=(0, 255, 0), thickness=2)
            img.draw_string(_cp[0]+8, _cp[1]-4, str(_ci), color=(0,255,0), scale=2)

        img.draw_string(2, 2, "Calib {}/4".format(len(_calib_pts)), color=(255,255,0), scale=2)

        _blobs = img.find_blobs(_calib_thresholds,
                                roi=DETECT_ROI,
                                pixels_threshold=50, area_threshold=50, merge=True)

        if _blobs:
            _bl = max(_blobs, key=lambda b: b.pixels())
            _bx, _by = _bl.cx(), _bl.cy()

            if _calib_wait_remove:
                img.draw_string(2, 220, "Remove marker...", color=(255,128,0), scale=2)
            else:
                img.draw_rectangle(_bl.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(_bx, _by, color=(255, 0, 0), size=10, thickness=2)
                img.draw_string(_bx+12, _by-8, "({},{})".format(_bx, _by),
                                color=(255,255,0), scale=2)

                _dx = abs(_bx - _calib_last[0])
                _dy = abs(_by - _calib_last[1])
                if _dx < _MOVE_THR and _dy < _MOVE_THR:
                    _calib_stable += 1
                    _bar = min(_calib_stable * 100 // _STABLE_NEED, 100)
                    img.draw_rectangle((10, 225, _bar, 10), color=(0,255,0), fill=True)
                    img.draw_rectangle((10, 225, 100, 10), color=(255,255,255))

                    if _calib_stable >= _STABLE_NEED:
                        _calib_pts.append([_bx, _by])
                        _n = len(_calib_pts) - 1
                        _calib_stable = 0
                        _calib_wait_remove = True
                else:
                    _calib_stable = 0
                _calib_last = [_bx, _by]
        else:
            _calib_stable = 0
            _calib_wait_remove = False

    for _ci in range(4):
        _cp = _calib_pts[_ci]

    _H_test = calc_homography(_calib_pts, CALIB_WORLD)
    while True:
        clock.tick()
        img = snapshot_frame()
        for _ci in range(4):
            _cp = _calib_pts[_ci]
            img.draw_circle(_cp[0], _cp[1], 6, color=(0, 255, 0), thickness=2)
            img.draw_string(_cp[0]+8, _cp[1]-4, str(_ci), color=(0,255,0), scale=2)
        img.draw_string(2, 2, "Verify", color=(0,255,0), scale=2)

        _blobs = img.find_blobs(_calib_thresholds,
                                roi=DETECT_ROI,
                                pixels_threshold=50, area_threshold=50, merge=True)
        if _blobs and _H_test:
            _bl = max(_blobs, key=lambda b: b.pixels())
            _bx, _by = _bl.cx(), _bl.cy()
            _wx, _wy = pixel_to_world(_bx, _by, _H_test)
            img.draw_cross(_bx, _by, color=(255,0,0), size=10, thickness=2)
            img.draw_string(_bx+12, _by-8, "({},{})".format(_bx, _by),
                            color=(255,255,0), scale=1)
            img.draw_string(_bx+12, _by+8, "w({:.1f},{:.1f})cm".format(_wx, _wy),
                            color=(0,255,255), scale=1)
            _now = time.ticks_ms()
            if time.ticks_diff(_now, _calib_t) >= 500:
                _calib_t = _now

# ======================================================================
# Bird-view frame buffer for debug mode
# ======================================================================
if BIRDVIEW_DEBUG and not CALIBRATION_MODE:
    bird = sensor.alloc_extra_fb(BIRD_W, BIRD_H, sensor.RGB565)

while True:
    # FPS accounting
    clock.tick()
    frame_count += 1
    log_force = frame_count <= LOG_FIRST_FRAMES
    log_checkpoint('loop_start', frame_count, force=log_force)

    # Receive host command
    log_checkpoint('before_uart_read', frame_count, force=log_force)
    cmd, param = receive_command_from_host()
    log_checkpoint('after_uart_read', frame_count, force=log_force)

    # Capture image and apply lens correction
    log_checkpoint('before_snapshot', frame_count, force=log_force)
    img = snapshot_frame(apply_lens_corr=RUNTIME_LENS_CORR)
    log_checkpoint('after_snapshot', frame_count, force=log_force)
    world_x = 0.0
    world_y = 0.0

    if openart_mode == MODE_RETURN:
        log_checkpoint('before_return_frame', frame_count, force=log_force)
        process_return_beacon_frame(img)
        log_checkpoint('after_return_frame', frame_count, force=log_force)
        feed_watchdog()
        continue

    if crossline_angle_enabled:
        log_checkpoint('before_crossline', frame_count, force=log_force)
        crossline_angle_result = crossline_ipm.process_frame(img)
        log_checkpoint('after_crossline', frame_count, force=log_force)

    # ===== Dynamic cut update =====
    log_checkpoint('before_dynamic_cut', frame_count, force=log_force)
    update_dynamic_cut(img, frame_count)
    log_checkpoint('after_dynamic_cut', frame_count, force=log_force)
    log_checkpoint('before_obstacle', frame_count, force=log_force)
    obstacle_flag, obstacle_blobs = detect_obstacle(img)
    log_checkpoint('after_obstacle', frame_count, force=log_force)
    log_checkpoint('before_yellow', frame_count, force=log_force)
    update_yellow_detection(img, frame_count)
    log_checkpoint('after_yellow', frame_count, force=log_force)

    # ===== Color blob detection / tracking =====
    best = None
    source = 'color'
    send_color_id = 0
    last_box = color_track_box if color_track_active else None

    model_found = None
    if USE_WHITE_BEAR_MODEL and target_color_id == WHITE_BEAR_COLOR_ID:
        log_checkpoint('before_model_target', frame_count, force=log_force)
        model_found = find_white_bear_model_target(img, last_box)
        log_checkpoint('after_model_target', frame_count, force=log_force)
        if model_found:
            send_color_id, x1, y1, w, h, model_score = model_found
            if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
                best = (send_color_id, x1, y1, w, h)
                color_track_active = True
                color_track_box = (x1, y1, w, h)
                color_track_color_id = send_color_id
                color_lost_count = 0
                source = 'model'
                found = True
            else:
                found = None
        else:
            found = None
    else:
        log_checkpoint('before_color_target', frame_count, force=log_force)
        found = find_color_target(img, last_box)
        log_checkpoint('after_color_target', frame_count, force=log_force)

    if found and (not USE_WHITE_BEAR_MODEL or target_color_id != WHITE_BEAR_COLOR_ID):
        send_color_id, blob = found
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
        if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
            best = (send_color_id, x1, y1, w, h)
            color_track_active = True
            color_track_box = (x1, y1, w, h)
            color_track_color_id = send_color_id
            color_lost_count = 0
        else:
            found = None

    if USE_WHITE_BEAR_MODEL and not found and target_color_id == 0:
        log_checkpoint('before_model_search', frame_count, force=log_force)
        model_found = find_white_bear_model_target(img, last_box)
        log_checkpoint('after_model_search', frame_count, force=log_force)
        if model_found:
            send_color_id, x1, y1, w, h, model_score = model_found
            if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
                best = (send_color_id, x1, y1, w, h)
                color_track_active = True
                color_track_box = (x1, y1, w, h)
                color_track_color_id = send_color_id
                color_lost_count = 0
                source = 'model'
                found = True

    if not found and color_track_active and color_track_box:
        color_lost_count += 1
        if color_lost_count <= COLOR_LOST_FRAMES:
            x1, y1, w, h = color_track_box
            send_color_id = color_track_color_id
            best = (send_color_id, x1, y1, w, h)
            source = 'color_hold'
        else:
            color_track_active = False
            color_track_box = None
            color_track_color_id = 0
            color_lost_count = 0

    pos_flag = current_pos_flag(frame_count)
    angle_flag, angle_cdeg = get_crossline_angle_fields()

    if best:
        lost_frame_count = 0
        stable_detect_count += 1
        send_color_id, x1, y1, w, h = best

        if target_color_id == 0 and send_color_id > 0 and stable_detect_count >= STABLE_FRAMES_REQUIRED and not SLAVE_MODE:
            target_color_id = send_color_id
            active_color_id = send_color_id
            active_threshold = [all_color_thresholds[send_color_id - 1]]
            red_thresholds = active_threshold

        cx = x1 + w // 2
        cy = y1 + h // 2
        x2 = x1 + w
        y2 = y1 + h

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        distance = calculate_distance(w, send_color_id)
        log_checkpoint('before_uart_write_target', frame_count, force=log_force)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag, obstacle_flag,
                        angle_flag, angle_cdeg)
        log_checkpoint('after_uart_write_target', frame_count, force=log_force)

        color = (255, 0, 0)
        if send_color_id == 1:
            color = (0, 170, 255)
        elif send_color_id == 2:
            color = (255, 0, 0)
        elif send_color_id == 3:
            color = (0, 255, 0)
        elif send_color_id == 4:
            color = (160, 96, 32)
        elif send_color_id == 5:
            color = (255, 255, 255)
        text = 'cid={} {}'.format(send_color_id, source)
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_cross(cx, cy, color=color, size=8, thickness=2)
        img.draw_string(x1, max(0, y1 - 15), text, color=color, scale=1)
        detect_count += 1

    else:
        lost_frame_count += 1
        stable_detect_count = 0
        if lost_frame_count > MAX_LOST_FRAMES and (target_color_id > 0 or active_threshold is not None or color_track_active):
            reset_target_tracking_state()
        log_checkpoint('before_uart_write_none', frame_count, force=log_force)
        send_world_no_target(yellow_detected, pos_flag, obstacle_flag, angle_flag, angle_cdeg)
        log_checkpoint('after_uart_write_none', frame_count, force=log_force)
    # Yellow detection is updated before pos_flag is calculated.

    # Draw dynamic cut line (debug)
    if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
        img.draw_line(CUT_LEFT_X, dynamic_cut_left_y, CUT_RIGHT_X, dynamic_cut_right_y,
                      color=(0, 180, 255), thickness=2)

    # Draw yellow boundary line
    if yellow_boundary_y > 0:
        img.draw_line(0, yellow_boundary_y, 320, yellow_boundary_y,
                     color=(255, 255, 0), thickness=2)

    # Render bird-view debug image
    if BIRDVIEW_DEBUG:
        bird.clear()
        for _by in range(BIRD_H):
            Y = Y_MIN + _by * SY
            for _bx in range(BIRD_W):
                X = X_MIN + _bx * SX
                u, v = world_to_pixel(X, Y, H_world2pix)
                if 0 <= u < 320 and 0 <= v < 240:
                    bird.set_pixel(_bx, _by, img.get_pixel(u, v))

        if world_x != 0.0 or world_y != 0.0:
            bx_mark = int((world_x - X_MIN) / SX)
            by_mark = int((world_y - Y_MIN) / SY)
            if 0 <= bx_mark < BIRD_W and 0 <= by_mark < BIRD_H:
                bird.draw_cross(bx_mark, by_mark, color=(255, 0, 0), size=5, thickness=2)

        img.clear()
        img.draw_image(bird, 0, 0)
        img.draw_string(85, 5, "w=({:.1f},{:.1f})".format(world_x, world_y),
                        color=(0,255,255), scale=1)
        img.draw_string(85, 20, "FPS:{:.1f}".format(clock.fps()),
                        color=(255,255,0), scale=1)

    log_checkpoint('loop_end', frame_count, force=log_force)
    feed_watchdog()
