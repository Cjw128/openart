# ======================================================================
# OpenART Plus fixed master-camera runtime - multi-target object detection
# ======================================================================


import sensor, gc
from machine import UART
try:
    from machine import WDT
except Exception:
    WDT = None

# ======================================================================
# Mode selection
# ======================================================================
ENABLE_WATCHDOG = True
WATCHDOG_TIMEOUT_MS = 8000

wdt = None

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

def snapshot_frame():
    return sensor.snapshot().replace(hmirror=True)

# White balance configuration
WB_GAINS = (101.00, 64.00, 97.00) # Fixed white balance
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.skip_frames(time=500)

# Fixed exposure
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)

# Both master and slave cameras are OpenART Plus boards and use UART12.
uart = UART(12, baudrate=115200)

# ======================================================================
# LAB color thresholds - dynamic multi-color detection
# ======================================================================
# LAB format: (L_min, L_max, A_min, A_max, B_min, B_max)
# L: brightness (0-100)
# A: red-green axis (positive=red, negative=green)
# B: yellow-blue axis (positive=yellow, negative=blue)

# Supported color thresholds (fallback defaults; overridden by /sd/color_thr.txt if present)
all_color_thresholds = [
    (34, 100, -41, 4, -72, -22),    # Color 1: light-blue bag
    (10, 80, 22, 122, -17, 93),     # Color 2: red bag
    (50, 100, -128, -27, 20, 127),  # Color 3: tennis ball
    (21, 52, -77, 25, 6, 99),        # Color 4: brown teddy bear; tune on field
    (51, 100, -5, 5, -38, 18)      # Color 5: white teddy bear
]
WHT_BEAR_GND_L_GAP = 2

def _average_ground_threshold(ground_rows):
    ground = ground_rows.get('ground')
    ground2 = ground_rows.get('ground2')
    if ground and ground2:
        averaged = []
        for i in range(6):
            averaged.append((ground[i] + ground2[i]) // 2)
        return tuple(averaged)
    return ground if ground else ground2

def _separate_white_bear_from_ground(threshold, ground):
    if not threshold or not ground:
        return threshold
    a_overlap = not (threshold[3] < ground[2] or ground[3] < threshold[2])
    b_overlap = not (threshold[5] < ground[4] or ground[5] < threshold[4])
    if not (a_overlap and b_overlap):
        return threshold
    if (threshold[0] + threshold[1]) <= (ground[0] + ground[1]):
        return threshold
    l0 = max(threshold[0], min(ground[1] + WHT_BEAR_GND_L_GAP,
                               threshold[1]))
    return (l0, threshold[1], threshold[2], threshold[3],
            threshold[4], threshold[5])

def _parse_int_values(parts):
    values = []
    for part in parts:
        values.append(int(part))
    return tuple(values)

def _load_calibrated_params(path='/sd/color_thr.txt'):
    try:
        rows = {}
        ground_rows = {}
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
                    try:
                        name, raw_values = line.split('=', 1)
                        values = _parse_int_values(raw_values.split(','))
                        if len(values) == 6:
                            ground_rows[name] = values
                    except Exception:
                        pass
                    continue
                parts = line.split(',')
                if len(parts) == 7:
                    slot = int(parts[0])
                    values = _parse_int_values(parts[1:])
                elif len(parts) == 6:
                    slot = len(rows) + 1
                    values = _parse_int_values(parts)
                else:
                    continue
                if 1 <= slot <= 5 and len(values) == 6:
                    rows[slot] = values
        if len(rows) == 5:
            loaded_rows = []
            for slot in range(1, 6):
                loaded_rows.append(rows[slot])
            return (loaded_rows, exposure, _average_ground_threshold(ground_rows))
        return None, None, None
    except Exception:
        pass
    return None, None, None

_loaded, _loaded_exposure, _loaded_ground_threshold = _load_calibrated_params()
if _loaded:
    all_color_thresholds = _loaded
    all_color_thresholds[4] = _separate_white_bear_from_ground(
        all_color_thresholds[4], _loaded_ground_threshold)
    if _loaded_exposure is not None:
        sensor.set_auto_exposure(False, exposure_us=_loaded_exposure)

_color_threshold_groups = []
for threshold in all_color_thresholds:
    _color_threshold_groups.append([threshold])

COLOR_LOST_FRAMES = 5
COLOR_TRACK_MARGIN = 45
COLOR_MIN_PIXELS = 70
COLOR_MIN_AREA = 100
TENNIS_MIN_PIXELS = 80
TENNIS_MIN_AREA = 80
NEAR_NOISE_Y_MIN = 170
NEAR_NOISE_BOX_AREA = 400
_color_blob_limits = (
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
)
MULTICOLOR_MIN_PIXELS = min(COLOR_MIN_PIXELS, TENNIS_MIN_PIXELS)
MULTICOLOR_MIN_AREA = min(COLOR_MIN_AREA, TENNIS_MIN_AREA)
TARGET_BOX_COLORS = (
    (0, 170, 255), (255, 0, 0), (0, 255, 0),
    (160, 96, 32), (255, 255, 255),
)

# Blue floor threshold, example values that must be tuned on field.
# Focus on the B channel; blue is usually below -20.
# BLUE_GROUND_THRESHOLD = (10, 50, -20, 50, -77, -25)

# Target-lost counters
lost_frame_count = 0                    # Consecutive frames without target
MAX_LOST_FRAMES = 30                    # Maximum lost frames, about 0.5s at 60 FPS

# ======================================================================
# Recognition parameters
# ======================================================================
DETECT_Y_MIN = 8           # Ignore image rows above this Y value
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)
COLOR_DETECT_Y_MAX = 230   # Color targets ignore image rows y=230..239.

# ======================================================================
# Dynamic cut line (kept in sync with the competition calibration preview)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
# 动态裁切使用 ground/ground2 六个 LAB 边界的逐项平均；缺失时使用单组或兜底值。
BLUE_GROUND_THRESHOLD = ([_loaded_ground_threshold]
                         if _loaded_ground_threshold
                         else [(25, 62, -3, 57, -96, 127)])
CUT_BLOB_MIN_H = 12
CUT_BLOB_BOTTOM_MARGIN = 25
CUT_GAP_BRIDGE = 10
# 多条竖直采样带横跨画面，以横向分布和稳健分位数生成水平裁切线。
CUT_STRIP_XS = (10, 85, 160, 235, 310)
CUT_MIN_VALID_STRIPS = 3
CUT_SINGLE_STRIP_MAX_LEAD = 40
CUT_MIN_VALID_X_SPAN = 180
CUT_BAFFLE_SPREAD_PX = 30
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
CUT_ROI_Y_OFFSET = -10      # Target ROI starts 10 px above the detected blue-ground boundary.
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

# Target continuity filters
TRACK_MAX_JUMP_PX = 90
TRACK_MAX_JUMP2 = TRACK_MAX_JUMP_PX * TRACK_MAX_JUMP_PX
TRACK_AREA_CHANGE_MAX_PERCENT = 60
TRACK_MIN_IOU = 0.05
BRN_BEAR_MERGE_MARGIN = 12
BRN_BEAR_FRAGMENT_MIN_PIXELS = 20
BRN_BEAR_FRAGMENT_MIN_AREA = 20
BEAR_ACQUIRE_MIN_PIXELS = 120
BEAR_ACQUIRE_MIN_AREA = 300
BEAR_ACQUIRE_CONFIRM_FRAMES = 3
BEAR_ACQUIRE_MAX_JUMP_PX = 45
BEAR_ACQUIRE_MAX_JUMP2 = BEAR_ACQUIRE_MAX_JUMP_PX * BEAR_ACQUIRE_MAX_JUMP_PX
BEAR_ACQUIRE_MAX_AREA_CHANGE_PERCENT = 80
BRN_BEAR_BALL_SHADOW_MAX_AREA_PERCENT = 55
BRN_BEAR_BALL_SHADOW_X_OVERLAP_PERCENT = 60
BRN_BEAR_BALL_SHADOW_Y_MARGIN = 6
WHT_BEAR_MERGE_MARGIN = 10
WHT_BEAR_BOX_EMA_NEW_NUM = 1
WHT_BEAR_BOX_EMA_DEN = 3
WHT_BEAR_SMOOTH_MAX_JUMP_PX = 55
WHT_BEAR_SMOOTH_MAX_JUMP2 = WHT_BEAR_SMOOTH_MAX_JUMP_PX * WHT_BEAR_SMOOTH_MAX_JUMP_PX

dynamic_cut_left_y = DETECT_Y_MIN
dynamic_cut_valid = False
dynamic_cut_miss_count = 0
dynamic_detect_roi = DETECT_ROI
target_color_id = 0
host_color_id_received = False
color_track_active = False
color_track_box = None
color_track_color_id = 0
color_lost_count = 0
bear_acquire_color_id = 0
bear_acquire_box = None
bear_acquire_count = 0
bear_acquire_last_frame = -1
_cmd_rx_buf = bytearray()
front_scan_requested = False
FRONT_SCAN_PACKET_ID = 0xC7
FRONT_SCAN_EXCLUDE_IOU = 0.20
FRONT_SCAN_EXCLUDE_CENTER_PX = 35
FRONT_SCAN_EXCLUDE_CENTER2 = FRONT_SCAN_EXCLUDE_CENTER_PX * FRONT_SCAN_EXCLUDE_CENTER_PX
FRONT_SCAN_Y_MAX = 150
FRONT_SCAN_MIN_PIXELS = 60
FRONT_SCAN_STABLE_FRAMES = 6
FRONT_SCAN_MAX_FRAMES = 12
front_scan_last_current_id = 0
front_scan_last_mask = -1
front_scan_last_count = 0
front_scan_stable_count = 0
front_scan_total_count = 0
# ======================================================================
# Yellow line detection parameters
# ======================================================================
yellow_threshold = [(62, 100, -59, 10, 26, 127)]    # Yellow LAB threshold
ENABLE_YELLOW_DRAW = True
RETURN_YELLOW_PACKET_ID = 0xC8
RETURN_YELLOW_THRESHOLD = yellow_threshold
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
RETURN_STATUS_Y_VALID = 0x01
RETURN_STATUS_STOP = 0x02
return_yellow_last_y = -1
return_yellow_stable_count = 0
return_yellow_detected = False
return_yellow_y = 0
return_stop_x = -1
return_stop_requested = False

YELLOW_ROI_TOP = (0, 90, 320, 20)        # Horizontal strip centered near y=100
YELLOW_ROI_BOTTOM = (0, 130, 320, 20)    # Horizontal strip centered near y=140
YELLOW_DETECT_INTERVAL = 2              # Detect yellow line every N frames
YELLOW_ENTER_PIXELS = 70                # Pixel threshold for first yellow-line hit
YELLOW_KEEP_PIXELS = 20                 # Pixel threshold while tracking a yellow line
YELLOW_CARRY_CONFIRM_FRAMES = 2         # Consecutive hits before carry mode treats yellow as confirmed
YELLOW_MIN_FIT_DX = 30                  # Reject near-vertical fits commonly formed by the carried object
YELLOW_MAX_FIT_SLOPE_X100 = 100         # Yellow boundary must stay within +/-45 degrees
YELLOW_TARGET_OVERLAP_PERCENT = 60      # Reject yellow blobs mostly contained by the tracked target
YELLOW_BOTTOM_Y = 220                   # Arm disappearance detection near the image bottom
YELLOW_LOST_THRESHOLD = 1               # Consecutive misses required after bottom contact

yellow_line_k = 0.0
yellow_line_b = 0.0
yellow_detected = False     # Whether yellow line is visible
yellow_tracking = False      # Hysteresis state after first yellow-line hit
yellow_lost_count = 0       # Consecutive yellow-line misses after bottom contact
yellow_seen_in_carry = False # Whether yellow line was confirmed in carry mode
yellow_bottom_reached_in_carry = False # Whether fitted yellow line has reached a bottom corner
yellow_carry_confirm_count = 0 # Consecutive carry-mode yellow hits before confirmation
YELLOW_CARRY_IGNORE_FRAMES = 4 # Ignore stale yellow line right after entering carry mode
carry_start_frame = -1

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

def reset_target_tracking_state():
    """清空上一轮搬运留下的目标锁定状态，下一帧从全局重新找场地中央目标。"""
    global lost_frame_count
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_color_id, color_lost_count

    lost_frame_count = 0
    target_color_id = 0
    host_color_id_received = False
    color_track_active = False
    color_track_box = None
    color_track_color_id = 0
    color_lost_count = 0
    reset_bear_acquire_state()

def reset_yellow_state():
    """清空黄线状态，避免新一轮任务继承上一轮的边界/滞回。"""
    global yellow_lost_count, yellow_seen_in_carry, yellow_tracking, yellow_detected
    global yellow_bottom_reached_in_carry, yellow_carry_confirm_count
    global yellow_line_k, yellow_line_b

    yellow_lost_count = 0
    yellow_seen_in_carry = False
    yellow_bottom_reached_in_carry = False
    yellow_carry_confirm_count = 0
    yellow_tracking = False
    yellow_detected = False
    yellow_line_k = 0.0
    yellow_line_b = 0.0

def reset_return_yellow_state():
    global return_yellow_last_y, return_yellow_stable_count
    global return_yellow_detected, return_yellow_y
    global return_stop_x, return_stop_requested
    return_yellow_last_y = -1
    return_yellow_stable_count = 0
    return_yellow_detected = False
    return_yellow_y = 0
    return_stop_x = -1
    return_stop_requested = False

# ======================================================================
# Homography transform for inverse perspective mapping
# ======================================================================

WORLD_X_LIMIT_CM = 250.0
WORLD_Y_MAX_CM = 300.0

def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def pick_top_y_from_strip(blobs):
    if not blobs:
        return None
    top_y = None
    for b in blobs:
        if b.h() < CUT_BLOB_MIN_H:
            continue
        if b.y() + b.h() < CUT_SCAN_Y_MAX - CUT_BLOB_BOTTOM_MARGIN:
            continue
        if top_y is None or b.y() < top_y:
            top_y = b.y()
    if top_y is None:
        return None
    # 黄线可能把地面分成上下两段，只向上桥接一次小间隙。
    bridged_top = None
    for b in blobs:
        if b.h() < CUT_BLOB_MIN_H:
            continue
        by2 = b.y() + b.h()
        if by2 <= top_y and top_y - by2 <= CUT_GAP_BRIDGE and b.y() < top_y:
            if bridged_top is None or b.y() < bridged_top:
                bridged_top = b.y()
    if bridged_top is not None:
        top_y = bridged_top
    return top_y

def update_dynamic_cut(img, frame_count):
    global dynamic_cut_left_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi

    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    top_ys = []
    strip_xs = []
    for i in range(len(CUT_STRIP_ROIS)):
        roi = CUT_STRIP_ROIS[i]
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                               pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            top_ys.append(ty)
            strip_xs.append(CUT_STRIP_XS[i])

    valid_strips = len(top_ys)
    if valid_strips >= CUT_MIN_VALID_STRIPS:
        if max(strip_xs) - min(strip_xs) < CUT_MIN_VALID_X_SPAN:
            valid_strips = 0

    top_y_pick = None
    if valid_strips >= 1:
        top_ys.sort()
        pick_i = (valid_strips - 1) * 2 // 3
        top_y_pick = top_ys[pick_i]
        if valid_strips >= 3 and top_ys[pick_i] - top_ys[0] > CUT_BAFFLE_SPREAD_PX:
            top_y_pick = top_ys[pick_i]
        elif valid_strips >= 2 and top_ys[1] - top_ys[0] > CUT_SINGLE_STRIP_MAX_LEAD:
            top_y_pick = top_ys[1]

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = top_y_pick
            dynamic_cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            delta = top_y_pick - dynamic_cut_left_y
            if delta < -CUT_MAX_STEP_UP:
                top_y_pick = dynamic_cut_left_y - CUT_MAX_STEP_UP
            elif delta > CUT_MAX_STEP_DOWN:
                top_y_pick = dynamic_cut_left_y + CUT_MAX_STEP_DOWN
            dynamic_cut_left_y = int(
                a * top_y_pick + (1.0 - a) * dynamic_cut_left_y)

        dynamic_cut_left_y = clamp_int(dynamic_cut_left_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
    else:
        dynamic_cut_miss_count += 1
        if dynamic_cut_miss_count > CUT_MAX_MISS:
            dynamic_cut_valid = False
            dynamic_cut_left_y = DETECT_Y_MIN

    if dynamic_cut_valid:
        y_base = dynamic_cut_left_y + CUT_ROI_Y_OFFSET
        y_base = clamp_int(y_base, DETECT_Y_MIN, 239)
    else:
        y_base = DETECT_Y_MIN
    dynamic_detect_roi = (0, y_base, 320, 240 - y_base)

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

def box_area_change_percent(a, b):
    area_a = a[2] * a[3]
    area_b = b[2] * b[3]
    if area_a <= 0 or area_b <= 0:
        return 1000
    return abs(area_a - area_b) * 100 // area_b

def reset_bear_acquire_state():
    global bear_acquire_color_id, bear_acquire_box
    global bear_acquire_count, bear_acquire_last_frame
    bear_acquire_color_id = 0
    bear_acquire_box = None
    bear_acquire_count = 0
    bear_acquire_last_frame = -1

def valid_bear_acquire_blob(blob):
    return (blob.pixels() >= BEAR_ACQUIRE_MIN_PIXELS and
            blob.w() * blob.h() >= BEAR_ACQUIRE_MIN_AREA)

def confirm_new_bear_target(found):
    global bear_acquire_color_id, bear_acquire_box
    global bear_acquire_count, bear_acquire_last_frame

    if not found:
        reset_bear_acquire_state()
        return None

    color_id, blob = found
    if color_id != 4 and color_id != 5:
        reset_bear_acquire_state()
        return found
    if color_track_active and color_track_color_id == color_id:
        reset_bear_acquire_state()
        return found
    if not valid_bear_acquire_blob(blob):
        reset_bear_acquire_state()
        return None

    box = (blob.x(), blob.y(), blob.w(), blob.h())
    consecutive = (bear_acquire_color_id == color_id and
                   bear_acquire_box is not None and
                   bear_acquire_last_frame == frame_count - 1 and
                   center_dist2(box, bear_acquire_box) <= BEAR_ACQUIRE_MAX_JUMP2 and
                   box_area_change_percent(box, bear_acquire_box)
                   <= BEAR_ACQUIRE_MAX_AREA_CHANGE_PERCENT)
    if consecutive:
        bear_acquire_count += 1
    else:
        bear_acquire_count = 1
    bear_acquire_color_id = color_id
    bear_acquire_box = box
    bear_acquire_last_frame = frame_count

    if bear_acquire_count < BEAR_ACQUIRE_CONFIRM_FRAMES:
        return None
    reset_bear_acquire_state()
    return found

def stabilize_target_box(previous, current, color_id):
    if color_id != 5 or previous is None:
        return current
    if (center_dist2(previous, current) > WHT_BEAR_SMOOTH_MAX_JUMP2
            and box_iou(previous, current) < TRACK_MIN_IOU):
        return current
    old_num = WHT_BEAR_BOX_EMA_DEN - WHT_BEAR_BOX_EMA_NEW_NUM
    values = []
    for i in range(4):
        value = (previous[i] * old_num
                 + current[i] * WHT_BEAR_BOX_EMA_NEW_NUM
                 + WHT_BEAR_BOX_EMA_DEN // 2) // WHT_BEAR_BOX_EMA_DEN
        values.append(value)
    x = clamp_int(values[0], 0, 319)
    y = clamp_int(values[1], 0, 239)
    w = clamp_int(values[2], 1, 320 - x)
    h = clamp_int(values[3], 1, 240 - y)
    return (x, y, w, h)

def make_roi_from_box(box):
    if not box:
        x, y, w, h = dynamic_detect_roi
        y1 = min(y + h, COLOR_DETECT_Y_MAX)
        return (x, y, w, max(1, y1 - y))
    x, y, w, h = box
    y_floor = dynamic_detect_roi[1] if ENABLE_DYNAMIC_CUT and dynamic_cut_valid else DETECT_Y_MIN
    y_floor = clamp_int(y_floor, DETECT_Y_MIN, COLOR_DETECT_Y_MAX - 1)
    x0 = clamp_int(x - COLOR_TRACK_MARGIN, 0, 319)
    y0 = clamp_int(y - COLOR_TRACK_MARGIN, y_floor, COLOR_DETECT_Y_MAX - 1)
    x1 = clamp_int(x + w + COLOR_TRACK_MARGIN, x0 + 1, 320)
    y1 = clamp_int(y + h + COLOR_TRACK_MARGIN, y0 + 1, COLOR_DETECT_Y_MAX)
    return (x0, y0, x1 - x0, y1 - y0)

def valid_color_blob(blob, color_id, pixels_threshold_override=0):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    box_area = w * h
    if blob.y() > NEAR_NOISE_Y_MIN and box_area < NEAR_NOISE_BOX_AREA:
        return False
    pixels_threshold, area_threshold = _color_blob_limits[color_id - 1]
    if pixels_threshold_override > 0:
        pixels_threshold = pixels_threshold_override
    if blob.pixels() < pixels_threshold or box_area < area_threshold:
        return False
    if color_id == 3:
        if w * 100 < h * 45 or w * 100 > h * 185:
            return False
        if blob.density() < 0.35:
            return False
    elif color_id == 4 or color_id == 5:
        if w * 100 < h * 30 or w * 100 > h * 250:
            return False
        if blob.density() < 0.25:
            return False
    else:
        if w * 100 < h * 60 or w * 100 > h * 180:
            return False
        if blob.density() < 0.40:
            return False
    return True

def color_id_from_blob_code(code):
    # Overlapping thresholds are ambiguous; never resolve them by lower ID.
    if code <= 0 or code & (code - 1):
        return 0
    color_id = 1
    while code > 1:
        code >>= 1
        color_id += 1
    if color_id > len(all_color_thresholds):
        return 0
    return color_id

class _MergedColorBlob:
    """Minimal blob-compatible box used when firmware lacks margin merging."""
    def __init__(self, blob):
        self._x = int(blob.x())
        self._y = int(blob.y())
        self._w = int(blob.w())
        self._h = int(blob.h())
        self._pixels = int(blob.pixels())

    def x(self):
        return self._x

    def y(self):
        return self._y

    def w(self):
        return self._w

    def h(self):
        return self._h

    def cx(self):
        return self._x + self._w // 2

    def cy(self):
        return self._y + self._h // 2

    def pixels(self):
        return self._pixels

    def density(self):
        area = self._w * self._h
        return self._pixels / float(area) if area > 0 else 0.0

    def rect(self):
        return (self._x, self._y, self._w, self._h)

    def absorb(self, other):
        x0 = min(self._x, other._x)
        y0 = min(self._y, other._y)
        x1 = max(self._x + self._w, other._x + other._w)
        y1 = max(self._y + self._h, other._y + other._h)
        self._x = x0
        self._y = y0
        self._w = x1 - x0
        self._h = y1 - y0
        self._pixels += other._pixels

def _blob_boxes_near(a, b, margin):
    return not (a.x() + a.w() + margin < b.x()
                or b.x() + b.w() + margin < a.x()
                or a.y() + a.h() + margin < b.y()
                or b.y() + b.h() + margin < a.y())

def _merge_nearby_color_blobs(blobs, margin):
    groups = []
    if not blobs:
        return groups
    for blob in blobs:
        merged = _MergedColorBlob(blob)
        changed = True
        while changed:
            changed = False
            i = 0
            while i < len(groups):
                if _blob_boxes_near(merged, groups[i], margin):
                    merged.absorb(groups[i])
                    del groups[i]
                    changed = True
                else:
                    i += 1
        groups.append(merged)
    return groups

def find_color_blobs_once(img, roi, fixed_color_id=0, pixels_threshold_override=0):
    if fixed_color_id > 0:
        thresholds = _color_threshold_groups[fixed_color_id - 1]
        pixels_threshold, area_threshold = _color_blob_limits[fixed_color_id - 1]
        merge = True
    else:
        thresholds = all_color_thresholds
        pixels_threshold = MULTICOLOR_MIN_PIXELS
        area_threshold = MULTICOLOR_MIN_AREA
        # Do not merge connected regions belonging to different color codes.
        merge = False
    if pixels_threshold_override > 0:
        pixels_threshold = pixels_threshold_override
    try:
        if fixed_color_id == 4 or fixed_color_id == 5:
            margin = (BRN_BEAR_MERGE_MARGIN if fixed_color_id == 4
                      else WHT_BEAR_MERGE_MARGIN)
            try:
                return img.find_blobs(thresholds, roi=roi,
                                      pixels_threshold=pixels_threshold,
                                      area_threshold=area_threshold, merge=True,
                                      margin=margin)
            except TypeError:
                pass
        return img.find_blobs(thresholds, roi=roi,
                              pixels_threshold=pixels_threshold,
                              area_threshold=area_threshold, merge=merge)
    except Exception:
        return None

def find_merged_bear_blobs(img, roi, color_id):
    if color_id not in (4, 5) or len(_color_threshold_groups) < color_id:
        return None
    index = color_id - 1
    pixels_threshold, area_threshold = _color_blob_limits[index]
    if color_id == 4:
        pixels_threshold = BRN_BEAR_FRAGMENT_MIN_PIXELS
        area_threshold = BRN_BEAR_FRAGMENT_MIN_AREA
    margin = (BRN_BEAR_MERGE_MARGIN if color_id == 4
              else WHT_BEAR_MERGE_MARGIN)
    try:
        try:
            return img.find_blobs(_color_threshold_groups[index], roi=roi,
                                  pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True,
                                  margin=margin)
        except TypeError:
            if color_id == 4:
                raw_blobs = img.find_blobs(
                    _color_threshold_groups[index], roi=roi,
                    pixels_threshold=pixels_threshold,
                    area_threshold=area_threshold, merge=False)
                return _merge_nearby_color_blobs(raw_blobs, margin)
            return img.find_blobs(_color_threshold_groups[index], roi=roi,
                                  pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True)
    except Exception:
        return None

def brown_blob_is_ball_shadow(brown, ball_blobs):
    bx0 = brown.x()
    bx1 = bx0 + brown.w()
    brown_area = brown.w() * brown.h()
    for ball in ball_blobs:
        ball_area = ball.w() * ball.h()
        if brown_area * 100 > ball_area * BRN_BEAR_BALL_SHADOW_MAX_AREA_PERCENT:
            continue
        overlap_x = min(bx1, ball.x() + ball.w()) - max(bx0, ball.x())
        if overlap_x <= 0:
            continue
        if (overlap_x * 100
                < min(brown.w(), ball.w()) * BRN_BEAR_BALL_SHADOW_X_OVERLAP_PERCENT):
            continue
        if brown.y() < ball.cy():
            continue
        if brown.y() > ball.y() + ball.h() + BRN_BEAR_BALL_SHADOW_Y_MARGIN:
            continue
        return True
    return False

def find_color_target(img, last_box):
    tracking = last_box is not None and target_color_id > 0
    roi = make_roi_from_box(last_box if tracking else None)
    fixed_color_id = target_color_id if target_color_id > 0 else 0
    # Brown bear always uses its dedicated fragment-merging pass. Skipping the
    # normal fixed-color pass avoids doing the same LAB scan twice when locked.
    blobs = (None if fixed_color_id == 4
             else find_color_blobs_once(img, roi, fixed_color_id))
    scan_brown = fixed_color_id == 0 or fixed_color_id == 4
    scan_white = (fixed_color_id == 0 and color_track_active
                  and color_track_color_id == 5)
    if fixed_color_id == 0 and blobs:
        for blob in blobs:
            if blob.code() & (1 << 4):
                scan_white = True
    brown_roi = (make_roi_from_box(last_box)
                 if scan_brown and last_box is not None
                 and color_track_active and color_track_color_id == 4 else roi)
    white_roi = (make_roi_from_box(last_box)
                  if scan_white and last_box is not None
                  and color_track_active and color_track_color_id == 5 else roi)
    brown_blobs = (find_merged_bear_blobs(img, brown_roi, 4)
                   if scan_brown else None)
    white_blobs = (find_merged_bear_blobs(img, white_roi, 5)
                   if scan_white else None)
    if not blobs and not brown_blobs and not white_blobs:
        return None
    candidates = []
    if blobs:
        for blob in blobs:
            color_id = (fixed_color_id if fixed_color_id > 0
                        else color_id_from_blob_code(blob.code()))
            if color_id <= 0:
                continue
            if ((color_id == 4 and brown_blobs)
                    or (color_id == 5 and white_blobs)):
                continue
            candidates.append((color_id, blob))
    if brown_blobs:
        for blob in brown_blobs:
            candidates.append((4, blob))
    if white_blobs:
        for blob in white_blobs:
            candidates.append((5, blob))
    ball_shadow_refs = []
    for color_id, blob in candidates:
        if color_id == 3 and valid_color_blob(blob, 3):
            ball_shadow_refs.append(blob)
    if fixed_color_id == 4:
        fixed_ball_blobs = find_color_blobs_once(img, roi, 3)
        if fixed_ball_blobs:
            for blob in fixed_ball_blobs:
                if valid_color_blob(blob, 3):
                    ball_shadow_refs.append(blob)
    best_blob = None
    best_color_id = 0
    best_score = None
    best_distance = None
    for color_id, blob in candidates:
        if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
            # Keep targets that straddle the boundary; reject only blobs wholly above it.
            if blob.y() + blob.h() < dynamic_cut_left_y + CUT_BLOB_DELTA:
                continue
        if not valid_color_blob(blob, color_id):
            continue
        if ((color_id == 4 or color_id == 5) and
                (not color_track_active or color_track_color_id != color_id) and
                not valid_bear_acquire_blob(blob)):
            continue
        if color_id == 4 and brown_blob_is_ball_shadow(blob, ball_shadow_refs):
            continue
        if tracking:
            b_box = (blob.x(), blob.y(), blob.w(), blob.h())
            dist2 = center_dist2(b_box, last_box)
            iou = box_iou(b_box, last_box)
            area_change = box_area_change_percent(b_box, last_box)
            if area_change > TRACK_AREA_CHANGE_MAX_PERCENT:
                continue
            if dist2 <= TRACK_MAX_JUMP2 or iou >= TRACK_MIN_IOU:
                score = int(iou * 100000) - dist2 + blob.pixels() // 8
                if best_blob is None or score > best_score:
                    best_blob = blob
                    best_color_id = color_id
                    best_score = score
        else:
            distance = 240 - (blob.y() + blob.h())
            if (best_blob is None or distance < best_distance or
                    (distance == best_distance and
                     (blob.x(), -blob.pixels()) <
                     (best_blob.x(), -best_blob.pixels()))):
                best_blob = blob
                best_color_id = color_id
                best_distance = distance
    if best_blob is not None:
        return (best_color_id, best_blob)
    return None

def front_scan_current_target():
    if color_track_active and color_track_box:
        return color_track_box, color_track_color_id
    return None, target_color_id

def front_scan_blob_is_current(blob, current_box):
    if not current_box:
        return False
    b_box = (blob.x(), blob.y(), blob.w(), blob.h())
    if box_iou(b_box, current_box) >= FRONT_SCAN_EXCLUDE_IOU:
        return True
    return center_dist2(b_box, current_box) <= FRONT_SCAN_EXCLUDE_CENTER2

def front_scan_roi():
    x, y, w, h = dynamic_detect_roi
    y2 = min(y + h, FRONT_SCAN_Y_MAX)
    if y2 <= y:
        return None
    return (x, y, w, y2 - y)

def scan_front_other_color_ids(img):
    current_box, current_id = front_scan_current_target()
    mask = 0
    count = 0
    roi = front_scan_roi()
    if roi is None:
        return current_id, mask, count
    blobs = find_color_blobs_once(img, roi, 0, FRONT_SCAN_MIN_PIXELS)
    if not blobs:
        return current_id, mask, count
    for blob in blobs:
        color_id = color_id_from_blob_code(blob.code())
        if color_id <= 0:
            continue
        color_bit = 1 << (color_id - 1)
        if mask & color_bit:
            continue
        if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
            if blob.y() + blob.h() < dynamic_cut_left_y + CUT_BLOB_DELTA:
                continue
        if blob.pixels() <= FRONT_SCAN_MIN_PIXELS:
            continue
        if not valid_color_blob(blob, color_id, FRONT_SCAN_MIN_PIXELS + 1):
            continue
        if front_scan_blob_is_current(blob, current_box):
            continue
        mask |= color_bit
        count += 1
    return current_id, mask, count

def send_front_scan_result(current_id, mask, count):
    data = bytearray(7)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = FRONT_SCAN_PACKET_ID
    data[3] = current_id & 0xFF
    data[4] = mask & 0xFF
    data[5] = count & 0xFF
    data[6] = (data[2] + data[3] + data[4] + data[5]) & 0xFF
    uart.write(data)

_tx_return_yellow_buf = bytearray(7)
_tx_return_yellow_buf[0] = 0xAA
_tx_return_yellow_buf[1] = 0x55
_tx_return_yellow_buf[2] = RETURN_YELLOW_PACKET_ID

def send_return_yellow_result(valid, y, stop_requested):
    """发送 AA 55 C8 status y_lo y_hi checksum。"""
    data = _tx_return_yellow_buf
    status = RETURN_STATUS_STOP if stop_requested else 0x00
    if valid:
        y = clamp_int(int(y), 0, 239)
        status |= RETURN_STATUS_Y_VALID
    else:
        y = 0
    data[3] = status
    data[4] = y & 0xFF
    data[5] = (y >> 8) & 0xFF
    data[6] = (data[2] + data[3] + data[4] + data[5]) & 0xFF
    uart.write(data)

def reset_front_scan_state():
    global front_scan_last_current_id, front_scan_last_mask
    global front_scan_last_count, front_scan_stable_count, front_scan_total_count
    front_scan_last_current_id = 0
    front_scan_last_mask = -1
    front_scan_last_count = 0
    front_scan_stable_count = 0
    front_scan_total_count = 0

def process_front_scan_request(img):
    global front_scan_requested
    global front_scan_last_current_id, front_scan_last_mask
    global front_scan_last_count, front_scan_stable_count, front_scan_total_count
    if not front_scan_requested:
        return False
    front_scan_total_count += 1
    current_id, mask, count = scan_front_other_color_ids(img)
    if (current_id == front_scan_last_current_id and
            mask == front_scan_last_mask and
            count == front_scan_last_count):
        front_scan_stable_count += 1
    else:
        front_scan_last_current_id = current_id
        front_scan_last_mask = mask
        front_scan_last_count = count
        front_scan_stable_count = 1
    if (front_scan_stable_count >= FRONT_SCAN_STABLE_FRAMES or
            front_scan_total_count >= FRONT_SCAN_MAX_FRAMES):
        send_front_scan_result(current_id, mask, count)
        front_scan_requested = False
        reset_front_scan_state()
    return True

def return_line_overlaps_stop_roi(y):
    if y is None:
        return False
    roi_y = RETURN_STOP_ROI[1]
    roi_bottom = roi_y + RETURN_STOP_ROI[3] - 1
    return (y >= roi_y - RETURN_STOP_HORIZONTAL_GUARD and
            y <= roi_bottom + RETURN_STOP_HORIZONTAL_GUARD)

def detect_return_stop_x(img, return_y=None):
    if return_line_overlaps_stop_roi(return_y):
        return None
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_STOP_ROI,
            pixels_threshold=RETURN_STOP_MIN_PIXELS,
            area_threshold=RETURN_STOP_MIN_AREA, merge=True)
    except Exception:
        return None
    if not blobs:
        return None

    # Search the horizontal ROI from right to left.
    best_x = None
    best_pixels = -1
    for blob in blobs:
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
    return best_x

def detect_return_yellow_y(img):
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_YELLOW_ROI,
            pixels_threshold=RETURN_YELLOW_MIN_PIXELS,
            area_threshold=RETURN_YELLOW_MIN_AREA, merge=True)
    except Exception:
        return None
    if not blobs:
        return None

    # Scan the vertical strip from top to bottom and record the first yellow blob.
    best_y = None
    best_top = 241
    best_pixels = -1
    for blob in blobs:
        top = blob.y()
        y = blob.cy()
        pixels = blob.pixels()
        if (best_y is None or top < best_top or
                (top == best_top and pixels > best_pixels)):
            best_y = y
            best_top = top
            best_pixels = pixels
    return best_y

def process_return_yellow(img):
    global return_yellow_last_y, return_yellow_stable_count
    global return_yellow_detected, return_yellow_y
    global return_stop_x, return_stop_requested

    y = detect_return_yellow_y(img)
    stop_x = detect_return_stop_x(img, y)
    return_stop_x = stop_x if stop_x is not None else -1
    if stop_x is not None and stop_x > RETURN_STOP_X_THRESHOLD:
        return_stop_requested = True

    if y is None:
        return_yellow_last_y = -1
        return_yellow_stable_count = 0
        return_yellow_detected = False
        return_yellow_y = 0
        send_return_yellow_result(False, 0, return_stop_requested)
        return

    if (return_yellow_last_y >= 0 and
            abs(y - return_yellow_last_y) <= RETURN_YELLOW_STABLE_DELTA):
        return_yellow_stable_count += 1
    else:
        return_yellow_stable_count = 1
    return_yellow_last_y = y

    if return_yellow_stable_count >= RETURN_YELLOW_STABLE_FRAMES:
        return_yellow_detected = True
        return_yellow_y = y
        send_return_yellow_result(True, y, return_stop_requested)
    else:
        return_yellow_detected = False
        return_yellow_y = 0
        send_return_yellow_result(False, 0, return_stop_requested)

def draw_return_yellow_lines(img):
    if not ENABLE_YELLOW_DRAW or openart_mode != MODE_RETURN:
        return

    img.draw_rectangle(RETURN_YELLOW_ROI, color=(0, 255, 255), thickness=1)
    img.draw_rectangle(RETURN_STOP_ROI, color=(255, 0, 255), thickness=1)
    img.draw_line(RETURN_STOP_X_THRESHOLD, 0,
                  RETURN_STOP_X_THRESHOLD, 239,
                  color=(255, 0, 0), thickness=1)

    if return_yellow_detected:
        img.draw_line(0, return_yellow_y, 319, return_yellow_y,
                      color=(255, 255, 0), thickness=2)
    if return_stop_x >= 0:
        stop_color = ((255, 0, 0)
                      if return_stop_x > RETURN_STOP_X_THRESHOLD
                      else (0, 128, 255))
        img.draw_line(return_stop_x, 0, return_stop_x, 239,
                      color=stop_color, thickness=2)

# Precomputed from the d040b74 four-point competition calibration.
H_PIX2WORLD = (
    -0.789473684210523, 0.020532099479467793, 127.59861191440086,
    -1.8064645824409328e-16, 0.5067596876807382, -167.72758820127169,
    -1.2325598412554425e-17, -0.036184210526315652,
)

def box_to_world(x, y, w, h):
    # IPM describes points on the ground, so use the target's ground-contact
    # point. Averaging the top corners can cross the homography horizon and
    # turn a far positive distance into a very large negative value.
    px = x + w * 0.5
    py = y + h
    den = H_PIX2WORLD[6] * px + H_PIX2WORLD[7] * py + 1.0
    if -1e-10 < den < 1e-10:
        return (0.0, WORLD_Y_MAX_CM)
    wx = (H_PIX2WORLD[0] * px + H_PIX2WORLD[1] * py + H_PIX2WORLD[2]) / den
    wy = (H_PIX2WORLD[3] * px + H_PIX2WORLD[4] * py + H_PIX2WORLD[5]) / den
    # This form also catches zero, NaN and infinities from the horizon.
    if not (wy > 0.0 and wy <= WORLD_Y_MAX_CM):
        wy = WORLD_Y_MAX_CM
    if wx != wx:
        wx = 0.0
    elif wx < -WORLD_X_LIMIT_CM:
        wx = -WORLD_X_LIMIT_CM
    elif wx > WORLD_X_LIMIT_CM:
        wx = WORLD_X_LIMIT_CM
    return (wx, wy)

# ======================================================================
# UART protocol, 16-byte world-coordinate packet
# ======================================================================
# [0-1]  Header: 0xAA 0x55
# [2]    Color ID, 0=no target, 1=light blue, 2=red, 3=ball, 4=brown bear, 5=white bear
# [3-4]  World X in mm, int16 little-endian
# [5-6]  World Y in mm, int16 little-endian
# [7-8]  Pixel width, uint16 little-endian
# [9]    Yellow-line flag
# [10]   Position flag
# [11]   Reserved obstacle flag, currently zero
# [12-14] Reserved angle fields, currently zero
# [15]   Checksum, sum(data[2:15]) & 0xFF

_tx_world_buf = bytearray(16)
_tx_world_no_target_buf = bytearray(16)
_tx_world_buf[0] = _tx_world_no_target_buf[0] = 0xAA
_tx_world_buf[1] = _tx_world_no_target_buf[1] = 0x55

def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00):
    """发送 16 字节世界坐标数据包。
    [0-1]  帧头 0xAA 0x55
    [2]    颜色ID
    [3-4]  世界X (mm, int16, 小端序)
    [5-6]  世界Y (mm, int16, 小端序)
    [7-8]  像素宽度 (uint16)
    [9]    黄线标志 0x00/0x01
    [10]   位置关系 0x00/0x01/0x02
    [11]   预留障碍字段
    [12-14] 预留角度字段，当前固定为0
    [15]   校验和 (data[2:15])
    """
    # Saturate before encoding. Masking an out-of-range positive value into
    # int16 would otherwise make the receiver decode it as a negative value.
    wx_mm = clamp_int(wx_mm, -32768, 32767)
    wy_mm = clamp_int(wy_mm, -32768, 32767)
    data = _tx_world_buf
    data[2] = color_id & 0xFF
    data[3] = wx_mm & 0xFF
    data[4] = (wx_mm >> 8) & 0xFF
    data[5] = wy_mm & 0xFF
    data[6] = (wy_mm >> 8) & 0xFF
    data[7] = pw & 0xFF
    data[8] = (pw >> 8) & 0xFF
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[15] = (data[2] + data[3] + data[4] + data[5] + data[6] +
                data[7] + data[8] + data[9] + data[10]) & 0xFF
    uart.write(data)

def send_world_no_target(yellow_flag=False, pos_flag=0x00):
    # Same 16-byte packet layout as send_world_data(), with color_id/position fields zero.
    """发送无目标的 16 字节世界坐标数据包。"""
    data = _tx_world_no_target_buf
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[15] = (data[9] + data[10]) & 0xFF
    uart.write(data)

def receive_command_from_host():
    """接收RT1021主机命令"""
    global lost_frame_count, openart_mode, carry_start_frame
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    global _cmd_rx_buf, front_scan_requested

    available = uart.any()
    if available:
        chunk = uart.read(available)
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
            return
        if idx > 0:
            _cmd_rx_buf = _cmd_rx_buf[idx:]
        if len(_cmd_rx_buf) < 4:
            return

        command = _cmd_rx_buf[2]
        if command == 0x03 or command == 0x04:
            if len(_cmd_rx_buf) < 5:
                return
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
                host_color_id_received = True
                lost_frame_count = 0
                color_track_active = False
                color_track_box = None
                color_track_color_id = 0
                color_lost_count = 0
        elif command == 0x01:  # Enter carry mode
            front_scan_requested = False
            reset_front_scan_state()
            reset_return_yellow_state()
            openart_mode = MODE_CARRY
            carry_start_frame = frame_count
            reset_yellow_state()
        elif command == 0x04:  # Cross-line angle correction is disabled in competition runtime.
            pass
        elif command == 0x05:  # Legacy return-beacon command is disabled; consume it only.
            pass
        elif command == 0x06:  # Pre-carry front scan for other color IDs
            openart_mode = MODE_SEARCH
            reset_return_yellow_state()
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x07:  # Enter return-mode horizontal yellow-line tracking.
            openart_mode = MODE_RETURN
            front_scan_requested = False
            reset_front_scan_state()
            reset_yellow_state()
            reset_return_yellow_state()
        elif command == 0x00 or command == 0x02:  # Reset to search mode or turn completed
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
            reset_return_yellow_state()
        return

def yellow_blob_overlaps_tracked_target(blob):
    if (openart_mode != MODE_CARRY or not color_track_active
            or color_track_box is None):
        return False
    tx, ty, tw, th = color_track_box
    bx, by, bw, bh = blob.x(), blob.y(), blob.w(), blob.h()
    ix0 = max(tx, bx)
    iy0 = max(ty, by)
    ix1 = min(tx + tw, bx + bw)
    iy1 = min(ty + th, by + bh)
    if ix1 <= ix0 or iy1 <= iy0:
        return False
    overlap_area = (ix1 - ix0) * (iy1 - iy0)
    return overlap_area * 100 >= bw * bh * YELLOW_TARGET_OVERLAP_PERCENT

def pick_largest_blob(blobs, reject_tracked_target=False):
    if not blobs:
        return None
    best = None
    for b in blobs:
        if reject_tracked_target and yellow_blob_overlaps_tracked_target(b):
            continue
        if best is None or b.pixels() > best.pixels():
            best = b
    return best

def yellow_line_y_at_x(x):
    if not yellow_detected:
        return 0
    return int(yellow_line_k * x + yellow_line_b)

def yellow_line_reaches_bottom_corner():
    if not yellow_detected:
        return False
    return (yellow_line_y_at_x(0) >= YELLOW_BOTTOM_Y or
            yellow_line_y_at_x(319) >= YELLOW_BOTTOM_Y)

def draw_carry_yellow_line(img):
    if (not ENABLE_YELLOW_DRAW or
            (openart_mode != MODE_SEARCH and openart_mode != MODE_CARRY) or
            not yellow_detected):
        return
    points = []
    for x in (0, 319):
        y = yellow_line_k * x + yellow_line_b
        if 0 <= y <= 239:
            points.append((x, int(y)))
    if abs(yellow_line_k) > 0.0001:
        for y in (0, 239):
            x = (y - yellow_line_b) / yellow_line_k
            if 0 <= x <= 319:
                point = (int(x), y)
                duplicate = False
                for old_point in points:
                    if old_point == point:
                        duplicate = True
                        break
                if not duplicate:
                    points.append(point)
    if len(points) >= 2:
        img.draw_line(points[0][0], points[0][1],
                      points[1][0], points[1][1], color=(255, 255, 0))

def current_pos_flag(frame_count):
    global yellow_lost_count, yellow_seen_in_carry, yellow_bottom_reached_in_carry
    global yellow_carry_confirm_count, openart_mode
    if openart_mode == MODE_CARRY:
        if carry_start_frame >= 0 and frame_count - carry_start_frame < YELLOW_CARRY_IGNORE_FRAMES:
            yellow_lost_count = 0
            return POS_NO_BOUNDARY
        if yellow_detected:
            if not yellow_seen_in_carry:
                yellow_carry_confirm_count += 1
                if yellow_carry_confirm_count >= YELLOW_CARRY_CONFIRM_FRAMES:
                    yellow_seen_in_carry = True
            yellow_lost_count = 0
            if yellow_seen_in_carry and yellow_line_reaches_bottom_corner():
                yellow_bottom_reached_in_carry = True
            return POS_NO_BOUNDARY
        if not yellow_seen_in_carry:
            yellow_carry_confirm_count = 0
        if yellow_bottom_reached_in_carry:
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

def update_yellow_detection(img, frame_count):
    global yellow_tracking, yellow_detected
    global yellow_line_k, yellow_line_b

    detect_every_frame = openart_mode == MODE_CARRY and yellow_seen_in_carry
    if not detect_every_frame and frame_count % YELLOW_DETECT_INTERVAL != 0:
        return

    top_roi = YELLOW_ROI_TOP
    bottom_roi = YELLOW_ROI_BOTTOM
    yellow_pixels_threshold = (YELLOW_KEEP_PIXELS if yellow_tracking
                               else YELLOW_ENTER_PIXELS)

    top_blobs = img.find_blobs(yellow_threshold, roi=top_roi,
                               pixels_threshold=yellow_pixels_threshold,
                               area_threshold=20, merge=True)
    bottom_blobs = img.find_blobs(yellow_threshold, roi=bottom_roi,
                                  pixels_threshold=yellow_pixels_threshold,
                                  area_threshold=20, merge=True)
    reject_target = openart_mode == MODE_CARRY
    top_blob = pick_largest_blob(top_blobs, reject_target)
    bottom_blob = pick_largest_blob(bottom_blobs, reject_target)

    raw_yellow_seen = top_blob is not None and bottom_blob is not None
    if raw_yellow_seen:
        fit_dx = bottom_blob.cx() - top_blob.cx()
        fit_dy = bottom_blob.cy() - top_blob.cy()
        if (abs(fit_dx) < YELLOW_MIN_FIT_DX
                or abs(fit_dy) * 100 > abs(fit_dx) * YELLOW_MAX_FIT_SLOPE_X100):
            raw_yellow_seen = False

    # Use a higher threshold for initial detection, then a lower hold threshold.
    if raw_yellow_seen:
        yellow_tracking = True
        yellow_detected = True
    else:
        yellow_detected = False
        if openart_mode == MODE_SEARCH or (openart_mode == MODE_CARRY and not yellow_seen_in_carry):
            yellow_tracking = False

    if yellow_detected:
        line_x1 = top_blob.cx()
        line_y1 = top_blob.cy()
        line_x2 = bottom_blob.cx()
        line_y2 = bottom_blob.cy()

        dx = line_x2 - line_x1
        if dx == 0:
            yellow_line_k = 0.0
            yellow_line_b = (line_y1 + line_y2) / 2.0
        else:
            yellow_line_k = (line_y2 - line_y1) / dx
            yellow_line_b = line_y1 - yellow_line_k * line_x1
    elif openart_mode == MODE_SEARCH:
        yellow_line_k = 0.0
        yellow_line_b = 0.0

# ======================================================================
# Main loop
# ======================================================================
frame_count = 0

while True:
    frame_count += 1

    # Receive host command
    receive_command_from_host()

    img = snapshot_frame()

    # ===== Return-mode horizontal yellow line =====
    if openart_mode == MODE_RETURN:
        process_return_yellow(img)
        draw_return_yellow_lines(img)
        if frame_count % 10 == 0:
            gc.collect()
        feed_watchdog()
        continue

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    update_yellow_detection(img, frame_count)
    pos_flag = current_pos_flag(frame_count)
    draw_carry_yellow_line(img)
    if process_front_scan_request(img):
        if frame_count % 10 == 0:
            gc.collect()
        feed_watchdog()
        continue

    # ===== Color blob detection / tracking =====
    has_target = False
    send_color_id = 0
    last_box = color_track_box if color_track_active else None
    try:
        found = find_color_target(img, last_box)
    except Exception as error:
        found = None
        if frame_count % 30 == 1:
            print("[color] find_color_target error: " + str(error))
    found = confirm_new_bear_target(found)

    if found:
        send_color_id, blob = found
        raw_box = (blob.x(), blob.y(), blob.w(), blob.h())
        previous_box = (color_track_box
                        if color_track_active
                        and color_track_color_id == send_color_id else None)
        x1, y1, w, h = stabilize_target_box(
            previous_box, raw_box, send_color_id)
        has_target = True
        color_track_active = True
        color_track_box = (x1, y1, w, h)
        color_track_color_id = send_color_id
        color_lost_count = 0

    elif color_track_active and color_track_box:
        color_lost_count += 1
        if color_lost_count <= COLOR_LOST_FRAMES:
            x1, y1, w, h = color_track_box
            send_color_id = color_track_color_id
            has_target = True
        else:
            color_track_active = False
            color_track_box = None
            color_track_color_id = 0
            color_lost_count = 0

    if has_target:
        lost_frame_count = 0

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag)
        img.draw_rectangle((x1, y1, w, h),
                           color=TARGET_BOX_COLORS[send_color_id - 1], thickness=2)

    else:
        lost_frame_count += 1
        if lost_frame_count > MAX_LOST_FRAMES and (target_color_id > 0 or color_track_active):
            if host_color_id_received:
                color_track_active = False
                color_track_box = None
                color_track_color_id = 0
                color_lost_count = 0
                lost_frame_count = 0
            else:
                reset_target_tracking_state()
        send_world_no_target(yellow_detected, pos_flag)

    if frame_count % 10 == 0:
        gc.collect()
    feed_watchdog()
