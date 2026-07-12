# ======================================================================
# OpenART Plus fixed master-camera runtime - multi-target object detection
# ======================================================================


import sensor, image, time, gc
from machine import UART
try:
    from machine import WDT
except Exception:
    WDT = None

# ======================================================================
# Mode selection
# ======================================================================
CALIBRATION_MODE = False  # True = IPM calibration mode, False = normal detection
SOFTWARE_HMIRROR = True   # Hardware keeps vflip; software only adds hmirror to avoid full-frame flip tearing.
RUNTIME_LENS_CORR = False  # Test switch: disable per-frame lens_corr to isolate frame-buffer pressure.
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

# Fixed exposure
sensor.set_auto_exposure(False, exposure_us=1000)
sensor.set_auto_gain(False, gain_db=0)

# Both master and slave cameras are OpenART Plus boards and use UART12.
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

# Supported color thresholds (fallback defaults; overridden by /sd/color_thr.txt if present)
all_color_thresholds = [
    (34, 100, -41, 4, -72, -22),    # Color 1: light-blue bag
    (10, 80, 22, 122, -17, 93),     # Color 2: red bag
    (50, 100, -128, -27, 20, 127),  # Color 3: tennis ball
    (21, 52, -77, 25, 6, 99),        # Color 4: brown teddy bear; tune on field
    (53, 100, -10, 11, -11, 8)      # Color 5: white teddy bear
]

def _load_calibrated_params(path='/sd/color_thr.txt'):
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
            return [rows[i] for i in range(1, 6)], exposure
        return None, None
    except Exception:
        pass
    return None, None

_loaded, _loaded_exposure = _load_calibrated_params()
if _loaded:
    all_color_thresholds = _loaded
    if _loaded_exposure is not None:
        sensor.set_auto_exposure(False, exposure_us=_loaded_exposure)

_color_threshold_groups = [[threshold] for threshold in all_color_thresholds]

COLOR_SEARCH_ORDER = [1, 2, 3, 4, 5]
_single_color_ids = [[color_id] for color_id in COLOR_SEARCH_ORDER]

COLOR_LOST_FRAMES = 5
COLOR_TRACK_MARGIN = 45
COLOR_MIN_PIXELS = 150
COLOR_MIN_AREA = 100
COLOR_ID12_MIN_PIXELS = 100
TENNIS_COLOR_ID = 3
TENNIS_MIN_PIXELS = 45
TENNIS_MIN_AREA = 45
BEAR_MIN_BOX_AREA = 480

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

# ======================================================================
# Dynamic cut line (based on blue-ground strips on left/right)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
# 深蓝赛道地面 LAB 阈值（B 通道必须为负=偏蓝）。上场后用 IDE 阈值编辑器实测微调：
# 重点看 B_max（约 -15 ~ -25）和 L 范围（深蓝偏暗，L 上限别太高）。
BLUE_GROUND_THRESHOLD = [(0, 55, -30, 45, -90, -7)]
CUT_BLOB_MIN_H = 12          # 条带内蓝色块最小高度，滤掉零星蓝色噪点/浅蓝沙包边缘
CUT_BLOB_BOTTOM_MARGIN = 25  # 蓝色块底部须延伸到条带底部附近，才认为是连续的赛道地面
CUT_GAP_BRIDGE = 10          # 向上桥接的最大间隙(px)：黄线横穿会把蓝地面切成上下两段，跨过它继续延伸
# 多条竖直采样带横跨画面：正对直角弯时赛道尖角在画面中间、比左右两侧更远，
# 只采左右两条再连斜线会割掉尖角；改为取所有带的最高点做一条水平裁切线（保守）。
CUT_STRIP_XS = (10, 85, 160, 235, 310)
CUT_MIN_VALID_STRIPS = 2   # 至少几条带看到蓝地面才认为裁切线有效
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
TRACK_AREA_CHANGE_MAX_PERCENT = 60
TRACK_MIN_IOU = 0.05

dynamic_cut_left_y = DETECT_Y_MIN
dynamic_cut_right_y = DETECT_Y_MIN
dynamic_cut_valid = False
dynamic_cut_miss_count = 0
dynamic_detect_roi = DETECT_ROI
target_color_id = 0
host_color_id_received = False
color_track_active = False
color_track_box = None
color_track_color_id = 0
color_lost_count = 0
_cmd_rx_buf = bytearray()
front_scan_requested = False
FRONT_SCAN_PACKET_ID = 0xC7
FRONT_SCAN_EXCLUDE_IOU = 0.20
FRONT_SCAN_EXCLUDE_CENTER_PX = 35
FRONT_SCAN_Y_MAX = 150
FRONT_SCAN_MIN_PIXELS = 150
FRONT_SCAN_STABLE_FRAMES = 10
FRONT_SCAN_MAX_FRAMES = 30
front_scan_last_current_id = 0
front_scan_last_mask = -1
front_scan_last_count = 0
front_scan_stable_count = 0
front_scan_total_count = 0
crossline_angle_enabled = False
crossline_angle_result = None
# ======================================================================
# Yellow line detection parameters
# ======================================================================
yellow_threshold = [(51, 91, -32, 36, 1, 118)]    # Yellow LAB threshold
YELLOW_ROI_TOP = (0, 90, 320, 20)        # Horizontal strip centered near y=100
YELLOW_ROI_BOTTOM = (0, 130, 320, 20)    # Horizontal strip centered near y=140
YELLOW_DETECT_INTERVAL = 2              # Detect yellow line every N frames
YELLOW_ENTER_PIXELS = 70                # Pixel threshold for first yellow-line hit
YELLOW_KEEP_PIXELS = 8                  # Lower hold threshold after line is seen
YELLOW_CARRY_CONFIRM_FRAMES = 2         # Consecutive hits before carry mode treats yellow as confirmed
YELLOW_BOTTOM_Y = 239                   # Bottom edge used to arm disappear-after-bottom crossing

yellow_line_k = 0.0
yellow_line_b = 0.0
yellow_detected = False     # Whether yellow line is visible
yellow_tracking = False      # Hysteresis state after first yellow-line hit
yellow_lost_count = 0       # Consecutive yellow-line lost counter
YELLOW_LOST_THRESHOLD = 3   # Consecutive per-frame misses after the line reaches a bottom corner
yellow_seen_in_carry = False # Whether yellow line was confirmed in carry mode
yellow_bottom_reached_in_carry = False # Whether fitted yellow line has reached a bottom corner
yellow_carry_confirm_count = 0 # Consecutive carry-mode yellow hits before confirmation
YELLOW_CARRY_IGNORE_FRAMES = 4 # Ignore stale yellow line right after entering carry mode
carry_start_frame = -1

# State machine
MODE_SEARCH = 0
MODE_CARRY = 1
MODE_WAIT_TURN = 2

openart_mode = MODE_SEARCH   # 0=search, 1=carry, 2=wait for turn

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

def cut_line_y_at_x(x):
    # 水平裁切线：全画面统一取所有采样带中蓝地面的最高点（最保守）。
    return dynamic_cut_left_y

def pick_top_y_from_strip(blobs):
    # 在竖条带内找"从底部向上连续延伸的深蓝地面"的最高点。
    # 1) 种子段：色块够高且底部贴近扫描区下沿，避免场外零星蓝色把裁切线误抬高；
    # 2) 桥接：黄线横穿条带会把蓝地面切成上下两段，允许跨过 <=CUT_GAP_BRIDGE 的
    #    间隙继续向上延伸，使裁切线走到赛道真正的远边缘而不是吸附在黄线上。
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
    # 只桥接一次（黄线只横穿一次），且上方接续段本身也要够高；
    # 否则零星蓝色噪点会被迭代桥接一级级把裁切线爬高。
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
    global dynamic_cut_left_y, dynamic_cut_right_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi

    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    top_y_min = None
    valid_strips = 0
    for roi in CUT_STRIP_ROIS:
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                               pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            valid_strips += 1
            if top_y_min is None or ty < top_y_min:
                top_y_min = ty

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = top_y_min
            dynamic_cut_valid = True
        else:
            # 双向对称 EMA：单帧噪声不会瞬间把线顶高（棘轮效应），转弯时也能平滑跟随
            a = CUT_EMA_ALPHA
            dynamic_cut_left_y = int(a * top_y_min + (1.0 - a) * dynamic_cut_left_y)

        dynamic_cut_left_y = clamp_int(dynamic_cut_left_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
        dynamic_cut_right_y = dynamic_cut_left_y
    else:
        dynamic_cut_miss_count += 1
        if dynamic_cut_miss_count > CUT_MAX_MISS:
            dynamic_cut_valid = False
            dynamic_cut_left_y = DETECT_Y_MIN
            dynamic_cut_right_y = DETECT_Y_MIN

    if dynamic_cut_valid:
        y_base = min(dynamic_cut_left_y, dynamic_cut_right_y) + CUT_ROI_Y_OFFSET
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

def make_roi_from_box(box):
    if not box:
        return dynamic_detect_roi
    x, y, w, h = box
    y_floor = dynamic_detect_roi[1] if ENABLE_DYNAMIC_CUT and dynamic_cut_valid else DETECT_Y_MIN
    x0 = clamp_int(x - COLOR_TRACK_MARGIN, 0, 319)
    y0 = clamp_int(y - COLOR_TRACK_MARGIN, y_floor, 239)
    x1 = clamp_int(x + w + COLOR_TRACK_MARGIN, x0 + 1, 320)
    y1 = clamp_int(y + h + COLOR_TRACK_MARGIN, y0 + 1, 240)
    return (x0, y0, x1 - x0, y1 - y0)

def color_ids_for_search():
    if target_color_id > 0:
        return _single_color_ids[target_color_id - 1]
    return COLOR_SEARCH_ORDER

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
        if w * h <= BEAR_MIN_BOX_AREA:
            return False
        if blob.pixels() < 120:
            return False
    else:
        if aspect < 0.60 or aspect > 1.80:
            return False
        if blob.density() < 0.40:
            return False
    return True

def color_blob_thresholds(color_id):
    if color_id == TENNIS_COLOR_ID:
        return (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA)
    if color_id == 1 or color_id == 2:
        return (COLOR_ID12_MIN_PIXELS, COLOR_MIN_AREA)
    return (COLOR_MIN_PIXELS, COLOR_MIN_AREA)

def pick_initial_color_candidate(candidates):
    # For the same color, acquire the leftmost valid blob first. Keep the
    # existing cross-color priority by comparing one representative per color.
    representatives = [None] * len(COLOR_SEARCH_ORDER)
    for item in candidates:
        index = item[0] - 1
        current = representatives[index]
        if current is None:
            representatives[index] = item
            continue
        blob = item[1]
        current_blob = current[1]
        if ((blob.cx(), blob.x(), -blob.pixels()) <
                (current_blob.cx(), current_blob.x(), -current_blob.pixels())):
            representatives[index] = item

    best = None
    best_distance = None
    for color_id in COLOR_SEARCH_ORDER:
        item = representatives[color_id - 1]
        if item is None:
            continue
        blob = item[1]
        distance = 240 - (blob.y() + blob.h())
        if best is None or distance < best_distance:
            best = item
            best_distance = distance
    return best

def find_color_target(img, last_box):
    color_ids = color_ids_for_search()
    roi = make_roi_from_box(last_box)
    candidates = []
    for color_id in color_ids:
        if last_box and color_track_color_id > 0 and color_id != color_track_color_id:
            continue
        pixels_threshold, area_threshold = color_blob_thresholds(color_id)
        try:
            blobs = img.find_blobs(_color_threshold_groups[color_id - 1], roi=roi,
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
    if not candidates:
        return None
    if last_box:
        best_item = None
        best_score = None
        max_jump2 = TRACK_MAX_JUMP_PX * TRACK_MAX_JUMP_PX
        for item in candidates:
            b = item[1]
            b_box = (b.x(), b.y(), b.w(), b.h())
            dist2 = center_dist2(b_box, last_box)
            iou = box_iou(b_box, last_box)
            area_change = box_area_change_percent(b_box, last_box)
            if area_change > TRACK_AREA_CHANGE_MAX_PERCENT:
                continue
            if dist2 <= max_jump2 or iou >= TRACK_MIN_IOU:
                score = int(iou * 100000) - dist2 + b.pixels() // 8
                if best_item is None or score > best_score:
                    best_item = item
                    best_score = score
        return best_item
    return pick_initial_color_candidate(candidates)

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
    return center_dist2(b_box, current_box) <= FRONT_SCAN_EXCLUDE_CENTER_PX * FRONT_SCAN_EXCLUDE_CENTER_PX

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
    for color_id in COLOR_SEARCH_ORDER:
        if color_id < 1 or color_id > len(all_color_thresholds):
            continue
        pixels_threshold, area_threshold = color_blob_thresholds(color_id)
        try:
            blobs = img.find_blobs(_color_threshold_groups[color_id - 1], roi=roi,
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
            if blob.pixels() <= FRONT_SCAN_MIN_PIXELS:
                continue
            if not valid_color_blob(blob, color_id):
                continue
            if front_scan_blob_is_current(blob, current_box):
                continue
            mask |= 1 << (color_id - 1)
            count += 1
            break
    return current_id, mask, count

def send_front_scan_result(current_id, mask, count):
    data = bytearray(7)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = FRONT_SCAN_PACKET_ID
    data[3] = current_id & 0xFF
    data[4] = mask & 0xFF
    data[5] = count & 0xFF
    data[6] = sum(data[2:6]) & 0xFF
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

def box_to_world(x, y, w, h):
    if H_pix2world is None:
        return (0.0, 0.0)
    # IPM describes points on the ground, so use the target's ground-contact
    # point. Averaging the top corners can cross the homography horizon and
    # turn a far positive distance into a very large negative value.
    wx, wy = pixel_to_world(x + w / 2.0, y + h, H_pix2world)
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
# [12]   Angle status flag
# [13-14] Cross-line angle, int16 little-endian, degree * 100
# [15]   Checksum, sum(data[2:15]) & 0xFF

_tx_world_buf = bytearray(16)
_tx_world_no_target_buf = bytearray(16)
_tx_world_buf[0] = _tx_world_no_target_buf[0] = 0xAA
_tx_world_buf[1] = _tx_world_no_target_buf[1] = 0x55

def _calculate_checksum_range(data, start, end):
    checksum = 0
    for i in range(start, end):
        checksum += data[i]
    return checksum & 0xFF

def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                    angle_flag=0x00, angle_cdeg=0):
    # World packet v2, 16 bytes:
    # [12] angle_flag: bit0=angle enabled, bit1=angle valid
    # [13-14] crossline angle, int16 little-endian, degree * 100
    # [15] checksum = sum(data[2:15]) & 0xFF
    """发送 16 字节世界坐标数据包。
    [0-1]  帧头 0xAA 0x55
    [2]    颜色ID
    [3-4]  世界X (mm, int16, 小端序)
    [5-6]  世界Y (mm, int16, 小端序)
    [7-8]  像素宽度 (uint16)
    [9]    黄线标志 0x00/0x01
    [10]   位置关系 0x00/0x01/0x02
    [11]   预留障碍字段
    [12]   角度状态
    [13-14] 黄线角度
    [15]   校验和 (data[2:15])
    """
    # Saturate before encoding. Masking an out-of-range positive value into
    # int16 would otherwise make the receiver decode it as a negative value.
    wx_mm = clamp_int(int(wx_mm), -32768, 32767)
    wy_mm = clamp_int(int(wy_mm), -32768, 32767)
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
    data[11] = obstacle_flag & 0xFF
    data[12] = angle_flag & 0xFF
    data[13] = angle_cdeg & 0xFF
    data[14] = (angle_cdeg >> 8) & 0xFF
    data[15] = _calculate_checksum_range(data, 2, 15)
    uart.write(data)

def send_world_no_target(yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                         angle_flag=0x00, angle_cdeg=0):
    # Same 16-byte packet layout as send_world_data(), with color_id/position fields zero.
    """发送无目标的 16 字节世界坐标数据包。"""
    data = _tx_world_no_target_buf
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[11] = obstacle_flag & 0xFF
    data[12] = angle_flag & 0xFF
    data[13] = angle_cdeg & 0xFF
    data[14] = (angle_cdeg >> 8) & 0xFF
    data[15] = _calculate_checksum_range(data, 2, 15)
    uart.write(data)

def receive_command_from_host():
    """接收RT1021主机命令"""
    global lost_frame_count, openart_mode, carry_start_frame
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    global _cmd_rx_buf, crossline_angle_enabled, crossline_angle_result, front_scan_requested
    global yellow_seen_in_carry, yellow_tracking, yellow_detected

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
            openart_mode = MODE_CARRY
            carry_start_frame = frame_count
            reset_yellow_state()
        elif command == 0x04:  # SET_CROSSLINE_ANGLE_ENABLE
            crossline_angle_enabled = (param == 1)
            if not crossline_angle_enabled:
                crossline_angle_result = None
        elif command == 0x05:  # Return vision is disabled; consume and ignore broadcasts.
            pass
        elif command == 0x06:  # Pre-carry front scan for other color IDs
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x00 or command == 0x02:  # Reset to search mode or turn completed
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
            crossline_angle_enabled = False
            crossline_angle_result = None
        return

def pick_largest_blob(blobs):
    if not blobs:
        return None
    best = blobs[0]
    for b in blobs:
        if b.pixels() > best.pixels():
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

    detect_every_frame = openart_mode == MODE_CARRY and yellow_bottom_reached_in_carry
    if not detect_every_frame and frame_count % YELLOW_DETECT_INTERVAL != 0:
        return

    yellow_pixels_threshold = YELLOW_KEEP_PIXELS if yellow_tracking else YELLOW_ENTER_PIXELS

    top_blobs = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_TOP,
                               pixels_threshold=yellow_pixels_threshold,
                               area_threshold=20, merge=True)
    bottom_blobs = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_BOTTOM,
                                  pixels_threshold=yellow_pixels_threshold,
                                  area_threshold=20, merge=True)
    top_blob = pick_largest_blob(top_blobs)
    bottom_blob = pick_largest_blob(bottom_blobs)

    raw_yellow_seen = top_blob and bottom_blob

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

while True:
    # FPS accounting
    clock.tick()
    frame_count += 1

    # Receive host command
    receive_command_from_host()

    # Capture image and apply lens correction
    img = snapshot_frame(apply_lens_corr=RUNTIME_LENS_CORR)

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    if process_front_scan_request(img):
        if frame_count % 10 == 0:
            gc.collect()
        feed_watchdog()
        continue
    obstacle_flag = 0
    update_yellow_detection(img, frame_count)

    # ===== Color blob detection / tracking =====
    best = None
    send_color_id = 0
    last_box = color_track_box if color_track_active else None
    found = find_color_target(img, last_box)

    if found:
        send_color_id, blob = found
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
        best = (send_color_id, x1, y1, w, h)
        color_track_active = True
        color_track_box = (x1, y1, w, h)
        color_track_color_id = send_color_id
        color_lost_count = 0

    if not found and color_track_active and color_track_box:
        color_lost_count += 1
        if color_lost_count <= COLOR_LOST_FRAMES:
            x1, y1, w, h = color_track_box
            send_color_id = color_track_color_id
            best = (send_color_id, x1, y1, w, h)
        else:
            color_track_active = False
            color_track_box = None
            color_track_color_id = 0
            color_lost_count = 0

    pos_flag = current_pos_flag(frame_count)
    angle_flag, angle_cdeg = get_crossline_angle_fields()

    if best:
        lost_frame_count = 0
        send_color_id, x1, y1, w, h = best

        # Local detection is only a candidate report; final color lock comes from host 0x03.

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag, obstacle_flag,
                        angle_flag, angle_cdeg)

        color = (255, 0, 0)
        if send_color_id == 1:
            color = (0, 170, 255)
        elif send_color_id == 3:
            color = (0, 255, 0)
        elif send_color_id == 4:
            color = (160, 96, 32)
        elif send_color_id == 5:
            color = (255, 255, 255)
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)

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
        send_world_no_target(yellow_detected, pos_flag, obstacle_flag, angle_flag, angle_cdeg)
    # Yellow detection is updated before pos_flag is calculated.

    if frame_count % 10 == 0:
        gc.collect()
    feed_watchdog()
