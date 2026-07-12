# ======================================================================
# OpenART Plus fixed slave-camera runtime - multi-color target detection
# ======================================================================


import sensor, image, time
from machine import UART

# ======================================================================
# 模式选择
# ======================================================================
CALIBRATION_MODE = False   # True = 逆透视标定模式, False = 正常识别
SOFTWARE_HMIRROR = True    # Hardware keeps vflip; software only adds hmirror to avoid full replace tearing.
RUNTIME_LENS_CORR = False  # Match main.py runtime image path; calibration mode still calls snapshot_frame() directly.

# ======================================================================
# 硬件初始化
# ======================================================================

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)

# Firmware may only keep one hardware flip; keep vertical in hardware and add hmirror in snapshot_frame().
sensor.set_hmirror(False)
sensor.set_vflip(True)

sensor.skip_frames(time=500)

def snapshot_frame(apply_lens_corr=False):
    img = sensor.snapshot()
    if apply_lens_corr:
        img = img.lens_corr(2)
    if SOFTWARE_HMIRROR:
        img = img.replace(hmirror=True)
    return img

# 白平衡配置
# True = 使用固定增益(比赛用), False = 自动收敛后锁定(调试用)
WB_FIXED = True
# 固定白平衡增益 (R_db, G_db, B_db)
# 用 wb_calibrate.py 在比赛灯光下标定后填入
WB_GAINS = (101.00, 64.00, 97.00) # 写死白平衡

if WB_FIXED:
    sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
    sensor.skip_frames(time=500)
else:
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)

# 固定曝光
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)

# Both master and slave cameras are OpenART Plus boards and use UART12.
uart = UART(12, baudrate=115200)

# 帧率计时器
clock = time.clock()

# ======================================================================
# 颜色阈值配置 (LAB色彩空间) - 多颜色动态检测
# ======================================================================
# LAB格式: (L_min, L_max, A_min, A_max, B_min, B_max)
# L: 亮度 (0-100)
# A: 红绿轴 (正值=红色, 负值=绿色)
# B: 黄蓝轴 (正值=黄色, 负值=蓝色)

# 所有支持的颜色阈值；如果 /sd/color_thr.txt 完整有效，会在启动时覆盖这些默认值。
all_color_thresholds = [
    (34, 100, -41, 5, -72, -17),    # 颜色1: 淡蓝色沙包
    (10, 80, 22, 122, -17, 93),    # 颜色2: 红色沙包
    (50, 100, -128, -27, 20, 127), # 颜色3: 网球(浅绿/荧光黄绿)
    (21, 52, -77, 25, 1, 99),      # 颜色4: 棕色泰迪熊 ← 需实际标定!
    (53, 100, -10, 11, -11, 8)    # 颜色5: 白色泰迪熊 ← 需实际标定!
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
# Runtime target detection uses LAB color blobs only.

# 蓝色背景布阈值 (示例，需实测)
# 重点看 B 通道，蓝色通常在 -20 以下
# BLUE_GROUND_THRESHOLD = (10, 50, -20, 50, -77, -25)

# 目标丢失检测
lost_frame_count = 0                    # 连续丢失目标的帧数
MAX_LOST_FRAMES = 30                    # 最大允许丢失帧数 (30帧 ≈ 0.5秒)

# ======================================================================
# 识别参数配置
# ======================================================================
DETECT_Y_MIN = 8           # 主检测区域起始Y：忽略 y < 8 的区域
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)

# ======================================================================
# Dynamic cut line (based on blue-ground strips on left/right)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = [(34, 51, -54, 78, -104, -12)]
CUT_LEFT_X = 10
CUT_RIGHT_X = 310
CUT_STRIP_HALF_W = 2
CUT_SCAN_Y_MIN = 0
CUT_SCAN_Y_MAX = 140
CUT_STRIP_H = CUT_SCAN_Y_MAX - CUT_SCAN_Y_MIN
CUT_LEFT_ROI = (CUT_LEFT_X - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN,
                CUT_STRIP_HALF_W * 2 + 1, CUT_STRIP_H)
CUT_RIGHT_ROI = (CUT_RIGHT_X - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN,
                 CUT_STRIP_HALF_W * 2 + 1, CUT_STRIP_H)
CUT_UPDATE_INTERVAL = 2
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_ROI_Y_OFFSET = -10      # Target ROI starts 10 px above the detected blue-ground boundary.
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

# 目标连续性过滤
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
# ======================================================================
# 黄线检测参数
# ======================================================================
yellow_threshold = [(56, 100, -56, -2, 41, 127)]    # 黄色阈值 (LAB)
YELLOW_ROI_LEFT = (0, 80, 70, 160)       # 左侧垂直条，延伸到图像底部
YELLOW_ROI_RIGHT = (250, 80, 70, 160)    # 右侧垂直条，延伸到图像底部
YELLOW_DETECT_INTERVAL = 3              # 每3帧检测一次黄线
YELLOW_ENTER_PIXELS = 10                # 首次看到黄线需要达到的像素阈值
YELLOW_KEEP_PIXELS = 3                  # 已看到黄线后的保持阈值，防止边缘闪烁
YELLOW_SCAN_STRIP_H = 12                # 搬运模式下从屏幕底部往上分条扫描黄线

# 黄线边界状态
yellow_detected = False     # 黄线是否被检测到
yellow_raw_detected = False # 当前检测周期是否真实看到黄线
yellow_tracking = False      # 黄线滞回状态：首次看到后使用保持阈值
yellow_lost_count = 0       # 黄线连续丢失帧数
YELLOW_LOST_THRESHOLD = 2   # 连续丢失N次检测才判定过线
yellow_seen_in_carry = False # 进入搬运模式后是否已确认看到过黄线（防掉头误触发）
YELLOW_CARRY_HOLD_FRAMES = 40 # 搬运模式下首次看到黄线后保持的帧数
yellow_carry_hold_count = 0

# 状态机
MODE_SEARCH = 0
MODE_CARRY = 1
MODE_WAIT_TURN = 2

openart_mode = MODE_SEARCH   # 0=寻找对准, 1=搬运中, 2=等待右转完成

# 位置关系常量
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
    global yellow_lost_count, yellow_seen_in_carry, yellow_tracking, yellow_detected, yellow_raw_detected
    global yellow_carry_hold_count

    yellow_lost_count = 0
    yellow_seen_in_carry = False
    yellow_carry_hold_count = 0
    yellow_tracking = False
    yellow_detected = False
    yellow_raw_detected = False

# ======================================================================
# 单应性变换 (逆透视)
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

    left_blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=CUT_LEFT_ROI,
                                pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
    right_blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=CUT_RIGHT_ROI,
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
    # 同一颜色有多个合格色块时，先获取最左边的色块；不同颜色之间仍沿用原来的比较方式。
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
    wx_sum, wy_sum = pixel_to_world(x, y, H_pix2world)
    wx, wy = pixel_to_world(x + w, y, H_pix2world)
    wx_sum += wx
    wy_sum += wy
    wx, wy = pixel_to_world(x, y + h, H_pix2world)
    wx_sum += wx
    wy_sum += wy
    wx, wy = pixel_to_world(x + w, y + h, H_pix2world)
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

# ======================================================================
# 逆透视标定数据，当前沿用从车离地 22cm 的现场标定参数。
# 从车现为 OpenART Plus；若参数来自旧 Mini 安装或高度/俯仰角有变化，必须重新标定。
# ======================================================================
CALIB_PIXEL = [
    [90, 240],     # 点0: 近处左侧
    [228, 240],    # 点1: 近处右侧
    [115, 131],    # 点2: 远处左侧
    [202, 131],    # 点3: 远处右侧
]
CALIB_WORLD = [
    [-7.5, 7],   # 点0: 左7.5cm, 前方7.5cm
    [7.5, 7],    # 点1: 右7.5cm, 前方7.5cm
    [-7.5, 22],  # 点2: 左7.5cm, 前方22.5cm
    [7.5, 22],   # 点3: 右7.5cm, 前方22.5cm
]

# 计算单应性矩阵
H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)

# ======================================================================
# 串口通信协议 (16字节世界坐标版)
# ======================================================================
# [0-1]  帧头: 0xAA 0x55
# [2]    颜色ID (0=无目标, 1=淡蓝, 2=红色, 3=网球, 4=棕熊, 5=白熊)
# [3-4]  世界X (mm, int16, 小端序)
# [5-6]  世界Y (mm, int16, 小端序)
# [7-8]  像素宽度 (uint16, 小端序)
# [9]    黄线标志
# [10]   位置关系
# [11]   预留障碍字段，当前固定为0
# [12-14] 预留角度字段，当前固定为0
# [15]   校验和 (data[2:15]之和 & 0xFF)

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
    [12-14] 预留角度字段
    [15]   校验和 (data[2:15])
    """
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
    global lost_frame_count, openart_mode
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    global _cmd_rx_buf, front_scan_requested
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
        elif command == 0x01:  # 进入搬运模式
            reset_yellow_state()
            openart_mode = MODE_CARRY
        elif command == 0x04:  # Crossline angle correction removed on slave runtime.
            pass
        elif command == 0x05:  # Return mode is master-only; consume and ignore broadcasts.
            pass
        elif command == 0x06:  # 搬运前扫描其它颜色ID
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x00 or command == 0x02:  # 回到寻找模式/右转完成/重置
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
        return

def current_pos_flag(frame_count):
    global yellow_lost_count, yellow_seen_in_carry, openart_mode, yellow_carry_hold_count
    if openart_mode == MODE_CARRY:
        if yellow_raw_detected:
            yellow_seen_in_carry = True
            yellow_carry_hold_count = YELLOW_CARRY_HOLD_FRAMES
            yellow_lost_count = 0
            return POS_NO_BOUNDARY
        if not yellow_seen_in_carry:
            return POS_NO_BOUNDARY
        if yellow_carry_hold_count > 0:
            yellow_carry_hold_count -= 1
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
    global yellow_tracking, yellow_detected, yellow_raw_detected

    yellow_raw_detected = False
    if frame_count % YELLOW_DETECT_INTERVAL != 0:
        return

    yellow_pixels_threshold = YELLOW_KEEP_PIXELS if yellow_tracking else YELLOW_ENTER_PIXELS

    if openart_mode == MODE_CARRY:
        yellow_left = find_yellow_blob_bottom_up(img, YELLOW_ROI_LEFT, yellow_pixels_threshold)
        yellow_right = find_yellow_blob_bottom_up(img, YELLOW_ROI_RIGHT, yellow_pixels_threshold)
    else:
        yellow_left = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_LEFT,
                                     pixels_threshold=yellow_pixels_threshold,
                                     area_threshold=20, merge=True)
        yellow_right = img.find_blobs(yellow_threshold, roi=YELLOW_ROI_RIGHT,
                                      pixels_threshold=yellow_pixels_threshold,
                                      area_threshold=20, merge=True)

    raw_yellow_seen = yellow_left and yellow_right
    yellow_raw_detected = True if raw_yellow_seen else False

    # 首次进入用较高阈值，跟踪中用较低保持阈值，减少边缘闪烁造成的丢失。
    if raw_yellow_seen:
        yellow_tracking = True
        yellow_detected = True
    else:
        yellow_detected = False
        if openart_mode == MODE_SEARCH:
            yellow_tracking = False

# ======================================================================
# 主循环
# ======================================================================
frame_count = 0

# ======================================================================
# 标定模式，CALIBRATION_MODE=True 时运行
# ======================================================================
if CALIBRATION_MODE:
    _calib_pts = []
    _calib_stable = 0
    _calib_last = [0, 0]
    _calib_wait_remove = False
    _STABLE_NEED = 20
    _MOVE_THR = 10
    _calib_t = time.ticks_ms()

    print("=" * 50)
    print(">>> 前视逆透视标定模式 <<<")
    print("  [2]----[3]  远处")
    print("   |      |")
    print("  [0]----[1]  近处")
    print("=" * 50)
    print("请将标记物放在位置 0 ...")

    # 标定用阈值：只用前2个可靠的沙包颜色，避免泰迪熊等宽松阈值产生假色块
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
                        print(">>> Point {}: [{}, {}]".format(_n, _bx, _by))
                        _calib_stable = 0
                        _calib_wait_remove = True
                        if len(_calib_pts) < 4:
                            print("请拿走标记物，放到位置 {} ...".format(len(_calib_pts)))
                else:
                    _calib_stable = 0
                _calib_last = [_bx, _by]
        else:
            _calib_stable = 0
            _calib_wait_remove = False

    print("")
    print("=" * 50)
    print("标定完成! 复制以下内容到 CALIB_PIXEL:")
    print("")
    print("CALIB_PIXEL = [")
    for _ci in range(4):
        _cp = _calib_pts[_ci]
        print("    [{}, {}],     # 点{}".format(_cp[0], _cp[1], _ci))
    print("]")
    print("")

    _H_test = calc_homography(_calib_pts, CALIB_WORLD)
    if _H_test:
        print("进入验证模式: 移动标记物查看世界坐标")

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
                print("pixel=({},{}) -> world=({:.1f},{:.1f})cm".format(_bx, _by, _wx, _wy))
                _calib_t = _now

while True:
    # 帧率计算
    clock.tick()
    frame_count += 1

    # 接收主机命令
    receive_command_from_host()

    # 获取图像 + 镜头畸变校正
    img = snapshot_frame(apply_lens_corr=RUNTIME_LENS_CORR)

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    if process_front_scan_request(img):
        continue
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

    if best:
        lost_frame_count = 0
        send_color_id, x1, y1, w, h = best

        # Local detection is only a candidate report; final color lock comes from host 0x03.

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag)

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
        send_world_no_target(yellow_detected, pos_flag)
    # 黄线检测已提前更新，保证 pos_flag 使用当前帧状态。
