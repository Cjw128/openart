# ======================================================================
# OpenART Plus fixed slave-camera runtime - multi-color target detection
# ======================================================================
# Pure-LAB fallback from 7a29288 with return mode and completion-mask support.


import sensor
from machine import UART

# ======================================================================
# 模式选择
# ======================================================================

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

def snapshot_frame():
    return sensor.snapshot().replace(hmirror=True)

# 白平衡配置
WB_GAINS = (101.00, 64.00, 97.00) # 写死白平衡
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)

EXPOSURE_INIT = 1400
EXPOSURE_MIN = 100
EXPOSURE_MAX = 4500

def clamp_exposure(value):
    if value < EXPOSURE_MIN:
        return EXPOSURE_MIN
    if value > EXPOSURE_MAX:
        return EXPOSURE_MAX
    return value

def load_startup_exposure(path='/sd/color_thr.txt'):
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('exposure_us='):
                    exposure = clamp_exposure(
                        int(line.split('=', 1)[1].strip()))
                    print('[EXPOSURE] loaded %dus from %s' %
                          (exposure, path))
                    return exposure
    except Exception as error:
        print('[EXPOSURE] load failed: ' + str(error))
    print('[EXPOSURE] using fallback %dus' % EXPOSURE_INIT)
    return EXPOSURE_INIT

startup_exposure_us = load_startup_exposure()
sensor.set_auto_exposure(False, exposure_us=startup_exposure_us)
sensor.skip_frames(time=800)

# Both master and slave cameras are OpenART Plus boards and use UART12.
uart = UART(12, baudrate=115200)

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
    (51, 100, -5, 5, -38, 18)      # 颜色5: 白色泰迪熊 ← 需实际标定!
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
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('exposure_us='):
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
        ground_threshold = _average_ground_threshold(ground_rows)
        if len(rows) == 5:
            loaded_rows = []
            for slot in range(1, 6):
                loaded_rows.append(rows[slot])
            return loaded_rows, ground_threshold
        return None, ground_threshold
    except Exception:
        pass
    return None, None

_loaded, _loaded_ground_threshold = _load_calibrated_params()
if _loaded:
    all_color_thresholds = _loaded
    all_color_thresholds[4] = _separate_white_bear_from_ground(
        all_color_thresholds[4], _loaded_ground_threshold)

_color_threshold_groups = []
for threshold in all_color_thresholds:
    _color_threshold_groups.append([threshold])

COLOR_LOST_FRAMES = 2
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
COLOR_DETECT_Y_MAX = 230   # 普通目标色块忽略 y=230..239；其它检测不受影响。

# ======================================================================
# Dynamic cut line (based on multiple blue-ground strips)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = ([_loaded_ground_threshold]
                         if _loaded_ground_threshold
                         else [(25, 62, -3, 57, -96, 127)])
CUT_BLOB_MIN_H = 12  # 从上往下只接受连续高度至少 12 px 的第一段深蓝色
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
CUT_ROI_Y_OFFSET = -10      # Target ROI starts 10 px above the detected blue-ground boundary.
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2

# 目标连续性过滤
TRACK_MAX_JUMP_PX = 90
TRACK_MAX_JUMP2 = TRACK_MAX_JUMP_PX * TRACK_MAX_JUMP_PX
TRACK_AREA_CHANGE_MAX_PERCENT = 60
TRACK_MIN_IOU = 0.05
BRN_BEAR_MERGE_MARGIN = 12
BRN_BEAR_FRAGMENT_MIN_PIXELS = 20
BRN_BEAR_FRAGMENT_MIN_AREA = 20
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
completed_color_mask = 0
search_reset_cycle_active = False
pending_carry_color_id = 0
color_track_active = False
color_track_box = None
color_track_color_id = 0
color_lost_count = 0
_cmd_rx_buf = bytearray()
front_scan_requested = False
CMD_CLEAR_COMPLETED = 0x08
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
# 回库横向黄线检测参数
# ======================================================================
RETURN_YELLOW_PACKET_ID = 0xC8
RETURN_YELLOW_THRESHOLD = [(27, 100, -55, 16, 21, 105)]
RETURN_YELLOW_ROI = (150, 30, 20, 210)
RETURN_YELLOW_MIN_PIXELS = 5
RETURN_YELLOW_MIN_AREA = 5
RETURN_YELLOW_STABLE_FRAMES = 1
RETURN_YELLOW_STABLE_DELTA = 3
RETURN_STOP_Y_THRESHOLD = 200
RETURN_STATUS_Y_VALID = 0x01
RETURN_STATUS_STOP = 0x02
return_yellow_last_y = -1
return_yellow_stable_count = 0
return_yellow_detected = False
return_yellow_y = 0
return_stop_y = -1
return_stop_requested = False

MODE_SEARCH = 0
MODE_RETURN = 3
openart_mode = MODE_SEARCH

def color_id_completed(color_id):
    return (1 <= color_id <= len(all_color_thresholds) and
            bool(completed_color_mask & (1 << (color_id - 1))))

def color_id_available_for_search(color_id):
    if (color_id < 1 or color_id > len(all_color_thresholds) or
            color_id_completed(color_id)):
        return False
    if not color_id_completed(2):
        return color_id == 2
    return True

def mark_color_completed(color_id):
    global completed_color_mask
    if color_id_available_for_search(color_id):
        completed_color_mask |= 1 << (color_id - 1)

def clear_completed_carry_state():
    global completed_color_mask, pending_carry_color_id
    completed_color_mask = 0
    pending_carry_color_id = 0

def active_selected_color_id():
    if host_color_id_received and 1 <= target_color_id <= len(all_color_thresholds):
        return target_color_id
    if color_track_active and 1 <= color_track_color_id <= len(all_color_thresholds):
        return color_track_color_id
    return 0

def begin_pending_carry():
    global pending_carry_color_id
    color_id = active_selected_color_id()
    pending_carry_color_id = (
        color_id if 1 <= color_id <= len(all_color_thresholds) else 0)

def finish_pending_carry():
    global pending_carry_color_id
    if 1 <= pending_carry_color_id <= len(all_color_thresholds):
        mark_color_completed(pending_carry_color_id)
    pending_carry_color_id = 0

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

def reset_return_yellow_state():
    global return_yellow_last_y, return_yellow_stable_count
    global return_yellow_detected, return_yellow_y
    global return_stop_y, return_stop_requested
    return_yellow_last_y = -1
    return_yellow_stable_count = 0
    return_yellow_detected = False
    return_yellow_y = 0
    return_stop_y = -1
    return_stop_requested = False

# ======================================================================
# 单应性变换 (逆透视)
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
    # 取最小 y，等价于沿竖条带从上往下寻找第一段连续深蓝色。
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
    global dynamic_cut_left_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi

    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return

    top_y_sum = 0
    valid_strips = 0
    for roi in CUT_STRIP_ROIS:
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                               pixels_threshold=CUT_MIN_PIXELS,
                               area_threshold=CUT_MIN_AREA, merge=True)
        top_y = pick_top_y_from_strip(blobs)
        if top_y is not None:
            top_y_sum += top_y
            valid_strips += 1

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        top_y_average = top_y_sum // valid_strips
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = top_y_average
            dynamic_cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            dynamic_cut_left_y = int(
                a * top_y_average + (1.0 - a) * dynamic_cut_left_y)

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
    if fixed_color_id > 0 and not color_id_available_for_search(fixed_color_id):
        return None
    # Brown bear always uses its dedicated fragment-merging pass. Skipping the
    # normal fixed-color pass avoids doing the same LAB scan twice when locked.
    blobs = (None if fixed_color_id == 4
             else find_color_blobs_once(img, roi, fixed_color_id))
    scan_brown = (color_id_available_for_search(4) and
                  (fixed_color_id == 0 or fixed_color_id == 4))
    scan_white = (color_id_available_for_search(5) and
                  fixed_color_id == 0 and color_track_active
                  and color_track_color_id == 5)
    if fixed_color_id == 0 and blobs:
        for blob in blobs:
            if (color_id_available_for_search(5) and
                    blob.code() & (1 << 4)):
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
            if color_id <= 0 or not color_id_available_for_search(color_id):
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
        if not color_id_available_for_search(color_id):
            continue
        if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
            # Keep targets that straddle the boundary; reject only blobs wholly above it.
            if blob.y() + blob.h() < dynamic_cut_left_y + CUT_BLOB_DELTA:
                continue
        if not valid_color_blob(blob, color_id):
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

    # 纵向窄条从下往上扫描。
    best_y = None
    best_bottom = -1
    best_pixels = -1
    for blob in blobs:
        bottom = blob.y() + blob.h()
        y = blob.cy()
        pixels = blob.pixels()
        if (best_y is None or bottom > best_bottom or
                (bottom == best_bottom and pixels > best_pixels)):
            best_y = y
            best_bottom = bottom
            best_pixels = pixels
    return best_y

def process_return_yellow(img):
    global return_yellow_last_y, return_yellow_stable_count
    global return_yellow_detected, return_yellow_y
    global return_stop_y, return_stop_requested

    y = detect_return_yellow_y(img)
    return_stop_y = y if y is not None else -1
    return_stop_requested = (
        y is not None and y > RETURN_STOP_Y_THRESHOLD)

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
# 由 d040b74 的四点比赛标定参数预计算。
H_PIX2WORLD = (
    0.5835117773019284, -0.0026766595289079054, -92.13597430406873,
    9.349246523159213e-17, -0.33832976445396207, 118.77730192719507,
    5.8936449814456205e-18, 0.01820128479657392,
)

def box_to_world(x, y, w, h):
    # 单应性只描述地面点，目标底边中点才是接地点。四角平均会让上边缘越过
    # 投影地平线，使远处的正距离突然发散并翻成负数。
    px = x + w * 0.5
    py = y + h
    den = H_PIX2WORLD[6] * px + H_PIX2WORLD[7] * py + 1.0
    if -1e-10 < den < 1e-10:
        return (0.0, WORLD_Y_MAX_CM)
    wx = (H_PIX2WORLD[0] * px + H_PIX2WORLD[1] * py + H_PIX2WORLD[2]) / den
    wy = (H_PIX2WORLD[3] * px + H_PIX2WORLD[4] * py + H_PIX2WORLD[5]) / den
    # 该写法也能覆盖地平线处产生的零值、NaN 和无穷值。
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
# 串口通信协议 (16字节世界坐标版)
# ======================================================================
# [0-1]  帧头: 0xAA 0x55
# [2]    颜色ID (0=无目标, 1=淡蓝, 2=红色, 3=网球, 4=棕熊, 5=白熊)
# [3-4]  世界X (mm, int16, 小端序)
# [5-6]  世界Y (mm, int16, 小端序)
# [7-8]  像素宽度 (uint16, 小端序)
# [9-10] 主摄像头专用的黄线/位置字段，从机固定为0
# [11]   预留障碍字段，当前固定为0
# [12-14] 预留角度字段，当前固定为0
# [15]   校验和 (data[2:15]之和 & 0xFF)

_tx_world_buf = bytearray(16)
_tx_world_no_target_buf = bytearray(16)
_tx_world_buf[0] = _tx_world_no_target_buf[0] = 0xAA
_tx_world_buf[1] = _tx_world_no_target_buf[1] = 0x55

def send_world_data(color_id, wx_mm, wy_mm, pw):
    """发送 16 字节世界坐标数据包。
    [0-1]  帧头 0xAA 0x55
    [2]    颜色ID
    [3-4]  世界X (mm, int16, 小端序)
    [5-6]  世界Y (mm, int16, 小端序)
    [7-8]  像素宽度 (uint16)
    [9-10] 主摄像头专用字段，从机固定为0
    [11]   预留障碍字段
    [12-14] 预留角度字段
    [15]   校验和 (data[2:15])
    """
    # 编码前饱和；若直接掩码，超出 int16 的正数会被主控解码成负数。
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
    data[15] = (data[2] + data[3] + data[4] + data[5] + data[6] +
                data[7] + data[8]) & 0xFF
    uart.write(data)

def send_world_no_target():
    # Same 16-byte packet layout as send_world_data(), with color_id/position fields zero.
    """发送无目标的 16 字节世界坐标数据包。"""
    uart.write(_tx_world_no_target_buf)

def receive_command_from_host():
    """接收RT1021主机命令"""
    global lost_frame_count, openart_mode
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    global _cmd_rx_buf, front_scan_requested
    global search_reset_cycle_active

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
                if not color_id_available_for_search(param):
                    reset_target_tracking_state()
                else:
                    same_target = host_color_id_received and target_color_id == param
                    target_color_id = param
                    host_color_id_received = True
                    lost_frame_count = 0
                    if not same_target:
                        color_track_active = False
                        color_track_box = None
                        color_track_color_id = 0
                        color_lost_count = 0
        elif command == 0x01:  # 搬运模式由主摄像头处理，从机退出回库视觉。
            search_reset_cycle_active = False
            begin_pending_carry()
            openart_mode = MODE_SEARCH
            reset_return_yellow_state()
        elif command == 0x04:  # Crossline angle correction removed on slave runtime.
            pass
        elif command == 0x05:  # 旧回库信标命令已停用，仅消费该命令。
            pass
        elif command == CMD_CLEAR_COMPLETED:
            search_reset_cycle_active = False
            clear_completed_carry_state()
            openart_mode = MODE_SEARCH
            front_scan_requested = False
            reset_front_scan_state()
            reset_target_tracking_state()
            reset_return_yellow_state()
        elif command == 0x06:  # 搬运前扫描其它颜色ID
            openart_mode = MODE_SEARCH
            reset_return_yellow_state()
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x07:  # 进入回库横向黄线跟踪。
            search_reset_cycle_active = False
            openart_mode = MODE_RETURN
            front_scan_requested = False
            reset_front_scan_state()
            reset_return_yellow_state()
        elif command == 0x02:  # 每个搜索就绪周期只重置一次。
            finish_pending_carry()
            if not search_reset_cycle_active:
                search_reset_cycle_active = True
                openart_mode = MODE_SEARCH
                reset_target_tracking_state()
                reset_return_yellow_state()
        elif command == 0x00:  # 回到寻找模式/重置。
            finish_pending_carry()
            search_reset_cycle_active = False
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_return_yellow_state()
        return

# ======================================================================
# 主循环
# ======================================================================
frame_count = 0

while True:
    frame_count += 1

    # 接收主机命令
    receive_command_from_host()

    img = snapshot_frame()

    # ===== 回库横向黄线 =====
    if openart_mode == MODE_RETURN:
        process_return_yellow(img)
        continue

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    if process_front_scan_request(img):
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

    elif (color_track_active and color_track_box and
          color_id_available_for_search(color_track_color_id)):
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
        send_world_data(send_color_id, wx_mm, wy_mm, w)
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
        send_world_no_target()
