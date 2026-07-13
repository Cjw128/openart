# ======================================================================
# OpenART Plus fixed slave-camera runtime - multi-color target detection
# ======================================================================


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
sensor.skip_frames(time=500)

# 固定曝光
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)

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
TENNIS_MIN_PIXELS = 45
TENNIS_MIN_AREA = 45
BEAR_MIN_BOX_AREA = 480
_color_blob_limits = (
    (COLOR_ID12_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_ID12_MIN_PIXELS, COLOR_MIN_AREA),
    (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
)
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

# ======================================================================
# Dynamic cut line (based on blue-ground strips on left/right)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = [(0, 55, -30, 45, -90, -7)]
CUT_BLOB_MIN_H = 12
CUT_BLOB_BOTTOM_MARGIN = 25
CUT_GAP_BRIDGE = 10
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
_cmd_rx_buf = bytearray()
front_scan_requested = False
FRONT_SCAN_PACKET_ID = 0xC7
FRONT_SCAN_EXCLUDE_IOU = 0.20
FRONT_SCAN_EXCLUDE_CENTER_PX = 35
FRONT_SCAN_EXCLUDE_CENTER2 = FRONT_SCAN_EXCLUDE_CENTER_PX * FRONT_SCAN_EXCLUDE_CENTER_PX
FRONT_SCAN_Y_MAX = 150
FRONT_SCAN_MIN_PIXELS = 150
FRONT_SCAN_STABLE_FRAMES = 10
FRONT_SCAN_MAX_FRAMES = 30
front_scan_last_current_id = 0
front_scan_last_mask = -1
front_scan_last_count = 0
front_scan_stable_count = 0
front_scan_total_count = 0

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

    top_y_sum = 0
    valid_strips = 0
    for roi in CUT_STRIP_ROIS:
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                               pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
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
            dynamic_cut_left_y = int(a * top_y_average + (1.0 - a) * dynamic_cut_left_y)

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

def valid_color_blob(blob, color_id):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    if color_id == 3:
        if w * 100 < h * 45 or w * 100 > h * 185:
            return False
        if blob.density() < 0.35:
            return False
    elif color_id == 4 or color_id == 5:
        if w * 100 < h * 30 or w * 100 > h * 250:
            return False
        if w * h <= BEAR_MIN_BOX_AREA:
            return False
        if blob.pixels() < 120:
            return False
    else:
        if w * 100 < h * 60 or w * 100 > h * 180:
            return False
        if blob.density() < 0.40:
            return False
    return True

def find_color_target(img, last_box):
    color_ids = (_single_color_ids[target_color_id - 1]
                 if target_color_id > 0 else COLOR_SEARCH_ORDER)
    roi = make_roi_from_box(last_box)
    tracking = last_box is not None
    best_blob = None
    best_color_id = 0
    best_score = None
    best_distance = None
    for color_id in color_ids:
        if tracking and color_track_color_id > 0 and color_id != color_track_color_id:
            continue
        pixels_threshold, area_threshold = _color_blob_limits[color_id - 1]
        try:
            blobs = img.find_blobs(_color_threshold_groups[color_id - 1], roi=roi,
                                   pixels_threshold=pixels_threshold,
                                   area_threshold=area_threshold,
                                   merge=True)
        except Exception:
            blobs = None
        if not blobs:
            continue
        representative = None
        for blob in blobs:
            if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
                if blob.cy() < dynamic_cut_left_y + CUT_BLOB_DELTA:
                    continue
            if not valid_color_blob(blob, color_id):
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
            elif (representative is None or
                  (blob.cx(), blob.x(), -blob.pixels()) <
                  (representative.cx(), representative.x(), -representative.pixels())):
                representative = blob
        if not tracking and representative is not None:
            distance = 240 - (representative.y() + representative.h())
            if best_blob is None or distance < best_distance:
                best_blob = representative
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
    for color_id in COLOR_SEARCH_ORDER:
        if color_id < 1 or color_id > len(all_color_thresholds):
            continue
        pixels_threshold, area_threshold = _color_blob_limits[color_id - 1]
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
                if blob.cy() < dynamic_cut_left_y + CUT_BLOB_DELTA:
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
    global lost_frame_count
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
        elif command == 0x01:  # 搬运模式由主摄像头处理，从机只消费命令。
            pass
        elif command == 0x04:  # Crossline angle correction removed on slave runtime.
            pass
        elif command == 0x05:  # Return mode is master-only; consume and ignore broadcasts.
            pass
        elif command == 0x06:  # 搬运前扫描其它颜色ID
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x00 or command == 0x02:  # 回到寻找模式/右转完成/重置
            reset_target_tracking_state()
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

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    if process_front_scan_request(img):
        continue

    # ===== Color blob detection / tracking =====
    has_target = False
    send_color_id = 0
    last_box = color_track_box if color_track_active else None
    found = find_color_target(img, last_box)

    if found:
        send_color_id, blob = found
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
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
