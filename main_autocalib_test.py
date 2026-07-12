# ======================================================================
# [TEST] 纯色块方案 + 现场自动标定 — 前视 OpenART 测试代码
# ======================================================================
# 架构:
#   平时: 纯 find_blobs 色块检测(高帧率), 完全不跑模型
#   标定: 收到主控 0x04 命令时, 加载的模型只用来"框物体",
#         对框内取样自动生成 LAB 阈值 → 覆盖写 /sd/color_thr.txt
#
# 阈值修改机制(不改 .py 源码):
#   1) 内存覆盖: all_color_thresholds 是内存 list, find_blobs 每帧读它,
#      multi 标定成功后直接用本次结果覆盖, 下一帧生效
#   2) 持久化: 同时写 /sd/color_thr.txt, 开机 load_calibrated_params()
#      读文件覆盖默认值; 删掉该文件即回退代码默认值
#
# 串口协议与生产 main.py 一致:
#   上行 12 字节世界坐标帧: AA 55 id x x y y w w yellow pos ck
#   下行命令: 0x01 搬运 / 0x02 重置 / 0x03 锁色 / 0x04 自动标定(新增)
#   标定回执 8 字节: AA 55 C4 status count exp_lo exp_hi ck
# ======================================================================

import sensor, image, time, tf
from machine import UART

IS_SLAVE_CAR = False

# ---- 摄像头 ----
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)

WB_GAINS = (101.00, 64.00, 97.00)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_exposure(False, exposure_us=1200)   # 默认曝光, 标定文件可覆盖
sensor.skip_frames(time=500)

# ---- 颜色阈值(默认值, 标定文件会覆盖) ----
# 槽位: 1=淡蓝沙包 2=红沙包 3=网球 4=棕熊 5=白熊
all_color_thresholds = [
    (23, 96, -49, 4, -53, -30),
    (10, 80, 22, 122, -17, 93),
    (50, 100, -128, -27, 20, 127),
    (20, 55, -1, 30, 0, 50),
    (53, 100, -10, 11, -11, 8),
]
DEFAULT_THRESHOLDS = list(all_color_thresholds)   # 0x05 清空标定时恢复用
DEFAULT_EXPOSURE_US = 1200
SLOT_NAMES = {1: "blue_bag", 2: "red_bag", 3: "ball", 4: "brn_bear", 5: "wht_bear"}
LABEL_NAMES = {0: "bear", 1: "ball", 2: "bag"}
DRAW_COLORS = {1: (0, 170, 255), 2: (255, 0, 0), 3: (0, 255, 0),
               4: (255, 180, 0), 5: (255, 255, 255)}

CALIB_FILE = '/sd/color_thr.txt'
calibrated_slots = set()   # 标定过的槽位; 非空时主循环只检测这些槽位
                           # (没标定过的默认阈值在标定后的曝光下不可信,
                           #  例: 白熊默认阈值=亮中性色, 暗光拉高曝光后会吃掉整个背景)
ground_box = None          # 地面(赛道布)色度范围盒 (L0,L1,A0,A1,B0,B1), 标定采集并存文件。
                           # 用范围而非单点: 亮光下布面反光使 L 空间分布很宽,
                           # 单点±容差会漏掉远处发亮的布面
calib_file_exp = None      # 标定文件里的曝光值; 0x04 重新 multi 标定时只作为旧文件信息读取,
                           # 成功后由本次结果整体覆盖, 不再融合旧阈值
ground_box_far = None      # 远处地面盒(掠射角下布色度漂移大, 近处盒罩不住)
cut_left_y = 0             # 动态分界线(生产 update_dynamic_cut 同构): 左右竖带各自的布最高点,
cut_right_y = 0            # 检测 ROI 从分界线以下开始, 赛道外整体切掉(不影响地面取色块)
cut_valid = False
cut_miss = 0

# 标定日志: 0x04 开始后实时追加到 /sd/calib_log.txt。
# 这样即使标定卡住/断电, 也能拔 OpenART SD 卡看最后卡在哪个槽位/原因。
CALIB_LOG_FILE = '/sd/calib_log.txt'
AC_LOG = []
AC_LOG_LIVE = False

def _clog(*args):
    s = " ".join(str(a) for a in args)
    print(s)
    AC_LOG.append(s)
    if AC_LOG_LIVE:
        try:
            with open(CALIB_LOG_FILE, 'a') as fp:
                fp.write(s + "\n")
        except Exception:
            pass

def _start_calib_log():
    global AC_LOG_LIVE
    del AC_LOG[:]
    AC_LOG_LIVE = False
    try:
        with open(CALIB_LOG_FILE, 'w') as fp:
            fp.write("")
    except Exception as e:
        print("!! log init fail:", e)
    AC_LOG_LIVE = True

def load_calibrated_params():
    global calibrated_slots, ground_box, ground_box_far
    exposure = None
    try:
        with open(CALIB_FILE) as fp:
            for line in fp:
                line = line.strip()
                if not line:
                    continue
                if line.startswith('exposure_us='):
                    exposure = int(line.split('=')[1])
                    global calib_file_exp
                    calib_file_exp = exposure
                    continue
                if line.startswith('ground2='):
                    g = line.split('=')[1].split(',')
                    if len(g) == 6:
                        ground_box_far = tuple(int(v) for v in g)
                        _clog("[calib] ground_far <- {}".format(ground_box_far))
                    continue
                if line.startswith('ground='):
                    g = line.split('=')[1].split(',')
                    if len(g) == 6:
                        ground_box = tuple(int(v) for v in g)
                        _clog("[calib] ground <- {}".format(ground_box))
                    continue
                parts = line.split(',')
                if len(parts) != 7:
                    continue
                slot = int(parts[0])
                if 1 <= slot <= len(all_color_thresholds):
                    all_color_thresholds[slot - 1] = tuple(int(v) for v in parts[1:])
                    calibrated_slots.add(slot)
                    _clog("[calib] slot {} <- {}".format(slot, all_color_thresholds[slot - 1]))
    except Exception:
        return None
    return exposure

_exp = load_calibrated_params()
if _exp:
    sensor.set_auto_exposure(False, exposure_us=_exp)
    _clog("[calib] exposure <- {}us".format(_exp))

# ---- 模型(只在 0x04 标定时使用) ----
# 加载失败不能挡住纯色块主循环: net=None 时 0x04 标定直接回 FAIL, 色块照常跑
net = None
try:
    net = tf.load('/sd/dataset_25000_exposure.tflite')
    print("[model] loaded")
except Exception as e:
    print("[model] !! load failed, calib disabled:", e)
# 模型标签: 0=bear 1=ball 2=bag

# ---- 串口 ----
if IS_SLAVE_CAR:
    uart = UART(2, baudrate=115200)
else:
    uart = UART(12, baudrate=115200)
uart_extra = None
if not IS_SLAVE_CAR:
    try:
        uart_extra = UART(2, baudrate=115200)
    except Exception:
        uart_extra = None

def uart_write(data):
    uart.write(data)
    if uart_extra is not None:
        try:
            uart_extra.write(data)
        except Exception:
            pass

def uart_read_all():
    data = bytearray()
    if uart.any():
        chunk = uart.read(uart.any())
        if chunk:
            data.extend(chunk)
    if uart_extra is not None:
        try:
            if uart_extra.any():
                chunk = uart_extra.read(uart_extra.any())
                if chunk:
                    data.extend(chunk)
        except Exception:
            pass
    return data

# ---- 单应性(世界坐标, 与生产一致) ----
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
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]]

CALIB_PIXEL = [[60, 220], [260, 220], [100, 140], [220, 140]]
CALIB_WORLD = [[-7.5, 7.5], [7.5, 7.5], [-7.5, 22.5], [7.5, 22.5]]
H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)

def pixel_to_world(px, py):
    H = H_pix2world
    w = H[2][0] * px + H[2][1] * py + H[2][2]
    if abs(w) < 1e-10:
        return (0.0, 0.0)
    return ((H[0][0] * px + H[0][1] * py + H[0][2]) / w,
            (H[1][0] * px + H[1][1] * py + H[1][2]) / w)

def box_to_world(x, y, w, h):
    wx, wy = pixel_to_world(x + w / 2.0, y + h)  # 底边中点=接地点
    # 双向截断: 单应性在标定区外外推时会发散, 主控对 |wx|>500 或 wy 越界的帧
    # 整帧丢弃且不刷新时间戳, 会导致 Monitor 恒显示 Cam:OLD
    if wy > 250.0:
        wy = 250.0
    elif wy < 0.0:
        wy = 0.0
    if wx > 250.0:
        wx = 250.0
    elif wx < -250.0:
        wx = -250.0
    return (wx, wy)

def clamp_int(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

# ---- 色块检测参数 ----
DETECT_Y_MIN = 20
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)
COLOR_MIN_PIXELS = 20
COLOR_MIN_AREA = 20

# ---- 形状过滤(移植生产 valid_color_blob, 按类别区分) ----
# 贴边截断的 blob 长宽比已失真, 走宽松规则(只查像素数+密度)
EDGE_TOUCH_MARGIN_PX = 3
EDGE_CROPPED_MIN_PIXELS = 150
EDGE_CROPPED_MIN_DENSITY = 0.35
SMALL_BLOB_SHAPE_SKIP_W = 20
BEAR_BLOB_SHAPE_SKIP_W = 28
BRN_BEAR_BLOB_SHAPE_SKIP_W = 90
BAG_SMALL_MIN_W = 8
BAG_SMALL_MIN_H = 8
BALL_SMALL_MIN_W = 8
BALL_SMALL_MIN_H = 8
BEAR_SMALL_MIN_H = 8
BRN_BEAR_SMALL_MIN_H = 5
SMALL_BLOB_RELAX_W = 25
OFF_BOX_MARGIN_NEAR = 3
OFF_BOX_MARGIN_SMALL = 8
GND_BOX_SHRINK_SMALL = 4
BLUE_BAG_GND_BOX_SHRINK = 12
BLUE_BAG_SIDE_GUTTER_Y_MIN = 185
BLUE_BAG_SIDE_GUTTER_X_MARGIN = 35
BLUE_BAG_BOTTOM_SHADOW_Y_MIN = 150
BLUE_BAG_BOTTOM_SHADOW_MAX_W = 28
BLUE_BAG_BOTTOM_SHADOW_MAX_H = 34
BLUE_BAG_CORE_A_HI_SHRINK = 2
BLUE_BAG_CORE_B_HI_SHRINK = 2
BLUE_BAG_NEAR_CORE_MIN_H = 90
BLUE_BAG_NEAR_CORE_MIN_BOTTOM_Y = 220
BLUE_BAG_NEAR_CORE_BOTTOM_Y = 238
BLUE_BAG_NEAR_CORE_A_HI_SHRINK = 10
BLUE_BAG_NEAR_CORE_B_HI_SHRINK = 10
BLUE_BAG_CORE_MIN_PIXELS = 20
BLUE_BAG_CORE_FALLBACK_MAX_W = 110
BLUE_BAG_CORE_FALLBACK_MAX_H = 90
BLUE_BAG_CORE_ACCEPT_W_NUM = 7
BLUE_BAG_CORE_ACCEPT_H_NUM = 6
BLUE_BAG_CORE_ACCEPT_DEN = 10
BLUE_BAG_CORNER_MERGE_MIN_H = 70
BRN_BEAR_L_PAD_HI = 4
BRN_BEAR_MERGE_MARGIN = 12
BRN_BEAR_STABLE_FRAMES = 3
BRN_BEAR_STABLE_CENTER_PX = 12
BRN_BEAR_STABLE_SIZE_PX = 12
RED_BAG_STABLE_FRAMES = 3
RED_BAG_STABLE_CENTER_PX = 24
RED_BAG_STABLE_SIZE_PX = 16
RED_BAG_TRACK_CENTER_PX = 42
RED_BAG_TRACK_SIZE_PX = 24
WHT_BEAR_L_MIN = 58
WHT_BEAR_A_ABS_MAX = 18
WHT_BEAR_B_MAX = 32

def blue_bag_side_gutter_blob(b, cid):
    return (cid == 1 and b.y() >= BLUE_BAG_SIDE_GUTTER_Y_MIN
            and (b.x() < BLUE_BAG_SIDE_GUTTER_X_MARGIN
                 or b.x() + b.w() > 320 - BLUE_BAG_SIDE_GUTTER_X_MARGIN))

def blue_bag_bottom_shadow_blob(b, cid):
    return (cid == 1 and b.y() >= BLUE_BAG_BOTTOM_SHADOW_Y_MIN
            and b.w() <= BLUE_BAG_BOTTOM_SHADOW_MAX_W
            and b.h() <= BLUE_BAG_BOTTOM_SHADOW_MAX_H)

def blue_bag_near_merge_blob(b):
    by2 = b.y() + b.h()
    return (by2 >= BLUE_BAG_NEAR_CORE_BOTTOM_Y
            or (b.h() >= BLUE_BAG_NEAR_CORE_MIN_H
                and by2 >= BLUE_BAG_NEAR_CORE_MIN_BOTTOM_Y))

def blue_bag_core_big_enough(core, coarse):
    return (core.w() * BLUE_BAG_CORE_ACCEPT_DEN >= coarse.w() * BLUE_BAG_CORE_ACCEPT_W_NUM
            and core.h() * BLUE_BAG_CORE_ACCEPT_DEN >= coarse.h() * BLUE_BAG_CORE_ACCEPT_H_NUM)

def brn_bear_box_stable(prev, box):
    if prev is None:
        return False
    pcx = prev[0] + prev[2] // 2
    pcy = prev[1] + prev[3] // 2
    cx = box[0] + box[2] // 2
    cy = box[1] + box[3] // 2
    return (abs(cx - pcx) <= BRN_BEAR_STABLE_CENTER_PX
            and abs(cy - pcy) <= BRN_BEAR_STABLE_CENTER_PX
            and abs(box[2] - prev[2]) <= BRN_BEAR_STABLE_SIZE_PX
            and abs(box[3] - prev[3]) <= BRN_BEAR_STABLE_SIZE_PX)

def red_bag_box_stable(prev, box, center_px=RED_BAG_STABLE_CENTER_PX,
                       size_px=RED_BAG_STABLE_SIZE_PX):
    if prev is None:
        return False
    pcx = prev[0] + prev[2] // 2
    pcy = prev[1] + prev[3] // 2
    cx = box[0] + box[2] // 2
    cy = box[1] + box[3] // 2
    return (abs(cx - pcx) <= center_px
            and abs(cy - pcy) <= center_px
            and abs(box[2] - prev[2]) <= size_px
            and abs(box[3] - prev[3]) <= size_px)

def slot_to_label(cid):
    # 0=bear 1=ball 2=bag (与模型标签一致)
    if cid in (4, 5):
        return 0
    if cid == 3:
        return 1
    return 2

def bear_small_min_h(cid):
    return BRN_BEAR_SMALL_MIN_H if cid == 4 else BEAR_SMALL_MIN_H

def bear_shape_skip_w(cid):
    return BRN_BEAR_BLOB_SHAPE_SKIP_W if cid == 4 else BEAR_BLOB_SHAPE_SKIP_W

def valid_color_blob(blob, cid):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    if blue_bag_side_gutter_blob(blob, cid):
        return False
    if blue_bag_bottom_shadow_blob(blob, cid):
        return False
    # 接近全画面宽的 blob 只可能是赛道布/背景, 物体不可能这么宽
    if w >= 280:
        return False
    # 贴边截断: 不查长宽比, 用更高的像素/密度门槛滤噪声
    if (blob.y() + h >= 240 - EDGE_TOUCH_MARGIN_PX
            or blob.x() + w >= 320 - EDGE_TOUCH_MARGIN_PX
            or blob.y() <= EDGE_TOUCH_MARGIN_PX
            or blob.x() <= EDGE_TOUCH_MARGIN_PX):
        return (blob.pixels() >= EDGE_CROPPED_MIN_PIXELS
                and blob.density() >= EDGE_CROPPED_MIN_DENSITY)
    label = slot_to_label(cid)
    if label == 2 and (w < BAG_SMALL_MIN_W or h < BAG_SMALL_MIN_H):
        return False
    if label == 1 and (w < BALL_SMALL_MIN_W or h < BALL_SMALL_MIN_H):
        return False
    if label == 0 and h < bear_small_min_h(cid):
        return False
    if label == 0 and w < bear_shape_skip_w(cid):
        return True
    if w < SMALL_BLOB_SHAPE_SKIP_W:
        return True
    aspect = w / h
    if label == 1:      # 网球: 接近圆形, 密度高
        if aspect < 0.45 or aspect > 1.85:
            return False
        if blob.density() < 0.35:
            return False
    elif label == 0:    # 熊: 形状多变, 放宽比例但要求像素多
        if aspect < 0.30 or aspect > 2.50:
            return False
        if blob.pixels() < 120:
            return False
    else:               # 沙包: 近方形。暗光下 blob 边缘破碎, 密度门槛不能太高
        if aspect < 0.55 or aspect > 1.90:
            return False
        if blob.density() < 0.45:
            return False
    return True

# ---- 动态分界线(移植 xuezhang/main.py 的 update_dynamic_cut, 阈值用标定近地面盒) ----
# 水平裁切线: 5 条竖带找"从底部向上连续的布"的最高点; 贴底种子段+单次桥接防噪点/黄线;
# 孤证不立+限步长对称 EMA 防单带误检抬线。检测 ROI 从线以下开始(赛道外整体切掉)。
CUT_BLOB_MIN_H = 12          # 带内布块最小高度, 滤零星噪点
CUT_BLOB_BOTTOM_MARGIN = 25  # 布块底部须贴近扫描区下沿(只认从底部连续的地面)
CUT_GAP_BRIDGE = 20          # 单次桥接最大间隙(黄线横穿会把布切成两段)
CUT_STRIP_XS = (10, 85, 160, 235, 310)
CUT_MIN_VALID_STRIPS = 2     # 至少几条带看到布才认为裁切线有效
CUT_STRIP_HALF_W = 2
CUT_SCAN_Y_MIN = 0
CUT_SCAN_Y_MAX = 140
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_Y_MARGIN = 14      # 6->14: 分界线太紧, 检测ROI向线上方多放一段
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = -6    # 2->-6: 允许blob中心高出线6px(贴线物体上半部不被误裁)
CUT_BLOB_DELTA_SMALL = -12
CUT_TINY_BLOB_MAX_W = 8
CUT_TINY_BLOB_MAX_H = 8
CUT_TINY_BLOB_MIN_BELOW = 2
CUT_OUTSIDE_SHORT_MAX_H = 12
CUT_OUTSIDE_BOTTOM_MARGIN = 2

def _gnd_thr_for_blobs():
    if ground_box is None:
        return None
    return (clamp_int(int(ground_box[0]), 0, 100), clamp_int(int(ground_box[1]), 0, 100),
            clamp_int(int(ground_box[2]), -128, 127), clamp_int(int(ground_box[3]), -128, 127),
            clamp_int(int(ground_box[4]), -128, 127), clamp_int(int(ground_box[5]), -128, 127))

def cut_line_y_at_x(x):
    # 水平裁切线: 全画面统一
    return cut_left_y

def tiny_blob_outside_cut(b):
    if not cut_valid:
        return False
    line_y = cut_line_y_at_x(b.cx())
    if b.y() < line_y and b.h() <= CUT_OUTSIDE_SHORT_MAX_H:
        return (b.y() + b.h()) <= line_y + CUT_OUTSIDE_BOTTOM_MARGIN
    if b.w() > CUT_TINY_BLOB_MAX_W or b.h() > CUT_TINY_BLOB_MAX_H:
        return False
    return b.y() < line_y and (b.y() + b.h()) <= line_y + CUT_TINY_BLOB_MIN_BELOW

def cut_delta_for_blob(b):
    return CUT_BLOB_DELTA_SMALL if b.w() < SMALL_BLOB_RELAX_W else CUT_BLOB_DELTA

def off_margin_for_blob(b):
    return OFF_BOX_MARGIN_SMALL if b.w() < SMALL_BLOB_RELAX_W else OFF_BOX_MARGIN_NEAR

def ground_margin_for_blob(b):
    return -GND_BOX_SHRINK_SMALL if b.w() < SMALL_BLOB_RELAX_W else 0

def ground_margin_for_slot(b, cid):
    if cid == 1:
        return -BLUE_BAG_GND_BOX_SHRINK
    return ground_margin_for_blob(b)

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
    # 只桥接一次, 且上方接续段本身也要够高
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

def update_cut_line(img):
    # 与 xuezhang/另一个/main.py 的 update_dynamic_cut 同构:
    # 5 带取最高点做水平线, >=2 带有效, 双向对称 EMA
    global cut_left_y, cut_right_y, cut_valid, cut_miss
    thr = _gnd_thr_for_blobs()
    if thr is None:
        cut_valid = False
        return
    strip_h = CUT_SCAN_Y_MAX - CUT_SCAN_Y_MIN
    top_y_min = None
    valid_strips = 0
    for sx in CUT_STRIP_XS:
        roi = (sx - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN, CUT_STRIP_HALF_W * 2 + 1, strip_h)
        try:
            blobs = img.find_blobs([thr], roi=roi, pixels_threshold=CUT_MIN_PIXELS,
                                   area_threshold=CUT_MIN_AREA, merge=True)
        except Exception:
            blobs = None
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            valid_strips += 1
            if top_y_min is None or ty < top_y_min:
                top_y_min = ty

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        cut_miss = 0
        if not cut_valid:
            cut_left_y = top_y_min
            cut_valid = True
        else:
            # 双向对称 EMA: 单帧噪声不会棘轮式顶高
            a = CUT_EMA_ALPHA
            cut_left_y = int(a * top_y_min + (1.0 - a) * cut_left_y)
        cut_left_y = clamp_int(cut_left_y, DETECT_Y_MIN, CUT_SCAN_Y_MAX)
        cut_right_y = cut_left_y
    else:
        cut_miss += 1
        if cut_miss > CUT_MAX_MISS:
            cut_valid = False
            cut_left_y = DETECT_Y_MIN
            cut_right_y = DETECT_Y_MIN

def dynamic_detect_roi():
    """检测 ROI 底线抬到裁切线下方, 赛道外整体不参与检测。"""
    if cut_valid:
        y_base = clamp_int(cut_left_y - CUT_Y_MARGIN, DETECT_Y_MIN, 220)
    else:
        y_base = DETECT_Y_MIN
    return (0, y_base, 320, 240 - y_base)


def draw_cut_line_debug(img):
    if not cut_valid:
        return
    try:
        img.draw_line(0, cut_left_y, 319, cut_left_y, color=(255, 0, 0))
    except Exception:
        pass

# ---- 运行时地面复核 + 槽位色度严检 ----
# 阈值/形状都可能放过赛道布, 最后防线(每个候选 blob 实测中心色度):
#   1) 中心色度落在地面范围盒内 → 是赛道布, 拒绝
#   2) 中心色度必须落在本槽阈值盒内 → 杀掉"边缘像素凑数"的碎片 blob
#      (布的褶皱碎片能靠尾部像素通过 find_blobs, 但它的中位数不在沙袋盒里)

def _blob_center_stats(img, b):
    w, h = b.w(), b.h()
    side = max(4, int(min(w, h) * 0.5))
    rx = clamp_int(b.cx() - side // 2, 0, 319)
    ry = clamp_int(b.cy() - side // 2, 0, 239)
    st = img.get_statistics(roi=(rx, ry, min(side, 320 - rx), min(side, 240 - ry)))
    return (st.l_median(), st.a_median(), st.b_median())

def _in_box(lab, box, margin=0):
    return (box[0] - margin <= lab[0] <= box[1] + margin
            and box[2] - margin <= lab[1] <= box[3] + margin
            and box[4] - margin <= lab[2] <= box[5] + margin)

# ---- 尺寸-距离一致性(生产 bear_box_plausible 思路推广到全类别) ----
# 颜色维度对"远处布面反光"已不可靠(掠射角把深蓝照成浅蓝, 色度真会进沙袋盒),
# 但几何骗不了: blob 底边反推距离, 该距离下物体像素宽是确定的,
# 布的反光斑块会超出期望宽度数倍。
TARGET_REAL_W_MM = {1: 70.0, 2: 70.0, 3: 67.0, 4: 120.0, 5: 120.0}
def blue_bag_core_threshold(thr, tight=False):
    a_shrink = BLUE_BAG_NEAR_CORE_A_HI_SHRINK if tight else BLUE_BAG_CORE_A_HI_SHRINK
    b_shrink = BLUE_BAG_NEAR_CORE_B_HI_SHRINK if tight else BLUE_BAG_CORE_B_HI_SHRINK
    a1 = max(thr[2], thr[3] - a_shrink)
    b1 = max(thr[4], thr[5] - b_shrink)
    return (thr[0], thr[1], thr[2], a1, thr[4], b1)

def blue_bag_corner_merge_blob(b):
    return (b.h() >= BLUE_BAG_CORNER_MERGE_MIN_H
            and b.y() + b.h() >= 238
            and (b.x() <= 2 or b.x() + b.w() >= 318))

def refine_blue_bag_blob(img, b, thr):
    force_core = blue_bag_near_merge_blob(b)
    core_thr = blue_bag_core_threshold(thr, force_core)
    try:
        cores = img.find_blobs([core_thr], roi=b.rect(),
                               pixels_threshold=BLUE_BAG_CORE_MIN_PIXELS,
                               area_threshold=BLUE_BAG_CORE_MIN_PIXELS,
                               merge=True)
    except Exception:
        cores = None
    best_core = None
    if cores:
        for cb in cores:
            if cb.w() < BAG_SMALL_MIN_W or cb.h() < BAG_SMALL_MIN_H:
                continue
            if (blue_bag_side_gutter_blob(cb, 1)
                    or blue_bag_bottom_shadow_blob(cb, 1)
                    or tiny_blob_outside_cut(cb)):
                continue
            if cut_valid and cb.cy() < cut_line_y_at_x(cb.cx()) + cut_delta_for_blob(cb):
                continue
            if blue_bag_corner_merge_blob(cb):
                continue
            lab = _blob_center_stats(img, cb)
            if not _in_box(lab, core_thr, off_margin_for_blob(cb)):
                continue
            if best_core is None or cb.pixels() > best_core.pixels():
                best_core = cb
    if force_core and best_core is not None:
        return best_core
    if force_core:
        return None
    if best_core is not None and blue_bag_core_big_enough(best_core, b):
        return best_core
    if blue_bag_bottom_shadow_blob(b, 1):
        return None
    if blue_bag_corner_merge_blob(b):
        return None
    if b.w() > BLUE_BAG_CORE_FALLBACK_MAX_W or b.h() > BLUE_BAG_CORE_FALLBACK_MAX_H:
        return None
    return b

def find_slot_blobs(img, thr, roi, pixels_threshold, area_threshold, cid):
    if cid == 4:
        try:
            return img.find_blobs([thr], roi=roi, pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True,
                                  margin=BRN_BEAR_MERGE_MARGIN)
        except TypeError:
            pass
    return img.find_blobs([thr], roi=roi, pixels_threshold=pixels_threshold,
                          area_threshold=area_threshold, merge=True)

FOCAL_LENGTH = 167.5
SIZE_RATIO_MAX = 2.2      # 实宽/期望宽 上限(留裕量: 单应性误差+blob溢出)
SIZE_RATIO_MIN = 0.30     # 下限(滤掉远距离的小噪点误配)

def blob_size_ratio(b, cid):
    _, wy = box_to_world(b.x(), b.y(), b.w(), b.h())
    dist_mm = wy * 10.0
    if dist_mm < 80.0:
        return 1.0, dist_mm   # 过近时物体必然贴边截断, 宽度已不可信
    exp_w = TARGET_REAL_W_MM.get(cid, 70.0) * FOCAL_LENGTH / dist_mm
    return b.w() / exp_w, dist_mm

def blob_size_plausible(b, cid):
    ratio, dist_mm = blob_size_ratio(b, cid)
    return SIZE_RATIO_MIN <= ratio <= SIZE_RATIO_MAX

def blob_reject_reason(img, b, cid):
    """返回 None=通过, 'gnd'=判为地面/场外, 'off'=中心色度不在本槽阈值盒内,
    'size'=尺寸与距离不符(布面反光斑块的主要死因)"""
    if blue_bag_side_gutter_blob(b, cid):
        return 'gnd'
    if blue_bag_bottom_shadow_blob(b, cid):
        return 'gnd'
    if tiny_blob_outside_cut(b):
        return 'gnd'
    if cut_valid and b.cy() < cut_line_y_at_x(b.cx()) + cut_delta_for_blob(b):
        return 'gnd'    # 分界线以上=场外(生产同款判定)
    label = slot_to_label(cid)
    if label == 1 and (b.w() < BALL_SMALL_MIN_W or b.h() < BALL_SMALL_MIN_H):
        return 'size'
    if label == 2 and (b.w() < BAG_SMALL_MIN_W or b.h() < BAG_SMALL_MIN_H):
        return 'size'
    if label == 0 and b.h() < bear_small_min_h(cid):
        return 'size'
    if not blob_size_plausible(b, cid):
        return 'size'
    lab = _blob_center_stats(img, b)
    gnd_margin = ground_margin_for_slot(b, cid)
    if ground_box is not None and _in_box(lab, ground_box, gnd_margin):
        return 'gnd'
    if ground_box_far is not None and _in_box(lab, ground_box_far, gnd_margin):
        return 'gnd'
    if not _in_box(lab, all_color_thresholds[cid - 1], off_margin_for_blob(b)):
        return 'off'
    return None

target_color_id = 0     # 0=不限色(多色检测); 0x03 命令锁定后只检测该色

# ---- 上行数据帧(12字节, 坐标单位0.01 cm / 0.1 mm) ----
WORLD_UART_UNITS_PER_CM = 100.0

def world_cm_to_uart_units(value_cm):
    scaled = float(value_cm) * WORLD_UART_UNITS_PER_CM
    if scaled >= 0.0:
        return int(scaled + 0.5)
    return int(scaled - 0.5)

def send_world_data(color_id, wx_01mm, wy_01mm, pw, yellow_flag=False, pos_flag=0x00):
    x = int(wx_01mm)
    y = int(wy_01mm)
    if x < -32768: x = -32768
    if x > 32767: x = 32767
    if y < -32768: y = -32768
    if y > 32767: y = 32767
    data = bytearray(12)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = color_id & 0xFF
    data[3] = x & 0xFF
    data[4] = (x >> 8) & 0xFF
    data[5] = y & 0xFF
    data[6] = (y >> 8) & 0xFF
    data[7] = pw & 0xFF
    data[8] = (pw >> 8) & 0xFF
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[11] = sum(data[2:11]) & 0xFF
    uart_write(data)

# ======================================================================
# 自动标定 (0x04 触发; 模型仅在此处使用)
# ======================================================================
AC_METER_ROI = (0, 140, 320, 100)
                                    # 覆盖更宽布面(太窄的ROI采出的地面盒代表性不足)
AC_METER_ROI_FAR = (0, 15, 320, 45)
AC_GB_L_M, AC_GB_AB_M = 12, 8        # 地面盒外扩余量
AC_L_TARGET = 40.0   # 45→40: 全画面测光下再保守一档, 宁欠勿过
AC_EXPOSURE_MIN = 100
AC_EXPOSURE_MAX = 4500
AC_DET_SCORE = 0.20
                      # 标定时物体静止摆放, 多帧取样+质量门足以滤掉误检, 门槛可以放低
AC_SAMPLE_FRAMES = 10
AC_N_OBJECTS = 5
AC_CALIB_SLOT_PHASES = ((1, 2), (3,), (4, 5))
AC_SWAP_IDLE = 450
AC_SWAP_CLEAR_FRAMES = 8
AC_SAMPLE_L_CLIP = 98
AC_IQR_L_MAX = 80
AC_IQR_AB_MAX = 100
AC_IQR_AB_MAX_BALL = 90
AC_BOX_EXPAND = 1.25
AC_BOX_EXPAND_H = 1.6
AC_BOX_OFF_X = -1
AC_BOX_OFF_Y = 0
AC_SAMPLE_ROI_W_FRAC = 0.80
AC_SAMPLE_ROI_H_FRAC = 0.85
AC_BEAR_GND_RETRY_ROI_W_FRAC = 0.55
AC_BEAR_GND_RETRY_ROI_H_FRAC = 0.30
AC_BEAR_GND_RETRY_Y_FRAC = 0.25
AC_IQR_K = 0.8
AC_AB_SPAN_MAX = 45
AC_AB_SPAN_MAX_BALL = 75
AC_AB_SPAN_MAX_BAG = 60
AC_L_SPAN_MAX = 60
AC_AB_MARGIN = 5
AC_L_WIDE = (15, 100)
AC_NEUTRAL_AB = 12
AC_L_MARGIN_LOWSAT = 8
AC_MIN_BOX_W = 10
AC_GND_NEAR = 10
AC_GND_CUT_W = 0.35
RED_BAG_GND_L_PAD_LO = 4
RED_BAG_GND_L_PAD_HI = 8
BLUE_BAG_GND_B_MAX = -25
BLUE_BAG_B_MARGIN_HI = 12
BLUE_BAG_A_CUT_MIN_GAP = 8
BLUE_BAG_A_CUT_GAP = 6
AC_BAG_EDGE_AB_MAX_DELTA = 35
AC_BAG_EDGE_AB_MAX_DELTA_BLUE = 22
AC_BAG_EDGE_L_PAD = 8
AC_BAG_EDGE_AB_PAD = 10
AC_BAG_EDGE_L_SPAN_MAX = 70
AC_BAG_EDGE_AB_SPAN_MAX = 90

def _ac_median(lst):
    s = sorted(lst)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) * 0.5

def _ac_merge_stat_tuple(s, st):
    s[0] = min(s[0], st.l_lq())
    s[1] = max(s[1], st.l_uq())
    s[2] = min(s[2], st.a_lq())
    s[3] = max(s[3], st.a_uq())
    s[4] = min(s[4], st.b_lq())
    s[5] = max(s[5], st.b_uq())

def _ac_try_merge_bag_edge_stat(s, st):
    # 边带可能混入少量地面/背景, 只取中位数附近小窗口, 不并入整块 LQ/UQ。
    lm, am, bm = st.l_median(), st.a_median(), st.b_median()
    cand = [s[i] for i in range(6)]
    cand[0] = min(cand[0], max(0, lm - AC_BAG_EDGE_L_PAD))
    cand[1] = max(cand[1], min(100, lm + AC_BAG_EDGE_L_PAD))
    cand[2] = min(cand[2], max(-128, am - AC_BAG_EDGE_AB_PAD))
    cand[3] = max(cand[3], min(127, am + AC_BAG_EDGE_AB_PAD))
    cand[4] = min(cand[4], max(-128, bm - AC_BAG_EDGE_AB_PAD))
    cand[5] = max(cand[5], min(127, bm + AC_BAG_EDGE_AB_PAD))
    if (cand[1] - cand[0] > AC_BAG_EDGE_L_SPAN_MAX
            or cand[3] - cand[2] > AC_BAG_EDGE_AB_SPAN_MAX
            or cand[5] - cand[4] > AC_BAG_EDGE_AB_SPAN_MAX):
        return False
    for i in range(6):
        s[i] = cand[i]
    return True

def _ac_merge_bag_edge_strips(img, s, roi, gnd_boxes_for_edge=None):
    rx, ry, rw, rh = roi
    if rw < 4 or rh < 4:
        return
    sw = max(3, rw // 5)
    sh = max(3, rh // 5)
    rois = ((rx, ry, rw, sh),
            (rx, ry + rh - sh, rw, sh),
            (rx, ry, sw, rh),
            (rx + rw - sw, ry, sw, rh))
    is_blue_bag = s[8] < -10
    max_delta = AC_BAG_EDGE_AB_MAX_DELTA_BLUE if is_blue_bag else AC_BAG_EDGE_AB_MAX_DELTA
    for eroi in rois:
        st = img.get_statistics(roi=eroi)
        lab = (st.l_median(), st.a_median(), st.b_median())
        if is_blue_bag and gnd_boxes_for_edge:
            if any(_in_box(lab, g) for g in gnd_boxes_for_edge):
                continue
        if (abs(lab[1] - s[7]) > max_delta
                or abs(lab[2] - s[8]) > max_delta):
            continue
        _ac_try_merge_bag_edge_stat(s, st)

def _flush_calib_log():
    """标定结束时重写一次完整日志; 运行中已实时追加, 卡住也能看到现场。"""
    global AC_LOG_LIVE
    AC_LOG_LIVE = False
    try:
        with open(CALIB_LOG_FILE, 'w') as fp:
            for line in AC_LOG:
                fp.write(line + "\n")
    except Exception as e:
        print("!! log write fail:", e)
    del AC_LOG[:]

def _rect_overlap(r1, r2):
    return not (r1[0] + r1[2] <= r2[0] or r2[0] + r2[2] <= r1[0]
                or r1[1] + r1[3] <= r2[1] or r2[1] + r2[3] <= r1[1])

def _make_gnd_box(img, roi, exclude_boxes, tag=""):
    """分块地面采样: ROI 切 8x3 块, 跳过与物体框相交的块, 剩余块的中位数做
    离群剔除(未检出的物体/场外内容被剔), 幸存块中位数范围±外扩=地面盒。"""
    tw = max(8, roi[2] // 8)
    th = max(8, roi[3] // 3)
    meds = []
    for iy in range(3):
        for ix in range(8):
            tx = roi[0] + ix * tw
            ty = roi[1] + iy * th
            if tx + tw > 320 or ty + th > 240:
                continue
            tile = (tx, ty, tw, th)
            # 分界线以上(赛道外)的块不作为地面取样
            if cut_valid and (ty + th // 2) < cut_line_y_at_x(tx + tw // 2) + CUT_BLOB_DELTA:
                continue
            skip = False
            for eb in exclude_boxes:
                if _rect_overlap(tile, eb):
                    skip = True
                    break
            if skip:
                continue
            st = img.get_statistics(roi=tile)
            meds.append((st.l_median(), st.a_median(), st.b_median()))
    if len(meds) < 6:
        _clog("[calib] !! {} 地面可用块太少({})".format(tag, len(meds)))
        return None, None
    ml = _ac_median([m[0] for m in meds])
    ma = _ac_median([m[1] for m in meds])
    mb = _ac_median([m[2] for m in meds])
    keep = [m for m in meds if abs(m[0] - ml) <= 15
            and abs(m[1] - ma) <= 10 and abs(m[2] - mb) <= 10]
    if len(meds) - len(keep):
        _clog("[calib] {} 地面剔除离群块 {}/{}".format(tag, len(meds) - len(keep), len(meds)))
    if len(keep) < 4:
        keep = meds
    box = (min(m[0] for m in keep) - AC_GB_L_M, max(m[0] for m in keep) + AC_GB_L_M,
           min(m[1] for m in keep) - AC_GB_AB_M, max(m[1] for m in keep) + AC_GB_AB_M,
           min(m[2] for m in keep) - AC_GB_AB_M, max(m[2] for m in keep) + AC_GB_AB_M)
    return (ml, ma, mb), box

def send_calib_slot(slot, t):
    """槽位阈值详情帧(11字节): AA 55 C5 slot L0 L1 A0 A1 B0 B1 ck
    A/B 为 int8 补码; 供主控 LCD 显示标定结果(脱机无 IDE 时唯一可见渠道)。"""
    data = bytearray(11)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = 0xC5
    data[3] = slot & 0xFF
    data[4] = clamp_int(t[0], 0, 100)
    data[5] = clamp_int(t[1], 0, 100)
    data[6] = clamp_int(t[2], -128, 127) & 0xFF
    data[7] = clamp_int(t[3], -128, 127) & 0xFF
    data[8] = clamp_int(t[4], -128, 127) & 0xFF
    data[9] = clamp_int(t[5], -128, 127) & 0xFF
    data[10] = sum(data[2:10]) & 0xFF
    uart_write(data)
    time.sleep_ms(5)   # 帧间隔, 防主控解析粘包压力

def send_calib_ack(status, count, exposure):
    exp = int(exposure) & 0xFFFF
    data = bytearray(8)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = 0xC4
    data[3] = status & 0xFF
    data[4] = count & 0xFF
    data[5] = exp & 0xFF
    data[6] = (exp >> 8) & 0xFF
    data[7] = (data[2] + data[3] + data[4] + data[5] + data[6]) & 0xFF
    uart_write(data)

def _ac_calibrate_exposure():
    """全画面测光(与生产 TARGET_BRIGHTNESS=45 的语义一致) + 高光防炸。
    不能对着深蓝布 ROI 测光: 布反射率低, 拉它到 45 会把曝光翻倍,
    白熊/网球直接过曝饱和(L→100, 色度→0), 阈值全毁。"""
    exp = 1200
    sensor.set_auto_exposure(False, exposure_us=exp)
    for it in range(10):
        time.sleep_ms(150)
        for _ in range(3):
            img = sensor.snapshot()
        # 4帧平均测光: 单帧噪声±2~3个L, 物体挪几厘米就可能跨过容差边界,
        # 导致同光线下两次标定曝光差200+
        l_sum = 0.0
        luq = 0
        for _ in range(4):
            img = sensor.snapshot()
            st = img.get_statistics()
            l_sum += st.l_mean()
            luq = st.l_uq()
        l = l_sum / 4.0
        _clog("[calib] exp iter {}: {}us L={:.1f} Luq={}".format(it, exp, l, luq))
        if abs(l - AC_L_TARGET) <= 10.0:   # 容差 3→5: 收敛更早, 曝光调节不那么激进
            break
        ratio = AC_L_TARGET / max(l, 1.0)
        ratio = min(max(ratio, 0.85), 1.25)   # 单轮步幅 ±60%→±25%, 防过冲
        exp = int(exp * ratio)
        exp = min(max(exp, AC_EXPOSURE_MIN), AC_EXPOSURE_MAX)
        sensor.set_auto_exposure(False, exposure_us=exp)
    # 高光防炸: 画面 P75 亮度顶到 96+ 说明大片饱和(白熊/球会炸白), 逐步压曝光
    for it in range(5):
        for _ in range(3):
            img = sensor.snapshot()
        luq = img.get_statistics().l_uq()
        if luq < 96:
            break
        exp = max(int(exp * 0.8), AC_EXPOSURE_MIN)
        _clog("[calib] 高光饱和(Luq={}), 压曝光 -> {}us".format(luq, exp))
        sensor.set_auto_exposure(False, exposure_us=exp)
        time.sleep_ms(150)
    return exp

def _ac_cut_blue_bag_ground_by_a(a0, a1, a_med, ground_boxes):
    cut = False
    for g in ground_boxes:
        ga0 = g[2]
        if a_med <= ga0 - BLUE_BAG_A_CUT_MIN_GAP:
            na1 = ga0 - BLUE_BAG_A_CUT_GAP
            if na1 >= a0:
                a1 = min(a1, na1)
                cut = True
    return a0, a1, cut

def _ac_build_threshold(m, ground=None, gnd_box=None, gnd_boxes_for_conflict=None, slot=0):
    """m: 合并后统计量; ground: (L,A,B) 地面(赛道布)中位数, 作负样本。
    高饱和色默认放宽 L; 但若地面色度落进该色的 A/B 盒(如深蓝布 vs 淡蓝沙袋,
    区别主要在 L), 则保留实测 L 并把 L 边界收到物体与地面的中点。"""
    llq, luq, alq, auq, blq, buq = m[0], m[1], m[2], m[3], m[4], m[5]
    l_med = m[6]
    a_med, b_med = m[7], m[8]
    l0 = llq - AC_IQR_K * (luq - llq)
    l1 = luq + AC_IQR_K * (luq - llq)
    a0 = alq - AC_IQR_K * (auq - alq) - AC_AB_MARGIN
    a1 = auq + AC_IQR_K * (auq - alq) + AC_AB_MARGIN
    b0 = blq - AC_IQR_K * (buq - blq) - AC_AB_MARGIN
    b1 = buq + AC_IQR_K * (buq - blq) + AC_AB_MARGIN
    def axis_dist(lo, hi):
        return 0.0 if lo <= 0 <= hi else min(abs(lo), abs(hi))
    low_sat = (axis_dist(a0, a1) < AC_NEUTRAL_AB and axis_dist(b0, b1) < AC_NEUTRAL_AB)
    # 地面冲突判定: 用"两盒带余量接近"而非"地面中位数在盒内"。
    # 布在远处/掠射角下 A/B 会漂移 10-20, 中位数隔着十几也照样会漂进来
    # (实测: 蓝包盒 A 上界 4, 布 A 中位 27, 判无冲突放宽了 L → 布又误报)
    ground_conflict_ls = []
    ground_conflict_boxes = []
    if slot != 3:
        for g in (gnd_boxes_for_conflict or ([gnd_box] if gnd_box else [])):
            if not g:
                continue
            ga0, ga1, gb0, gb1 = g[2], g[3], g[4], g[5]
            a_touch = not (a1 + AC_GND_NEAR < ga0 or ga1 + AC_GND_NEAR < a0)
            b_touch = not (b1 + AC_GND_NEAR < gb0 or gb1 + AC_GND_NEAR < b0)
            if a_touch and b_touch:
                ground_conflict_ls.append((g[0] + g[1]) * 0.5)
                ground_conflict_boxes.append(g)
    ground_in_ab = bool(ground_conflict_ls)
    blue_bag_a_cut = False
    if slot == 1 and ground_conflict_boxes:
        a0, a1, blue_bag_a_cut = _ac_cut_blue_bag_ground_by_a(
            a0, a1, a_med, ground_conflict_boxes)
        if blue_bag_a_cut:
            _clog("[calib] blue_bag A切地面, L放宽到 ({}, {}), A=({},{})".format(
                AC_L_WIDE[0], AC_L_WIDE[1], int(a0), int(a1)))
    if slot == 3 or blue_bag_a_cut:
        l0, l1 = AC_L_WIDE
    elif low_sat or ground_in_ab:
        # 保留实测 L 区分亮暗(棕/白熊排白杆; 淡蓝包排深蓝布)
        l0 = max(0, l0 - AC_L_MARGIN_LOWSAT)
        l1 = min(100, l1 + AC_L_MARGIN_LOWSAT)
        if ground_in_ab:
            # 切割点偏向地面(35%处)而非中点: 暗光下物体与布的 L 只差 10-15,
            # 中点会切进物体本体致 blob 破碎(实测: 切 40, 蓝包中位数才 46)。
            # 漏进来的布缘像素由运行时地面复核+尺寸检查兜底。
            for gl in ground_conflict_ls:
                if gl < l_med:      # 地面更暗: 抬高下界
                    l0 = max(l0, gl + AC_GND_CUT_W * (l_med - gl))
                else:               # 地面更亮: 压低上界
                    l1 = min(l1, gl - AC_GND_CUT_W * (gl - l_med))
            _clog("[calib] 地面冲突, L 收紧到 ({}, {})".format(int(l0), int(l1)))
    else:
        l0, l1 = AC_L_WIDE
    # 跨度硬上限: 绕中位数收紧, 防止污染样本把盒撑爆(蓝包 A 曾到 -60..30)
    if slot == 3:
        ab_span_max = AC_AB_SPAN_MAX_BALL
    elif slot in (1, 2):
        ab_span_max = AC_AB_SPAN_MAX_BAG
    else:
        ab_span_max = AC_AB_SPAN_MAX
    if a1 - a0 > ab_span_max:
        a0 = max(a0, a_med - ab_span_max / 2)
        a1 = min(a1, a_med + ab_span_max / 2)
    if b1 - b0 > ab_span_max:
        b0 = max(b0, b_med - ab_span_max / 2)
        b1 = min(b1, b_med + ab_span_max / 2)
    if l1 - l0 > AC_L_SPAN_MAX and (l0, l1) != AC_L_WIDE:
        l0 = max(l0, l_med - AC_L_SPAN_MAX / 2)
        l1 = min(l1, l_med + AC_L_SPAN_MAX / 2)
    return (int(l0), int(l1), int(a0), int(a1), int(b0), int(b1))

def tighten_slot_threshold(slot, t, m):
    if slot == 1:
        b1 = min(t[5], int(m[8] + BLUE_BAG_B_MARGIN_HI))
        t = (t[0], t[1], t[2], t[3], t[4], max(t[4], b1))
    return t

def _ac_allow_ground_sample(label, s):
    # 只给蓝沙袋采样放过地面盒, 避免蓝包 B 通道接近赛道布时被误杀。
    return label == 2 and s[8] <= BLUE_BAG_GND_B_MAX

def _ac_assign_slot(label, m):
    l_med, a_med, b_med = m[6], m[7], m[8]
    if label == 1:
        return 3
    if label == 2:
        return 1 if b_med < -10 else 2
    if label == 0:
        if (l_med >= WHT_BEAR_L_MIN and abs(a_med) <= WHT_BEAR_A_ABS_MAX
                and b_med <= WHT_BEAR_B_MAX):
            return 5
        return 4
    return 0

def _ac_ab_overlap(t1, t2):
    return not (t1[3] < t2[2] or t2[3] < t1[2] or t1[5] < t2[4] or t2[5] < t1[4])


# ---- IDE-style staged multi calibration helpers ----
AC_PHASE_EXP = 1
AC_PHASE_GND = 2
AC_PHASE_WAIT = 3
AC_PHASE_SAMPLE = 4
AC_PHASE_THR = 5
AC_PHASE_SAVE = 6
AC_PHASE_DONE = 7
AC_PHASE_FAIL = 8

AC_REASON_NONE = 0
AC_REASON_NO_MODEL = 1
AC_REASON_LOW_SCORE = 2
AC_REASON_PHASE = 3
AC_REASON_OVEREXP = 4
AC_REASON_GROUND = 5
AC_REASON_IQR = 6
AC_REASON_DONE = 7
AC_REASON_STABLE = 8
AC_REASON_TIMEOUT = 9
AC_REASON_BAD_THR = 10
AC_REASON_NO_GROUND = 11
AC_REASON_SWAP = 12
AC_SWAP_ARG_OLD_BASE = 200


def _ac_slot_mask(slots):
    mask = 0
    if not slots:
        return 0
    for s in slots:
        if 1 <= int(s) <= 8:
            mask |= 1 << (int(s) - 1)
    return mask


def send_calib_progress(phase, slot=0, sample=0, total=0, done_mask=0,
                        wait_mask=0, reason=0, arg=0):
    arg = clamp_int(int(arg), 0, 65535)
    data = bytearray(13)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = 0xC6
    data[3] = int(phase) & 0xFF
    data[4] = int(slot) & 0xFF
    data[5] = int(sample) & 0xFF
    data[6] = int(total) & 0xFF
    data[7] = int(done_mask) & 0xFF
    data[8] = int(wait_mask) & 0xFF
    data[9] = int(reason) & 0xFF
    data[10] = arg & 0xFF
    data[11] = (arg >> 8) & 0xFF
    data[12] = sum(data[2:12]) & 0xFF
    uart_write(data)
    time.sleep_ms(2)


def _ac_detect_boxes(img):
    out = []
    img1 = img.copy(0.75, 1)
    for obj in tf.detect(net, img1):
        x1, y1, x2, y2, label, score = obj
        x = int(float(x1) * img.width())
        y = int(float(y1) * img.height())
        w = int((float(x2) - float(x1)) * img.width())
        h = int((float(y2) - float(y1)) * img.height())
        out.append((int(label), float(score), x, y, w, h))
    return out


def _ac_correct_box(x, y, w, h):
    cx = x + w / 2.0 + AC_BOX_OFF_X
    cy = y + h / 2.0 + AC_BOX_OFF_Y
    nw = w * AC_BOX_EXPAND
    nh = h * AC_BOX_EXPAND_H
    nx = clamp_int(int(cx - nw / 2), 0, 319)
    ny = clamp_int(int(cy - nh / 2), 0, 239)
    return (nx, ny, clamp_int(int(nw), 1, 320 - nx),
            clamp_int(int(nh), 1, 240 - ny))


def _ac_draw_model_box(img, det, ok=True):
    label, score, x, y, w, h = det
    try:
        img.draw_rectangle(x, y, w, h, color=(255, 255, 0))
        img.draw_string(x, max(0, y - 10), "%s %.2f" % (
            LABEL_NAMES.get(label, "?"), score), color=(255, 255, 0))
        img.draw_rectangle(_ac_correct_box(x, y, w, h), color=(0, 255, 255))
        if not ok:
            img.draw_line(x, y, x + w, y + h, color=(255, 0, 0))
    except Exception:
        pass


def _ac_build_threshold(m, ground=None, gnd_box=None, gnd_boxes_for_conflict=None, slot=0):
    llq, luq, alq, auq, blq, buq = m[0], m[1], m[2], m[3], m[4], m[5]
    l_med = m[6]
    a_med, b_med = m[7], m[8]
    l0 = llq - AC_IQR_K * (luq - llq)
    l1 = luq + AC_IQR_K * (luq - llq)
    a0 = alq - AC_IQR_K * (auq - alq) - AC_AB_MARGIN
    a1 = auq + AC_IQR_K * (auq - alq) + AC_AB_MARGIN
    b0 = blq - AC_IQR_K * (buq - blq) - AC_AB_MARGIN
    b1 = buq + AC_IQR_K * (buq - blq) + AC_AB_MARGIN

    def axis_dist(lo, hi):
        return 0.0 if lo <= 0 <= hi else min(abs(lo), abs(hi))

    low_sat = (axis_dist(a0, a1) < AC_NEUTRAL_AB and
               axis_dist(b0, b1) < AC_NEUTRAL_AB)
    ground_conflict_ls = []
    ground_conflict_boxes = []
    if slot != 3:
        for g in (gnd_boxes_for_conflict or ([gnd_box] if gnd_box else [])):
            if not g:
                continue
            ga0, ga1, gb0, gb1 = g[2], g[3], g[4], g[5]
            a_touch = not (a1 + AC_GND_NEAR < ga0 or ga1 + AC_GND_NEAR < a0)
            b_touch = not (b1 + AC_GND_NEAR < gb0 or gb1 + AC_GND_NEAR < b0)
            if a_touch and b_touch:
                ground_conflict_ls.append((g[0] + g[1]) * 0.5)
                ground_conflict_boxes.append(g)
    ground_in_ab = bool(ground_conflict_ls)
    blue_bag_a_cut = False
    if slot == 1 and ground_conflict_boxes:
        a0, a1, blue_bag_a_cut = _ac_cut_blue_bag_ground_by_a(
            a0, a1, a_med, ground_conflict_boxes)
        if blue_bag_a_cut:
            _clog("[calib] blue_bag A cut ground, L wide ({}, {}), A=({},{})".format(
                AC_L_WIDE[0], AC_L_WIDE[1], int(a0), int(a1)))
    if slot == 3 or blue_bag_a_cut:
        l0, l1 = AC_L_WIDE
    elif low_sat or ground_in_ab:
        l0 = max(0, l0 - AC_L_MARGIN_LOWSAT)
        l1 = min(100, l1 + AC_L_MARGIN_LOWSAT)
        if ground_in_ab:
            for gl in ground_conflict_ls:
                if gl < l_med:
                    l0 = max(l0, gl + AC_GND_CUT_W * (l_med - gl))
                else:
                    l1 = min(l1, gl - AC_GND_CUT_W * (gl - l_med))
            if slot == 2:
                l0 = max(0, l0 - RED_BAG_GND_L_PAD_LO)
                l1 = min(100, l1 + RED_BAG_GND_L_PAD_HI)
            _clog("[calib] ground conflict, L=({}, {})".format(int(l0), int(l1)))
    else:
        l0, l1 = AC_L_WIDE

    if slot == 3:
        ab_span_max = AC_AB_SPAN_MAX_BALL
    elif slot in (1, 2):
        ab_span_max = AC_AB_SPAN_MAX_BAG
    else:
        ab_span_max = AC_AB_SPAN_MAX
    if a1 - a0 > ab_span_max:
        a0 = max(a0, a_med - ab_span_max / 2)
        a1 = min(a1, a_med + ab_span_max / 2)
    if b1 - b0 > ab_span_max:
        b0 = max(b0, b_med - ab_span_max / 2)
        b1 = min(b1, b_med + ab_span_max / 2)
    if l1 - l0 > AC_L_SPAN_MAX and (l0, l1) != AC_L_WIDE:
        l0 = max(l0, l_med - AC_L_SPAN_MAX / 2)
        l1 = min(l1, l_med + AC_L_SPAN_MAX / 2)
    return (int(l0), int(l1), int(a0), int(a1), int(b0), int(b1))


def _ac_slot_list_names(slots):
    if slots is None:
        return "any"
    if not slots:
        return "-"
    return "/".join(SLOT_NAMES.get(s, str(s)) for s in slots)


def _ac_allowed_collect_slots(done_slots):
    if not AC_CALIB_SLOT_PHASES:
        return None
    for phase in AC_CALIB_SLOT_PHASES:
        remain = tuple(s for s in phase if s not in done_slots)
        if remain:
            return remain
    return None


def _ac_label_possible_for_slots(label, slots):
    if slots is None:
        return True
    if label == 1:
        return 3 in slots
    if label == 2:
        return 1 in slots or 2 in slots
    if label == 0:
        return 4 in slots or 5 in slots
    return False


def _ac_collect_slot_priority(slot, exclude_slots, allowed_slots=None):
    if allowed_slots is not None and slot in allowed_slots:
        return 2
    if (1 in exclude_slots) != (2 in exclude_slots):
        remaining_bag = 2 if 1 in exclude_slots else 1
        if slot == remaining_bag:
            return 1
    return 0


def brn_bear_box_stable(prev, box):
    if prev is None:
        return False
    pcx = prev[0] + prev[2] // 2
    pcy = prev[1] + prev[3] // 2
    cx = box[0] + box[2] // 2
    cy = box[1] + box[3] // 2
    return (abs(cx - pcx) <= BRN_BEAR_STABLE_CENTER_PX
            and abs(cy - pcy) <= BRN_BEAR_STABLE_CENTER_PX
            and abs(box[2] - prev[2]) <= BRN_BEAR_STABLE_SIZE_PX
            and abs(box[3] - prev[3]) <= BRN_BEAR_STABLE_SIZE_PX)


def _ac_sample_det(img, det, gnd_boxes_for_edge, draw_debug=True):
    label, score, x, y, w, h = det
    x, y, w, h = _ac_correct_box(x, y, w, h)
    rw = max(4, int(w * AC_SAMPLE_ROI_W_FRAC))
    rh = max(4, int(h * AC_SAMPLE_ROI_H_FRAC))
    rx = clamp_int(x + (w - rw) // 2, 0, 319)
    ry = clamp_int(y + (h - rh) // 2, 0, 239)
    roi = (rx, ry, min(rw, 320 - rx), min(rh, 240 - ry))
    st = img.get_statistics(roi=roi)
    s = [st.l_lq(), st.l_uq(), st.a_lq(), st.a_uq(), st.b_lq(), st.b_uq(),
         st.l_median(), st.a_median(), st.b_median()]
    if label == 0 and any(_in_box((s[6], s[7], s[8]), g) for g in (gnd_boxes_for_edge or ()) if g):
        rw2 = max(4, int(w * AC_BEAR_GND_RETRY_ROI_W_FRAC))
        rh2 = max(4, int(h * AC_BEAR_GND_RETRY_ROI_H_FRAC))
        rx2 = clamp_int(x + (w - rw2) // 2, 0, 319)
        ry2 = clamp_int(y + int(h * AC_BEAR_GND_RETRY_Y_FRAC), 0, 239)
        roi2 = (rx2, ry2, min(rw2, 320 - rx2), min(rh2, 240 - ry2))
        st2 = img.get_statistics(roi=roi2)
        s2 = [st2.l_lq(), st2.l_uq(), st2.a_lq(), st2.a_uq(), st2.b_lq(), st2.b_uq(),
              st2.l_median(), st2.a_median(), st2.b_median()]
        if not any(_in_box((s2[6], s2[7], s2[8]), g) for g in (gnd_boxes_for_edge or ()) if g):
            roi = roi2
            s = s2
    if draw_debug:
        try:
            img.draw_rectangle(roi, color=(0, 255, 0))
        except Exception:
            pass
    if label == 1:
        bh = max(3, h // 3)
        bx = clamp_int(x + w // 6, 0, 319)
        by = clamp_int(y + h - bh, 0, 239)
        broi = (bx, by, clamp_int(w * 2 // 3, 4, 320 - bx), min(bh, 240 - by))
        st2 = img.get_statistics(roi=broi)
        if draw_debug:
            try:
                img.draw_rectangle(broi, color=(0, 255, 0))
            except Exception:
                pass
        s[0] = min(s[0], st2.l_lq())
        s[1] = max(s[1], st2.l_uq())
        s[2] = min(s[2], st2.a_lq())
        s[3] = max(s[3], st2.a_uq())
        s[4] = min(s[4], st2.b_lq())
        s[5] = max(s[5], st2.b_uq())
    elif label == 2:
        if s[8] >= -10:
            _ac_merge_bag_edge_strips(img, s, roi, gnd_boxes_for_edge)
    return tuple(s)


def _ac_collect_one(exclude_slots, allowed_slots, ground, gnd_boxes):
    if allowed_slots is not None:
        allowed_slots = tuple(allowed_slots)
    _clog("[collect] begin wait={} done={}".format(
        _ac_slot_list_names(allowed_slots), _ac_slot_list_names(tuple(sorted(exclude_slots)))))
    samples = []
    target_slot = -1
    target_label = -1
    idle = 0
    hint_tick = 0
    brn_bear_stable_box = None
    brn_bear_stable_count = 0
    brn_bear_stable_hint = 0
    red_bag_stable_box = None
    red_bag_stable_count = 0
    red_bag_stable_hint = 0
    red_bag_track_box = None
    swap_clear_count = AC_SWAP_CLEAR_FRAMES if not exclude_slots else 0
    while len(samples) < AC_SAMPLE_FRAMES:
        idle += 1
        log_det = (idle % 30 == 1)
        if idle > AC_SWAP_IDLE:
            _clog("[collect] timeout target={} samples={} wait={} done={}".format(
                SLOT_NAMES.get(target_slot, target_slot), len(samples),
                _ac_slot_list_names(allowed_slots),
                _ac_slot_list_names(tuple(sorted(exclude_slots)))))
            send_calib_progress(AC_PHASE_WAIT, target_slot if target_slot > 0 else 0,
                                len(samples), AC_SAMPLE_FRAMES,
                                _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                AC_REASON_TIMEOUT, idle)
            return None
        img = sensor.snapshot()
        update_cut_line(img)
        try:
            img.draw_rectangle(AC_METER_ROI, color=(120, 120, 0))
            img.draw_rectangle(AC_METER_ROI_FAR, color=(120, 120, 0))
        except Exception:
            pass
        dets = sorted(_ac_detect_boxes(img), key=lambda d: d[1], reverse=True)
        draw_cut_line_debug(img)
        if not dets:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            if exclude_slots and swap_clear_count < AC_SWAP_CLEAR_FRAMES:
                swap_clear_count += 1
                idle = 0
                hint_tick += 1
                if hint_tick % 8 == 1:
                    _clog("[collect] swap clear {}/{} wait={} done={}".format(
                        swap_clear_count, AC_SWAP_CLEAR_FRAMES,
                        _ac_slot_list_names(allowed_slots),
                        _ac_slot_list_names(tuple(sorted(exclude_slots)))))
                    send_calib_progress(AC_PHASE_WAIT, 0, len(samples), AC_SAMPLE_FRAMES,
                                        _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                        AC_REASON_SWAP, swap_clear_count)
                continue
            hint_tick += 1
            if hint_tick % 30 == 1:
                _clog("[collect] no_model idle={} wait={} done={} samples={}".format(
                    idle, _ac_slot_list_names(allowed_slots),
                    _ac_slot_list_names(tuple(sorted(exclude_slots))), len(samples)))
                send_calib_progress(AC_PHASE_WAIT, 0, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_NO_MODEL, idle)
            continue
        candidates = []
        reject_draw = None
        done_hint_slot = None
        phase_hint_slot = 0
        phase_hint_name = None
        bad_hint = None
        last_reason = AC_REASON_NONE
        for det in dets:
            label, score, x, y, w, h = det
            if score < AC_DET_SCORE or w < AC_MIN_BOX_W or h < AC_MIN_BOX_W:
                if reject_draw is None:
                    reject_draw = det
                if bad_hint is None:
                    bad_hint = (label, score, w)
                last_reason = AC_REASON_LOW_SCORE
                continue
            if not _ac_label_possible_for_slots(label, allowed_slots):
                if reject_draw is None:
                    reject_draw = det
                phase_hint_name = LABEL_NAMES.get(label, "?")
                last_reason = AC_REASON_PHASE
                continue
            s = _ac_sample_det(img, det, gnd_boxes, False)
            reason = None
            reason_code = AC_REASON_NONE
            ab_lim = AC_IQR_AB_MAX_BALL if label == 1 else AC_IQR_AB_MAX
            l_lim = AC_IQR_L_MAX
            gnd_hit = any(_in_box((s[6], s[7], s[8]), g) for g in gnd_boxes if g)
            if s[6] >= AC_SAMPLE_L_CLIP:
                reason = "overexp Lmed=%d" % s[6]
                reason_code = AC_REASON_OVEREXP
            elif gnd_hit and not _ac_allow_ground_sample(label, s):
                reason = "ground sample"
                reason_code = AC_REASON_GROUND
            elif ((s[1] - s[0]) > l_lim or (s[3] - s[2]) > ab_lim
                  or (s[5] - s[4]) > ab_lim):
                reason = "wide IQR=(%d,%d,%d)" % (s[1] - s[0], s[3] - s[2], s[5] - s[4])
                reason_code = AC_REASON_IQR
            if reason:
                if reject_draw is None:
                    reject_draw = det
                _clog("[det] {} score={:.2f} med=({},{},{}) drop: {}".format(
                    LABEL_NAMES.get(label, "?"), score, s[6], s[7], s[8], reason))
                last_reason = reason_code
                continue
            elif gnd_hit:
                _clog("[det] blue_bag med=({},{},{}) ground hit but accepted".format(
                    s[6], s[7], s[8]))
            slot = _ac_assign_slot(label, s)
            if slot <= 0:
                if reject_draw is None:
                    reject_draw = det
                if log_det:
                    _clog("[map] label={} score={:.2f} med=({},{},{}) -> slot=0 wait={} done={}".format(
                        LABEL_NAMES.get(label, "?"), score, s[6], s[7], s[8],
                        _ac_slot_list_names(allowed_slots),
                        _ac_slot_list_names(tuple(sorted(exclude_slots)))))
                continue
            if log_det:
                _clog("[map] label={} score={:.2f} box=({},{},{},{}) med=({},{},{}) -> {} wait={} done={}".format(
                    LABEL_NAMES.get(label, "?"), score, x, y, w, h,
                    s[6], s[7], s[8], SLOT_NAMES.get(slot, slot),
                    _ac_slot_list_names(allowed_slots),
                    _ac_slot_list_names(tuple(sorted(exclude_slots)))))
            if slot in exclude_slots:
                if reject_draw is None:
                    reject_draw = det
                done_hint_slot = slot
                last_reason = AC_REASON_DONE
                if log_det:
                    _clog("[skip] {} already_done; still_wait={} samples={} idle={}".format(
                        SLOT_NAMES.get(slot, slot), _ac_slot_list_names(allowed_slots),
                        len(samples), idle))
                continue
            if allowed_slots is not None and slot not in allowed_slots:
                if reject_draw is None:
                    reject_draw = det
                phase_hint_slot = slot
                phase_hint_name = SLOT_NAMES.get(slot, str(slot))
                last_reason = AC_REASON_PHASE
                if log_det:
                    _clog("[skip] {} phase_mismatch; wait={} samples={} idle={}".format(
                        SLOT_NAMES.get(slot, slot), _ac_slot_list_names(allowed_slots),
                        len(samples), idle))
                continue
            if target_slot >= 0 and (slot != target_slot or label != target_label):
                if reject_draw is None:
                    reject_draw = det
                if log_det:
                    _clog("[skip] {} target_locked={} samples={} idle={}".format(
                        SLOT_NAMES.get(slot, slot), SLOT_NAMES.get(target_slot, target_slot),
                        len(samples), idle))
                continue
            candidates.append((slot, label, score, x, y, w, h, s, det))

        if exclude_slots and done_hint_slot is not None:
            if reject_draw is not None:
                _ac_draw_model_box(img, reject_draw, ok=False)
            swap_clear_count = 0
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                _clog("[collect] wait swap: old {} still visible; wait={} samples={}".format(
                    SLOT_NAMES.get(done_hint_slot, done_hint_slot),
                    _ac_slot_list_names(allowed_slots), len(samples)))
                send_calib_progress(AC_PHASE_WAIT, done_hint_slot, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_SWAP, AC_SWAP_ARG_OLD_BASE + done_hint_slot)
            continue
        if exclude_slots and swap_clear_count < AC_SWAP_CLEAR_FRAMES:
            if reject_draw is not None:
                _ac_draw_model_box(img, reject_draw, ok=False)
            elif candidates:
                wait_pick = max(candidates, key=lambda it: (
                    _ac_collect_slot_priority(it[0], exclude_slots, allowed_slots), it[2]))
                _ac_draw_model_box(img, wait_pick[8], ok=False)
            swap_clear_count += 1
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                _clog("[collect] swap clear {}/{} wait={} done={}".format(
                    swap_clear_count, AC_SWAP_CLEAR_FRAMES,
                    _ac_slot_list_names(allowed_slots),
                    _ac_slot_list_names(tuple(sorted(exclude_slots)))))
                send_calib_progress(AC_PHASE_WAIT, 0, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_SWAP, swap_clear_count)
            continue

        if not candidates:
            if reject_draw is not None:
                _ac_draw_model_box(img, reject_draw, ok=False)
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            hint_tick += 1
            if done_hint_slot is not None and hint_tick % 30 == 1:
                _clog("[det] {} already calibrated, change object".format(
                    SLOT_NAMES.get(done_hint_slot, done_hint_slot)))
                send_calib_progress(AC_PHASE_WAIT, done_hint_slot, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_DONE, idle)
            elif phase_hint_name is not None and hint_tick % 30 == 1:
                _clog("[det] wait {}, ignore {}".format(
                    _ac_slot_list_names(allowed_slots), phase_hint_name))
                send_calib_progress(AC_PHASE_WAIT, phase_hint_slot, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_PHASE, idle)
            elif bad_hint is not None and hint_tick % 30 == 1:
                label, score, w = bad_hint
                _clog("[det] {} score={:.2f} box_w={} bad score/box".format(
                    LABEL_NAMES.get(label, "?"), score, w))
                send_calib_progress(AC_PHASE_WAIT, 0, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    AC_REASON_LOW_SCORE, idle)
            elif hint_tick % 30 == 1:
                send_calib_progress(AC_PHASE_WAIT, 0, len(samples), AC_SAMPLE_FRAMES,
                                    _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                    last_reason, idle)
            continue

        picked = max(candidates, key=lambda it: (
            _ac_collect_slot_priority(it[0], exclude_slots, allowed_slots), it[2]))
        _ac_draw_model_box(img, picked[8], ok=True)
        _ac_sample_det(img, picked[8], gnd_boxes, True)
        slot, label, score, x, y, w, h, s, det = picked
        target_slot = slot
        target_label = label
        if slot == 2:
            box = (x, y, w, h)
            if len(samples) <= 0:
                if red_bag_box_stable(red_bag_stable_box, box):
                    red_bag_stable_count += 1
                else:
                    red_bag_stable_count = 1
                red_bag_stable_box = box
                if red_bag_stable_count < RED_BAG_STABLE_FRAMES:
                    red_bag_stable_hint += 1
                    idle = 0
                    if red_bag_stable_hint % 8 == 1:
                        _clog("[det] red_bag wait stable {}/{} box=({},{},{},{})".format(
                            red_bag_stable_count, RED_BAG_STABLE_FRAMES, x, y, w, h))
                        send_calib_progress(AC_PHASE_SAMPLE, slot, len(samples), AC_SAMPLE_FRAMES,
                                            _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                            AC_REASON_STABLE, red_bag_stable_count)
                    continue
                red_bag_track_box = box
            elif not red_bag_box_stable(red_bag_track_box, box,
                                        RED_BAG_TRACK_CENTER_PX,
                                        RED_BAG_TRACK_SIZE_PX):
                red_bag_stable_hint += 1
                idle = 0
                if red_bag_stable_hint % 8 == 1:
                    _clog("[det] red_bag box jump, wait stable box=({},{},{},{}) base=({},{},{},{})".format(
                        x, y, w, h,
                        red_bag_track_box[0], red_bag_track_box[1],
                        red_bag_track_box[2], red_bag_track_box[3]))
                    send_calib_progress(AC_PHASE_SAMPLE, slot, len(samples), AC_SAMPLE_FRAMES,
                                        _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                        AC_REASON_STABLE, len(samples))
                continue
            red_bag_track_box = box
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
        elif slot == 4:
            box = (x, y, w, h)
            if brn_bear_box_stable(brn_bear_stable_box, box):
                brn_bear_stable_count += 1
            else:
                brn_bear_stable_count = 1
            brn_bear_stable_box = box
            if brn_bear_stable_count < BRN_BEAR_STABLE_FRAMES:
                brn_bear_stable_hint += 1
                idle = 0
                if brn_bear_stable_hint % 8 == 1:
                    _clog("[det] brn_bear wait stable {}/{} box=({},{},{},{})".format(
                        brn_bear_stable_count, BRN_BEAR_STABLE_FRAMES, x, y, w, h))
                    send_calib_progress(AC_PHASE_SAMPLE, slot, len(samples), AC_SAMPLE_FRAMES,
                                        _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                                        AC_REASON_STABLE, brn_bear_stable_count)
                continue
        else:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
        samples.append(s)
        idle = 0
        _clog("[det] {} score={:.2f} box=({},{},{},{}) med=({},{},{}) sample {}/{}".format(
            SLOT_NAMES.get(slot, slot), score, x, y, w, h,
            s[6], s[7], s[8], len(samples), AC_SAMPLE_FRAMES))
        send_calib_progress(AC_PHASE_SAMPLE, slot, len(samples), AC_SAMPLE_FRAMES,
                            _ac_slot_mask(exclude_slots), _ac_slot_mask(allowed_slots),
                            AC_REASON_NONE, 0)
    m = [_ac_median([s[i] for s in samples]) for i in range(9)]
    t = tighten_slot_threshold(target_slot,
                               _ac_build_threshold(m, ground, gnd_boxes[0], gnd_boxes, target_slot), m)
    return (target_slot, t, (m[6], m[7], m[8]), len(samples))


def _ac_split_threshold_conflicts(thresholds, slot_meds):
    slots = sorted(thresholds)
    for i in range(len(slots)):
        for j in range(i + 1, len(slots)):
            sa, sb = slots[i], slots[j]
            if sa not in thresholds or sb not in thresholds:
                continue
            t1, t2 = thresholds[sa], thresholds[sb]
            if not _ac_ab_overlap(t1, t2):
                continue
            if t1[1] < t2[0] or t2[1] < t1[0]:
                continue
            ma, mb = slot_meds[sa], slot_meds[sb]
            gaps = [abs(ma[k] - mb[k]) for k in range(3)]
            k = gaps.index(max(gaps))
            if gaps[k] < 8:
                _clog("[calib] !! {} <-> {} too close, drop both".format(
                    SLOT_NAMES[sa], SLOT_NAMES[sb]))
                del thresholds[sa]
                del thresholds[sb]
                continue
            mid = (ma[k] + mb[k]) / 2.0
            lo_i, hi_i = k * 2, k * 2 + 1
            lower, higher = (sa, sb) if ma[k] < mb[k] else (sb, sa)
            tl = list(thresholds[lower])
            th = list(thresholds[higher])
            tl[hi_i] = min(tl[hi_i], int(mid - 1))
            th[lo_i] = max(th[lo_i], int(mid + 1))
            if k == 0 and lower == 4 and higher == 5:
                tl[hi_i] = min(100, tl[hi_i] + BRN_BEAR_L_PAD_HI)
            thresholds[lower] = tuple(tl)
            thresholds[higher] = tuple(th)
            _clog("[calib] split {}<->{}: channel {} at {:.0f}".format(
                SLOT_NAMES[sa], SLOT_NAMES[sb], "LAB"[k], mid))
# ---- End IDE-style staged multi calibration helpers ----

def run_auto_calibration():
    global calib_file_exp, ground_box, ground_box_far
    print("=" * 40)
    _start_calib_log()
    _clog(">>> [0x04] auto multi calibration start <<<")
    if net is None:
        _clog("[calib] !! model not loaded, calibration disabled")
        send_calib_progress(AC_PHASE_FAIL, reason=AC_REASON_NO_MODEL)
        _flush_calib_log()
        send_calib_ack(0, 0, 0)
        return False

    send_calib_progress(AC_PHASE_EXP)
    exposure = _ac_calibrate_exposure()
    _clog("[calib] final exposure = {}us".format(exposure))
    send_calib_progress(AC_PHASE_EXP, arg=exposure)

    send_calib_progress(AC_PHASE_GND, arg=exposure)
    old_ground_box = ground_box
    old_ground_far = ground_box_far
    img = sensor.snapshot()
    exclude = []
    for d in _ac_detect_boxes(img):
        if d[1] >= 0.15:
            exclude.append(_ac_correct_box(d[2] - 6, d[3] - 6, d[4] + 12, d[5] + 12))

    ground, gbx = _make_gnd_box(img, AC_METER_ROI, exclude, "near")
    if ground is None:
        _clog("[calib] !! near ground sampling failed")
        ground_box = old_ground_box
        ground_box_far = old_ground_far
        send_calib_progress(AC_PHASE_FAIL, reason=AC_REASON_NO_GROUND, arg=exposure)
        _flush_calib_log()
        send_calib_ack(0, 0, exposure)
        return False
    ground_box = gbx

    for _ in range(3):
        update_cut_line(sensor.snapshot())
    if cut_valid:
        _clog("[calib] cut line left_y={} right_y={}".format(cut_left_y, cut_right_y))

    _, gbx_far = _make_gnd_box(img, AC_METER_ROI_FAR, exclude, "far")
    ground_box_far = gbx_far
    gnd_boxes = [ground_box] + ([ground_box_far] if ground_box_far else [])
    _clog("[calib] ground med={} near={} far={}".format(ground, ground_box, ground_box_far))

    thresholds = {}
    slot_meds = {}
    while len(thresholds) < AC_N_OBJECTS:
        done_slots = set(thresholds)
        allowed_slots = _ac_allowed_collect_slots(done_slots)
        if allowed_slots is None:
            break
        _clog(">>> wait: {} (done: {}) <<<".format(
            _ac_slot_list_names(allowed_slots), _ac_slot_list_names(tuple(sorted(done_slots)))))
        send_calib_progress(AC_PHASE_WAIT, sample=0, total=AC_SAMPLE_FRAMES,
                            done_mask=_ac_slot_mask(done_slots),
                            wait_mask=_ac_slot_mask(allowed_slots))
        r = _ac_collect_one(done_slots, allowed_slots, ground, gnd_boxes)
        if r is None:
            if thresholds:
                _clog(">>> {} frames without new object, finish sampling <<<".format(AC_SWAP_IDLE))
            else:
                _clog("[calib] !! no valid object sampled")
            break
        slot, t, med, n = r
        if _in_box(ground, t):
            _clog("[calib] !! {} threshold covers ground {}, drop slot".format(
                SLOT_NAMES.get(slot, slot), ground))
            send_calib_progress(AC_PHASE_THR, slot, n, AC_SAMPLE_FRAMES,
                                _ac_slot_mask(thresholds), _ac_slot_mask(allowed_slots),
                                AC_REASON_BAD_THR, slot)
            continue
        thresholds[slot] = t
        slot_meds[slot] = med
        _clog("[calib] {} ({} frames) = {} med={}".format(
            SLOT_NAMES.get(slot, slot), n, t, med))
        send_calib_progress(AC_PHASE_THR, slot, n, AC_SAMPLE_FRAMES,
                            _ac_slot_mask(thresholds), _ac_slot_mask(allowed_slots),
                            AC_REASON_NONE, slot)

    for sa in sorted(list(thresholds)):
        if sa not in thresholds:
            continue
        ta = thresholds[sa]
        for sb in sorted(slot_meds):
            if sa == sb:
                continue
            mb = slot_meds[sb]
            if (ta[0] <= mb[0] <= ta[1] and ta[2] <= mb[1] <= ta[3]
                    and ta[4] <= mb[2] <= ta[5]):
                _clog("[calib] !! {} threshold covers {} med {}, drop {}".format(
                    SLOT_NAMES[sa], SLOT_NAMES[sb], mb, SLOT_NAMES[sa]))
                if sa in thresholds:
                    del thresholds[sa]
                break

    _ac_split_threshold_conflicts(thresholds, slot_meds)

    ok = bool(thresholds)
    if not ok:
        _clog("[calib] !! no usable slot")

    if ok:
        try:
            send_calib_progress(AC_PHASE_SAVE, sample=len(thresholds), total=AC_N_OBJECTS,
                                done_mask=_ac_slot_mask(thresholds), arg=exposure)
            with open(CALIB_FILE, "w") as fp:
                fp.write("exposure_us={}\n".format(exposure))
                fp.write("ground={},{},{},{},{},{}\n".format(
                    int(ground_box[0]), int(ground_box[1]), int(ground_box[2]),
                    int(ground_box[3]), int(ground_box[4]), int(ground_box[5])))
                if ground_box_far:
                    fp.write("ground2={},{},{},{},{},{}\n".format(
                        int(ground_box_far[0]), int(ground_box_far[1]), int(ground_box_far[2]),
                        int(ground_box_far[3]), int(ground_box_far[4]), int(ground_box_far[5])))
                for slot in sorted(thresholds):
                    t = thresholds[slot]
                    fp.write("{},{},{},{},{},{},{}\n".format(
                        slot, t[0], t[1], t[2], t[3], t[4], t[5]))
            calibrated_slots.clear()
            for i in range(len(DEFAULT_THRESHOLDS)):
                all_color_thresholds[i] = DEFAULT_THRESHOLDS[i]
            for slot in sorted(thresholds):
                all_color_thresholds[slot - 1] = thresholds[slot]
                calibrated_slots.add(slot)
            calib_file_exp = exposure
            _clog(">>> [calib] success, wrote {} slots={} <<<".format(
                CALIB_FILE, sorted(thresholds)))
        except Exception as e:
            _clog("[calib] !! write failed:", e)
            ok = False

    if not ok:
        _clog(">>> [calib] failed, keep old parameters <<<")
        old_exp = load_calibrated_params()
        if old_exp:
            sensor.set_auto_exposure(False, exposure_us=old_exp)
        send_calib_progress(AC_PHASE_FAIL, sample=len(thresholds), total=AC_N_OBJECTS,
                            done_mask=_ac_slot_mask(thresholds), reason=AC_REASON_BAD_THR,
                            arg=exposure)
    else:
        send_calib_progress(AC_PHASE_DONE, sample=len(thresholds), total=AC_N_OBJECTS,
                            done_mask=_ac_slot_mask(thresholds), arg=exposure)

    _flush_calib_log()
    for slot in sorted(thresholds):
        send_calib_slot(slot, thresholds[slot])
    send_calib_ack(1 if ok else 0, len(thresholds), exposure)
    return ok

def reset_calibration():
    """0x05: 删标定文件 + 恢复代码默认阈值/曝光/状态, 回执 C4(status=1,count=0)。
    用途: 换场地重标前清掉旧槽位, 让下一次 0x04 从默认阈值和默认曝光开始。"""
    global ground_box, ground_box_far, cut_valid, calib_file_exp
    import os
    try:
        os.remove(CALIB_FILE)
    except Exception:
        pass
    calib_file_exp = None
    for i in range(len(DEFAULT_THRESHOLDS)):
        all_color_thresholds[i] = DEFAULT_THRESHOLDS[i]
    calibrated_slots.clear()
    ground_box = None
    ground_box_far = None
    cut_valid = False
    sensor.set_auto_exposure(False, exposure_us=DEFAULT_EXPOSURE_US)
    print(">>> [0x05] 标定已清空, 恢复默认 <<<")
    send_calib_ack(1, 0, DEFAULT_EXPOSURE_US)

# ======================================================================
# 主控命令解析 (0x01/0x02/0x03/0x04/0x05)
# ======================================================================
_cmd_rx_buf = bytearray()

def receive_command_from_host():
    global _cmd_rx_buf, target_color_id
    chunk = uart_read_all()
    if chunk:
        _cmd_rx_buf.extend(chunk)
    if len(_cmd_rx_buf) > 64:
        _cmd_rx_buf = _cmd_rx_buf[-32:]

    while len(_cmd_rx_buf) >= 4:
        # 手写找帧头: OpenART 固件的 bytearray 不一定支持 .find (生产代码同款写法)
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
        # 丢弃被回环的 12 字节上行帧
        if len(_cmd_rx_buf) >= 12:
            if (sum(_cmd_rx_buf[2:11]) & 0xFF) == _cmd_rx_buf[11]:
                _cmd_rx_buf = _cmd_rx_buf[12:]
                continue
        command = _cmd_rx_buf[2]
        if command not in (0x01, 0x02, 0x03, 0x04, 0x05):
            _cmd_rx_buf = _cmd_rx_buf[2:]
            continue
        if command == 0x03:
            if len(_cmd_rx_buf) < 5:
                return (0, 0)
            b3, b4 = _cmd_rx_buf[3], _cmd_rx_buf[4]
            if ((command + b3) & 0xFF) == b4:
                param = b3
            elif ((command + b4) & 0xFF) == b3:
                param = b4
            else:
                _cmd_rx_buf = _cmd_rx_buf[2:]
                continue
            _cmd_rx_buf = _cmd_rx_buf[5:]
        else:
            if _cmd_rx_buf[3] != command:
                _cmd_rx_buf = _cmd_rx_buf[2:]
                continue
            param = 0
            _cmd_rx_buf = _cmd_rx_buf[4:]

        if command == 0x03:
            if 1 <= param <= len(all_color_thresholds):
                target_color_id = param
                print(">>> lock color {} <<<".format(param))
        elif command == 0x02:
            target_color_id = 0
            print(">>> reset <<<")
        elif command == 0x01:
            print(">>> carry mode (test: no-op) <<<")
        elif command == 0x04:
            run_auto_calibration()
            target_color_id = 0
        elif command == 0x05:
            # 清空标定: 删文件+恢复默认阈值/曝光 (比赛现场换场地时用, 不必插USB)
            reset_calibration()
            target_color_id = 0
        return (command, param)
    return (0, 0)

# ======================================================================
# 主循环: 纯色块检测
# ======================================================================
print("=" * 50)
print("[TEST] 纯色块 + 自动标定")
print("阈值:", all_color_thresholds)
print("命令: 0x01搬运 0x02重置 0x03锁色 0x04自动标定")
print("=" * 50)

clock = time.clock()
frame_count = 0

# ---- 追踪保持滞回(生产 COLOR_TRACK_KEEP_PIXELS 思路) ----
# 建立目标用 COLOR_MIN_PIXELS; 已在追踪的目标降级到 KEEP 门槛,
# 在上次位置附近用宽松形状规则找回, 避免暗光下 blob 破碎导致一帧丢一帧有
KEEP_PIXELS = 20
KEEP_MISS_MAX = 10          # 连续丢这么多帧才真正放弃
KEEP_ROI_MARGIN = 60
last_cid = 0
last_rect = None
miss_count = 0

# 诊断计数(每 30 帧打印): raw=原始数, shape_rej=形状淘汰,
# ground_rej=判为地面淘汰, off_rej=中心色度不在本槽阈值盒淘汰
dbg_raw = 0
dbg_shape_rej = 0
dbg_ground_rej = 0
dbg_off_rej = 0
dbg_size_rej = 0

while True:
    clock.tick()
    frame_count += 1
    img = sensor.snapshot()

    try:
        receive_command_from_host()
    except Exception as e:
        print("!! cmd err:", e)
        _cmd_rx_buf = bytearray()

    if frame_count % 2 == 0:
        try:
            update_cut_line(img)
        except Exception as e:
            print("!! cut err:", e)
    det_roi = dynamic_detect_roi()   # 生产同款: 分界线以上(赛道外)整体不检测

    # 检测: 锁色时只查该色; 否则只查标定过的槽位(没标过的默认阈值不可信),
    # 完全没有标定文件时才查全部 5 色
    if target_color_id > 0:
        check_ids = (target_color_id,)
    elif calibrated_slots:
        check_ids = tuple(sorted(calibrated_slots))
    else:
        check_ids = (1, 2, 3, 4, 5)

    best = None       # (color_id, blob)
    candidates = []
    dbg_raw = 0
    dbg_shape_rej = 0
    dbg_ground_rej = 0
    dbg_off_rej = 0
    dbg_size_rej = 0
    for cid in check_ids:
        blobs = find_slot_blobs(img, all_color_thresholds[cid - 1], det_roi,
                                COLOR_MIN_PIXELS, COLOR_MIN_AREA, cid)
        if not blobs:
            continue
        dbg_raw += len(blobs)
        for b in blobs:
            if cid == 1:
                b = refine_blue_bag_blob(img, b, all_color_thresholds[cid - 1])
                if b is None:
                    dbg_ground_rej += 1
                    continue
            if not valid_color_blob(b, cid):
                dbg_shape_rej += 1
                continue
            rej = blob_reject_reason(img, b, cid)
            if rej is not None:
                if rej == 'gnd':
                    dbg_ground_rej += 1
                elif rej == 'off':
                    dbg_off_rej += 1
                else:
                    dbg_size_rej += 1
                continue
            candidates.append((cid, b))
    if candidates:
        if last_rect is not None:
            # 生产 find_color_target 同款打分: 像素数 - 与上帧目标的中心距离惩罚,
            # 空间连续性优先, 防止多个候选间跳目标
            lx = last_rect[0] + last_rect[2] // 2
            ly = last_rect[1] + last_rect[3] // 2
            def _score(item):
                b = item[1]
                dx = b.cx() - lx
                dy = b.cy() - ly
                return b.pixels() - (dx * dx + dy * dy) // 20
            best = max(candidates, key=_score)
        else:
            best = max(candidates, key=lambda item: item[1].pixels())

    # 滞回找回: 常规检测丢了, 但上一帧还在追踪 → 在旧位置附近降门槛重找
    if best is None and last_cid > 0 and last_rect and miss_count < KEEP_MISS_MAX:
        rx = clamp_int(last_rect[0] - KEEP_ROI_MARGIN, 0, 319)
        ry = clamp_int(last_rect[1] - KEEP_ROI_MARGIN, det_roi[1], 239)
        rw = clamp_int(last_rect[2] + KEEP_ROI_MARGIN * 2, 1, 320 - rx)
        rh = clamp_int(last_rect[3] + KEEP_ROI_MARGIN * 2, 1, 240 - ry)
        blobs = find_slot_blobs(img, all_color_thresholds[last_cid - 1],
                                (rx, ry, rw, rh), KEEP_PIXELS, KEEP_PIXELS, last_cid)
        if blobs:
            b = max(blobs, key=lambda x: x.pixels())
            if last_cid == 1:
                b = refine_blue_bag_blob(img, b, all_color_thresholds[last_cid - 1])
            # 保持期不查长宽比(破碎blob比例失真), 但宽度上限和色度复核必须保留,
            # 否则赛道布被抓到一次后会被此通道无限找回(id 永不归 0)
            if (b is not None and b.density() >= 0.25 and b.w() < 280
                    and blob_reject_reason(img, b, last_cid) is None):
                best = (last_cid, b)

    if best:
        last_cid = best[0]
        last_rect = best[1].rect()
        miss_count = 0
    else:
        miss_count += 1
        if miss_count >= KEEP_MISS_MAX:
            last_cid = 0
            last_rect = None

    # 分界线可视化(接 IDE 时可见): 红斜线=切割线, 线以上(赛道外)不参与检测
    if cut_valid:
        try:
            img.draw_line(0, cut_left_y, 319, cut_left_y, color=(255, 0, 0))
        except Exception:
            pass

    try:
        if best:
            cid, b = best
            wx, wy = box_to_world(b.x(), b.y(), b.w(), b.h())
            send_world_data(cid, world_cm_to_uart_units(wx),
                            world_cm_to_uart_units(wy), b.w())
            img.draw_rectangle(b.rect(), color=DRAW_COLORS.get(cid, (255, 255, 255)))
            img.draw_string(b.x(), max(0, b.y() - 10), SLOT_NAMES.get(cid, "?"),
                            color=DRAW_COLORS.get(cid, (255, 255, 255)))
            img.draw_cross(b.cx(), b.cy(), color=(255, 0, 0))
        else:
            send_world_data(0, 0, 0, 0)
    except Exception as e:
        print("!! tx err:", e)

    if frame_count % 30 == 0:
        if best:
            _lab = _blob_center_stats(img, best[1])
            print("fps={:.1f} {} px={} den={:.2f} w/h={:.2f} LAB={} gnd={} world=({:.1f},{:.1f}) lock={}".format(
                clock.fps(), SLOT_NAMES.get(best[0], "?"), best[1].pixels(),
                best[1].density(), best[1].w() / max(best[1].h(), 1),
                _lab, ground_box, wx, wy, target_color_id))
        else:
            # raw>0 而 shape_rej>0 = 阈值能抓到但形状规则杀了(规则过严);
            # raw=0 = 阈值本身抓不到(标定不准/太窄)
            print("fps={:.1f} no target raw={} shape={} gnd={} off={} size={} miss={} lock={}".format(
                clock.fps(), dbg_raw, dbg_shape_rej, dbg_ground_rej, dbg_off_rej,
                dbg_size_rej, miss_count, target_color_id))
