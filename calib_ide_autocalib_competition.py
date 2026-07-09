# ======================================================================
# calib_ide_tune.py — OpenMV/OpenART IDE 标定调参脚本 (单物体流程 v2)
# ======================================================================
# 流程: 点运行 → ① 自动曝光(打印迭代过程)
#              → ② 采样阶段: 实时显示模型框(黄框+label+分数), 按分数尝试
#                 同帧多个模型框, 按阶段锁定一个未完成槽位采够 SAMPLE_FRAMES 帧
#              → ③ 大横幅输出该物体的阈值
#              → ④ 预览阶段: 色块检测画框(彩色) + 模型框对照(黄色),
#                 VIEW_MASK_SLOT 可切二值 mask 视图
# 用法: 一次只摆一个物体在画面里, 标完换物体重跑(Ctrl+R)。
# 调好后把改过的参数抄回 main_autocalib_test.py 同名常量。
# ======================================================================

import sensor, image, time, tf

SCRIPT_TITLE = "IDE auto-calib competition preview"
SCRIPT_TARGET = "xuezhang/shijue/main_autocalib_competition.py"
SCRIPT_RUNTIME = "auto_calib tuned runtime filters"
RUN_PROFILE = "debug"      # "debug"=full draw/model/cmp; "run"=minimal preview for FPS
RUN_DEBUG = (RUN_PROFILE == "debug")
CALIB_DRAW_DEBUG = RUN_DEBUG
PREVIEW_DRAW_DEBUG = RUN_DEBUG
PREVIEW_SHOW_COLOR_BOX = True
PREVIEW_SHOW_CUT_LINE = True
PREVIEW_CMP_DEBUG = RUN_DEBUG

# ================= ① 曝光参数 =================
L_TARGET        = 40.0    # 测光目标 L 均值(宁欠勿过, 过曝会压掉色度)
L_TOL           = 5.0     # 收敛容差
STEP_MIN        = 0.80    # 单轮曝光调整下限
STEP_MAX        = 1.1    # 单轮曝光调整上限
EXPOSURE_INIT   = 1200
EXPOSURE_MIN    = 100
EXPOSURE_MAX    = 4500
METER_FULL      = True    # True=全画面测光; False=用 METER_ROI 测光
METER_ROI       = (0, 140, 320, 100)  # 地面采样区(也是地面盒取样区):
                                       # 左右各留10px, 直到画面最底(y140~240),
                                       # 覆盖更宽的布面, 地面盒更能代表全场
METER_AVG       = 4       # 每轮测光平均帧数
HILIGHT_LUQ     = 96      # 高光防炸: 全画面 P75 亮度超过它就压曝光 20%

# ================= ② 模型取样参数 =================
DET_SCORE       = 0.20    # 模型置信度门槛
SAMPLE_FRAMES   = 10      # 需要采够的合格样本帧数
SAMPLE_ROI_W_FRAC = 0.80  # 取样矩形宽 = 修正框宽 * 该系数
SAMPLE_ROI_H_FRAC = 0.85  # 取样矩形高 = 修正框高 * 该系数
BEAR_GND_RETRY_ROI_W_FRAC = 0.55
BEAR_GND_RETRY_ROI_H_FRAC = 0.30
BEAR_GND_RETRY_Y_FRAC = 0.25
MIN_BOX_W       = 10      # 模型框最小边(px)
COLLECT_TIMEOUT = 300     # 采样阶段最多跑多少帧(采不够就放弃并提示)
BOX_EXPAND      = 1.25    # 模型框偏小: 宽度绕中心放大倍数(调到青框贴合物体)
BOX_EXPAND_H    = 1.6     # 高度放大倍数
BOX_OFF_X       = -1      # 模型框略偏右: 向左平移修正(px, 正数=向右)
BOX_OFF_Y       = 0       # 垂直方向平移修正(px, 正数=向下)

# ================= ③ 阈值生成参数 =================
IQR_K           = 0.8     # 四分位外扩系数(大=松, 小=紧)
AB_MARGIN       = 5       # A/B 额外余量
AB_SPAN_MAX     = 45      # A/B 盒跨度硬上限
AB_SPAN_MAX_BALL = 75     # 网球底部暗边+高光会拉开 A/B, 只对 ball 放宽
AB_SPAN_MAX_BAG = 60      # 沙袋表面/边缘阴影有方向性, 但仍比网球保守
L_SPAN_MAX      = 60      # 保留实测 L 时的跨度上限
L_WIDE          = (15, 100)  # 高饱和色 L 放宽范围
NEUTRAL_AB      = 12      # A/B 都离 0 近于此 = 低饱和色, 保留实测 L
L_MARGIN_LOWSAT = 8       # 低饱和色 L 余量
GND_NEAR        = 10      # 槽位盒与地面盒 A/B 接近判定余量
GND_CUT_W       = 0.35    # 地面冲突时 L 切割点(大=防布误报, 小=保物体完整)
RED_BAG_GND_L_PAD_LO = 4  # 红沙袋地面冲突后只小幅放宽 L, 避免框只剩核心小块
RED_BAG_GND_L_PAD_HI = 8
IQR_L_MAX       = 80      # 样本质量门: L 四分位距上限(基本只拦极端病态样本;
IQR_AB_MAX      = 100      # 阈值质量由下游把关: 跨度硬上限/地面盒包含/互吃检查)
SAMPLE_L_CLIP   = 98      # 样本中心 L 中位数≥它=过曝, 丢样本
BLUE_BAG_GND_B_MAX = -25
BLUE_BAG_B_MARGIN_HI = 12
BLUE_BAG_A_CUT_MIN_GAP = 8
BLUE_BAG_A_CUT_GAP = 6
BAG_EDGE_AB_MAX_DELTA = 35
BAG_EDGE_AB_MAX_DELTA_BLUE = 22
BAG_EDGE_L_PAD = 8
BAG_EDGE_AB_PAD = 10
BAG_EDGE_L_SPAN_MAX = 70
BAG_EDGE_AB_SPAN_MAX = 90

# ================= ④ 地面盒参数 =================
GB_L_M          = 12      # 地面盒 L 外扩
GB_AB_M         = 8       # 地面盒 A/B 外扩
METER_ROI_FAR   = (0, 15, 320, 45)   # 远处地面采样带: 掠射角下布的色度漂移大,
                                     # 只采近处会漏掉远处布(曾误报为蓝包)。
                                     # 标定时该区域必须也是布, 别让物体/场外杂物进去
IQR_AB_MAX_BALL = 90      # 网球单独放宽: 高光+绿边+底部暗边合并取样, 离散天然大

# ================= ⑤ 标定模式 =================
CALIB_MODE      = "multi"  # "single"=只标一个物体就进预览;
                           # "multi"=物体一个个轮流标(标完一个换下一个, 自动识别新物体),
                           #          全部标完(或超时)再进预览
N_OBJECTS       = 5        # multi 模式最多标几个物体
CALIB_SLOT_PHASES = ((1, 2), (3,), (4, 5))  # 沙袋任意顺序 -> 网球 -> 熊; 空元组=完全自动
SWAP_IDLE       = 450      # multi 模式: 连续这么多帧没等到新物体的合格样本就结束(~15s)
SWAP_CLEAR_FRAMES = 8     # 已完成物体离场后再等几帧, 防换物体过程混采

# ================= ⑥ 输出/预览 =================
WRITE_FILE      = True   # True=结果写 /sd/color_thr.txt
VIEW_MASK_SLOT  = 0       # 0=正常预览; 1~5=该槽位二值 mask 视图
PREVIEW_STRICT  = True    # True=每槽只画最大合格blob(近似车载); False=画全部原始blob
PREVIEW_MODEL   = RUN_DEBUG  # debug 预览跑模型对照; run 预览纯色块, 不调用模型
MODEL_EVERY     = 10      # 预览阶段模型检测的帧间隔(模型慢, 不必每帧跑)
CMP_CENTER_OK_PX = 35
DETECT_Y_MIN = 8
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)
if not RUN_DEBUG:
    VIEW_MASK_SLOT = 0
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
WB_GAINS        = (101.00, 64.00, 97.00)
MODEL_PATH      = '/sd/dataset_25000_exposure.tflite'
# =====================================================

SLOT_NAMES = {1: "blue_bag", 2: "red_bag", 3: "ball", 4: "brn_bear", 5: "wht_bear"}
LABEL_NAMES = {0: "bear", 1: "ball", 2: "bag"}
DRAW_COLORS = {1: (0, 170, 255), 2: (255, 0, 0), 3: (0, 255, 0),
               4: (255, 180, 0), 5: (255, 255, 255)}
MODEL_BOX_COLOR = (255, 255, 0)   # 模型框统一黄色

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_INIT)
sensor.skip_frames(time=800)
sensor.set_vflip(True)

net = tf.load(MODEL_PATH)

print("======== %s ========" % SCRIPT_TITLE)
print("target: %s" % SCRIPT_TARGET)
print("preview runtime: %s" % SCRIPT_RUNTIME)

def clamp_int(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def median(lst):
    s = sorted(lst)
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) * 0.5

def in_box(lab, box, m=0):
    return (box[0] - m <= lab[0] <= box[1] + m and box[2] - m <= lab[1] <= box[3] + m
            and box[4] - m <= lab[2] <= box[5] + m)

def blue_bag_side_gutter_blob(b, slot):
    return (slot == 1 and b.y() >= BLUE_BAG_SIDE_GUTTER_Y_MIN
            and (b.x() < BLUE_BAG_SIDE_GUTTER_X_MARGIN
                 or b.x() + b.w() > 320 - BLUE_BAG_SIDE_GUTTER_X_MARGIN))

def blue_bag_bottom_shadow_blob(b, slot):
    return (slot == 1 and b.y() >= BLUE_BAG_BOTTOM_SHADOW_Y_MIN
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

def bear_small_min_h_for_slot(slot):
    return BRN_BEAR_SMALL_MIN_H if slot == 4 else BEAR_SMALL_MIN_H

def bear_shape_skip_w_for_slot(slot):
    return BRN_BEAR_BLOB_SHAPE_SKIP_W if slot == 4 else BEAR_BLOB_SHAPE_SKIP_W

def detect_boxes(img):
    """返回 [(label, score, x, y, w, h)], 不做任何过滤(过滤交给调用方)。"""
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

# ---- 模型分段阈值(生产 main.py 同款): 近距离要求高分, 远距离目标小允许低分 ----
MODEL_NEAR_CY = 130
MODEL_NEAR_W = 55
MODEL_MID_CY = 90
MODEL_MID_W = 28
MODEL_FALLBACK_NEAR = 0.50
MODEL_FALLBACK_MID = 0.32
MODEL_FALLBACK_FAR = 0.12

def model_score_floor(x, y, w, h):
    """该框位置/大小对应的最低可接受分数(生产 fallback 档)。"""
    cy = y + h // 2
    if cy >= MODEL_NEAR_CY or w >= MODEL_NEAR_W:
        return MODEL_FALLBACK_NEAR
    if cy >= MODEL_MID_CY or w >= MODEL_MID_W:
        return MODEL_FALLBACK_MID
    return MODEL_FALLBACK_FAR

def correct_box(x, y, w, h):
    """模型框修正: 绕中心放大 BOX_EXPAND 倍, 平移 (BOX_OFF_X, BOX_OFF_Y)。
    取样和位置对比都用修正后的框。"""
    cx = x + w / 2.0 + BOX_OFF_X
    cy = y + h / 2.0 + BOX_OFF_Y
    nw = w * BOX_EXPAND
    nh = h * BOX_EXPAND_H
    nx = clamp_int(int(cx - nw / 2), 0, 319)
    ny = clamp_int(int(cy - nh / 2), 0, 239)
    return (nx, ny, clamp_int(int(nw), 1, 320 - nx), clamp_int(int(nh), 1, 240 - ny))

def box_to_world(x, y, w, h):
    if H_pix2world is None:
        return (0.0, 0.0)
    corners = ((x, y), (x + w, y), (x, y + h), (x + w, y + h))
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
    return [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1.0]]

def pixel_to_world(px, py, H):
    px = float(px)
    py = float(py)
    ww = H[2][0] * px + H[2][1] * py + H[2][2]
    if abs(ww) < 1e-10:
        return (0.0, 0.0)
    X = (H[0][0] * px + H[0][1] * py + H[0][2]) / ww
    Y = (H[1][0] * px + H[1][1] * py + H[1][2]) / ww
    return (X, Y)

def world_to_pixel(X, Y, H):
    ww = H[2][0] * X + H[2][1] * Y + H[2][2]
    if abs(ww) < 1e-10:
        return (-1, -1)
    u = (H[0][0] * X + H[0][1] * Y + H[0][2]) / ww
    v = (H[1][0] * X + H[1][1] * Y + H[1][2]) / ww
    return (int(u), int(v))

CALIB_PIXEL = [
    [90, 240],
    [236, 240],
    [121, 149],
    [210, 149],
]

CALIB_WORLD = [
    [-8, 6],
    [7, 6],
    [-8, 21],
    [8, 21],
]

H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)
H_world2pix = calc_homography(CALIB_WORLD, CALIB_PIXEL)

TARGET_REAL_WIDTH = [70.0, 70.0, 67.0, 120.0, 120.0]
FOCAL_LENGTH = 167.5
MIN_DETECT_DISTANCE = 50
MAX_DETECT_DISTANCE = 2000
TARGET_REAL_W_MM = {1: 70.0, 2: 70.0, 3: 67.0, 4: 120.0, 5: 120.0}
SIZE_RATIO_MAX = 2.2
SIZE_RATIO_MIN = 0.30

def calculate_distance(pixel_width, color_id=1):
    if pixel_width <= 0:
        return -1
    real_width = TARGET_REAL_WIDTH[color_id - 1] if 1 <= color_id <= len(TARGET_REAL_WIDTH) else 70.0
    distance = (real_width * FOCAL_LENGTH) / pixel_width
    if distance < MIN_DETECT_DISTANCE or distance > MAX_DETECT_DISTANCE:
        return -1
    return int(distance)

def blob_size_ratio(b, slot):
    _, wy = box_to_world(b.x(), b.y(), b.w(), b.h())
    dist_mm = wy * 10.0
    if dist_mm < 80.0:
        return 1.0, dist_mm
    exp_w = TARGET_REAL_W_MM.get(slot, 70.0) * FOCAL_LENGTH / dist_mm
    return b.w() / exp_w, dist_mm

def blob_size_plausible(b, slot):
    ratio, _ = blob_size_ratio(b, slot)
    return SIZE_RATIO_MIN <= ratio <= SIZE_RATIO_MAX

def world_info_for_blob(slot, b):
    wx, wy = box_to_world(b.x(), b.y(), b.w(), b.h())
    dist = calculate_distance(b.w(), slot)
    return wx, wy, dist

def format_world_info(slot, b):
    wx, wy, dist = world_info_for_blob(slot, b)
    return "world=({:.1f},{:.1f})cm dist={}mm".format(wx, wy, dist)

def draw_model_box(img, det, ok=True):
    if not CALIB_DRAW_DEBUG:
        return
    label, score, x, y, w, h = det
    img.draw_rectangle(x, y, w, h, color=MODEL_BOX_COLOR)
    img.draw_string(x, max(0, y - 10), "%s %.2f" % (LABEL_NAMES.get(label, "?"), score),
                    color=MODEL_BOX_COLOR)
    img.draw_rectangle(correct_box(x, y, w, h), color=(0, 255, 255))  # 青框=修正后
    if not ok:
        img.draw_line(x, y, x + w, y + h, color=(255, 0, 0))  # 被丢弃的框画红叉线

# ---------- ① 曝光 ----------
def calibrate_exposure():
    print("")
    print("======== ① 自动曝光 ========")
    exp = EXPOSURE_INIT
    sensor.set_auto_exposure(False, exposure_us=exp)
    for it in range(10):
        time.sleep_ms(150)
        for _ in range(3):
            sensor.snapshot()
        l_sum = 0.0
        luq = 0
        for _ in range(METER_AVG):
            img = sensor.snapshot()
            st = img.get_statistics() if METER_FULL else img.get_statistics(roi=METER_ROI)
            l_sum += st.l_mean()
            luq = img.get_statistics().l_uq()
        l = l_sum / METER_AVG
        print("[exp] iter %d: %dus L=%.1f Luq=%d" % (it, exp, l, luq))
        if abs(l - L_TARGET) <= L_TOL:
            break
        ratio = L_TARGET / max(l, 1.0)
        ratio = min(max(ratio, STEP_MIN), STEP_MAX)
        exp = clamp_int(int(exp * ratio), EXPOSURE_MIN, EXPOSURE_MAX)
        sensor.set_auto_exposure(False, exposure_us=exp)
    for it in range(5):
        for _ in range(3):
            img = sensor.snapshot()
        luq = img.get_statistics().l_uq()
        if luq < HILIGHT_LUQ:
            break
        exp = max(int(exp * 0.8), EXPOSURE_MIN)
        print("[exp] 高光饱和(Luq=%d), 压曝光 -> %dus" % (luq, exp))
        sensor.set_auto_exposure(False, exposure_us=exp)
        time.sleep_ms(150)
    print("[exp] FINAL = %dus" % exp)
    return exp

# ---------- 阈值生成 ----------
def cut_blue_bag_ground_by_a(a0, a1, a_med, ground_boxes):
    cut = False
    for g in ground_boxes:
        ga0 = g[2]
        if a_med <= ga0 - BLUE_BAG_A_CUT_MIN_GAP:
            na1 = ga0 - BLUE_BAG_A_CUT_GAP
            if na1 >= a0:
                a1 = min(a1, na1)
                cut = True
    return a0, a1, cut

def build_threshold(m, ground, gnd_box, gnd_boxes_for_conflict=None, slot=0):
    llq, luq, alq, auq, blq, buq = m[0], m[1], m[2], m[3], m[4], m[5]
    l_med, a_med, b_med = m[6], m[7], m[8]
    l0 = llq - IQR_K * (luq - llq)
    l1 = luq + IQR_K * (luq - llq)
    a0 = alq - IQR_K * (auq - alq) - AB_MARGIN
    a1 = auq + IQR_K * (auq - alq) + AB_MARGIN
    b0 = blq - IQR_K * (buq - blq) - AB_MARGIN
    b1 = buq + IQR_K * (buq - blq) + AB_MARGIN
    def axis_dist(lo, hi):
        return 0.0 if lo <= 0 <= hi else min(abs(lo), abs(hi))
    low_sat = (axis_dist(a0, a1) < NEUTRAL_AB and axis_dist(b0, b1) < NEUTRAL_AB)
    ground_conflict_ls = []
    ground_conflict_boxes = []
    if slot != 3:
        for g in (gnd_boxes_for_conflict or ([gnd_box] if gnd_box else [])):
            if not g:
                continue
            ga0, ga1, gb0, gb1 = g[2], g[3], g[4], g[5]
            a_touch = not (a1 + GND_NEAR < ga0 or ga1 + GND_NEAR < a0)
            b_touch = not (b1 + GND_NEAR < gb0 or gb1 + GND_NEAR < b0)
            if a_touch and b_touch:
                ground_conflict_ls.append((g[0] + g[1]) * 0.5)
                ground_conflict_boxes.append(g)
    ground_in_ab = bool(ground_conflict_ls)
    blue_bag_a_cut = False
    if slot == 1 and ground_conflict_boxes:
        a0, a1, blue_bag_a_cut = cut_blue_bag_ground_by_a(
            a0, a1, a_med, ground_conflict_boxes)
        if blue_bag_a_cut:
            print("[thr] blue_bag A切地面, L放宽到 (%d, %d), A=(%d,%d)" % (
                L_WIDE[0], L_WIDE[1], int(a0), int(a1)))
    if slot == 3 or blue_bag_a_cut:
        l0, l1 = L_WIDE
    elif low_sat or ground_in_ab:
        l0 = max(0, l0 - L_MARGIN_LOWSAT)
        l1 = min(100, l1 + L_MARGIN_LOWSAT)
        if ground_in_ab:
            for gl in ground_conflict_ls:
                if gl < l_med:
                    l0 = max(l0, gl + GND_CUT_W * (l_med - gl))
                else:
                    l1 = min(l1, gl - GND_CUT_W * (gl - l_med))
            if slot == 2:
                l0 = max(0, l0 - RED_BAG_GND_L_PAD_LO)
                l1 = min(100, l1 + RED_BAG_GND_L_PAD_HI)
            print("[thr] 地面冲突, L 收紧到 (%d, %d)" % (int(l0), int(l1)))
    else:
        l0, l1 = L_WIDE
    if slot == 3:
        ab_span_max = AB_SPAN_MAX_BALL
    elif slot in (1, 2):
        ab_span_max = AB_SPAN_MAX_BAG
    else:
        ab_span_max = AB_SPAN_MAX
    if a1 - a0 > ab_span_max:
        a0 = max(a0, a_med - ab_span_max / 2)
        a1 = min(a1, a_med + ab_span_max / 2)
    if b1 - b0 > ab_span_max:
        b0 = max(b0, b_med - ab_span_max / 2)
        b1 = min(b1, b_med + ab_span_max / 2)
    if l1 - l0 > L_SPAN_MAX and (l0, l1) != L_WIDE:
        l0 = max(l0, l_med - L_SPAN_MAX / 2)
        l1 = min(l1, l_med + L_SPAN_MAX / 2)
    return (int(l0), int(l1), int(a0), int(a1), int(b0), int(b1))

def tighten_slot_threshold(slot, t, m):
    if slot == 1:
        b1 = min(t[5], int(m[8] + BLUE_BAG_B_MARGIN_HI))
        t = (t[0], t[1], t[2], t[3], t[4], max(t[4], b1))
    return t

def allow_ground_sample(label, s):
    # 只给蓝沙袋采样放过地面盒, 避免蓝包 B 通道接近赛道布时被误杀。
    return label == 2 and s[8] <= BLUE_BAG_GND_B_MAX

def merge_stat_tuple(s, st):
    s[0] = min(s[0], st.l_lq())
    s[1] = max(s[1], st.l_uq())
    s[2] = min(s[2], st.a_lq())
    s[3] = max(s[3], st.a_uq())
    s[4] = min(s[4], st.b_lq())
    s[5] = max(s[5], st.b_uq())

def try_merge_bag_edge_stat(s, st):
    # 边带可能混入少量地面/背景, 只取中位数附近小窗口, 不并入整块 LQ/UQ。
    lm, am, bm = st.l_median(), st.a_median(), st.b_median()
    cand = [s[i] for i in range(6)]
    cand[0] = min(cand[0], max(0, lm - BAG_EDGE_L_PAD))
    cand[1] = max(cand[1], min(100, lm + BAG_EDGE_L_PAD))
    cand[2] = min(cand[2], max(-128, am - BAG_EDGE_AB_PAD))
    cand[3] = max(cand[3], min(127, am + BAG_EDGE_AB_PAD))
    cand[4] = min(cand[4], max(-128, bm - BAG_EDGE_AB_PAD))
    cand[5] = max(cand[5], min(127, bm + BAG_EDGE_AB_PAD))
    if (cand[1] - cand[0] > BAG_EDGE_L_SPAN_MAX
            or cand[3] - cand[2] > BAG_EDGE_AB_SPAN_MAX
            or cand[5] - cand[4] > BAG_EDGE_AB_SPAN_MAX):
        return False
    for i in range(6):
        s[i] = cand[i]
    return True

def merge_bag_edge_strips(img, s, roi, gnd_boxes_for_edge=None, draw_debug=True):
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
    max_delta = BAG_EDGE_AB_MAX_DELTA_BLUE if is_blue_bag else BAG_EDGE_AB_MAX_DELTA
    for eroi in rois:
        st = img.get_statistics(roi=eroi)
        lab = (st.l_median(), st.a_median(), st.b_median())
        if is_blue_bag and gnd_boxes_for_edge:
            if any(in_box(lab, g) for g in gnd_boxes_for_edge):
                continue
        if (abs(lab[1] - s[7]) > max_delta
                or abs(lab[2] - s[8]) > max_delta):
            continue
        if try_merge_bag_edge_stat(s, st) and CALIB_DRAW_DEBUG and draw_debug:
            img.draw_rectangle(eroi, color=(0, 180, 0))

def assign_slot(label, m):
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

def slot_list_names(slots):
    return "/".join(SLOT_NAMES.get(s, str(s)) for s in slots)

def allowed_collect_slots(done_slots):
    if not CALIB_SLOT_PHASES:
        return None
    for phase in CALIB_SLOT_PHASES:
        remain = tuple(s for s in phase if s not in done_slots)
        if remain:
            return remain
    return None

def label_possible_for_slots(label, slots):
    if slots is None:
        return True
    if label == 1:
        return 3 in slots
    if label == 2:
        return 1 in slots or 2 in slots
    if label == 0:
        return 4 in slots or 5 in slots
    return False

def collect_slot_priority(slot, exclude_slots, allowed_slots=None):
    if allowed_slots is not None and slot in allowed_slots:
        return 2
    # 采完一个沙袋后, 如果同帧也看到另一个沙袋, 优先继续采沙袋对。
    if (1 in exclude_slots) != (2 in exclude_slots):
        remaining_bag = 2 if 1 in exclude_slots else 1
        if slot == remaining_bag:
            return 1
    return 0

def sample_det(img, det, draw_debug=True):
    """对修正后的模型框中心矩形取样, 返回统计 9 元组; 附带取样区画框。"""
    label, score, x, y, w, h = det
    x, y, w, h = correct_box(x, y, w, h)   # 模型框偏小偏左, 先修正再取样
    rw = max(4, int(w * SAMPLE_ROI_W_FRAC))
    rh = max(4, int(h * SAMPLE_ROI_H_FRAC))
    rx = clamp_int(x + (w - rw) // 2, 0, 319)
    ry = clamp_int(y + (h - rh) // 2, 0, 239)
    roi = (rx, ry, min(rw, 320 - rx), min(rh, 240 - ry))
    st = img.get_statistics(roi=roi)
    s = [st.l_lq(), st.l_uq(), st.a_lq(), st.a_uq(), st.b_lq(), st.b_uq(),
         st.l_median(), st.a_median(), st.b_median()]
    if label == 0 and any(in_box((s[6], s[7], s[8]), g) for g in gnd_boxes):
        rw2 = max(4, int(w * BEAR_GND_RETRY_ROI_W_FRAC))
        rh2 = max(4, int(h * BEAR_GND_RETRY_ROI_H_FRAC))
        rx2 = clamp_int(x + (w - rw2) // 2, 0, 319)
        ry2 = clamp_int(y + int(h * BEAR_GND_RETRY_Y_FRAC), 0, 239)
        roi2 = (rx2, ry2, min(rw2, 320 - rx2), min(rh2, 240 - ry2))
        st2 = img.get_statistics(roi=roi2)
        s2 = [st2.l_lq(), st2.l_uq(), st2.a_lq(), st2.a_uq(), st2.b_lq(), st2.b_uq(),
              st2.l_median(), st2.a_median(), st2.b_median()]
        if not any(in_box((s2[6], s2[7], s2[8]), g) for g in gnd_boxes):
            roi = roi2
            s = s2
    if CALIB_DRAW_DEBUG and draw_debug:
        img.draw_rectangle(roi, color=(0, 255, 0))   # 绿框=实际取样区
    if label == 1:
        # 网球贴地一侧有阴影暗边(中心取样采不到, 阈值不覆盖 → blob 只框上2/3):
        # 补采底部条带, LQ/UQ 与中心区取并集, 中位数仍用中心区
        bh = max(3, h // 3)
        bx = clamp_int(x + w // 6, 0, 319)
        by = clamp_int(y + h - bh, 0, 239)
        broi = (bx, by, clamp_int(w * 2 // 3, 4, 320 - bx), min(bh, 240 - by))
        st2 = img.get_statistics(roi=broi)
        if CALIB_DRAW_DEBUG and draw_debug:
            img.draw_rectangle(broi, color=(0, 255, 0))
        s[0] = min(s[0], st2.l_lq())
        s[1] = max(s[1], st2.l_uq())
        s[2] = min(s[2], st2.a_lq())
        s[3] = max(s[3], st2.a_uq())
        s[4] = min(s[4], st2.b_lq())
        s[5] = max(s[5], st2.b_uq())
    elif label == 2:
        # 沙袋褶皱/阴影可能出现在任意方向: 只在中心取样框内部补四条边带,
        # 不扩大到模型框外, 避免把赛道布直接并入阈值。
        if s[8] >= -10:
            merge_bag_edge_strips(img, s, roi, gnd_boxes, draw_debug)
    return tuple(s)

# ======================================================================
# 主流程
# ======================================================================
exposure = calibrate_exposure()

def _rect_overlap(r1, r2):
    return not (r1[0] + r1[2] <= r2[0] or r2[0] + r2[2] <= r1[0]
                or r1[1] + r1[3] <= r2[1] or r2[1] + r2[3] <= r1[1])

def make_gnd_box(img, roi, exclude_boxes, tag=""):
    """分块地面采样: ROI 切 8x3 小块, 跳过与物体框相交的块, 对剩余块的
    中位数做离群剔除(未被模型检出的物体/场外杂物也会被剔), 幸存块的
    中位数范围±外扩构成地面盒。防止地面ROI写死后被物体/场外内容污染。
    可视化: 绿块=参与统计, 红块=与物体框相交跳过, 橙块=离群被剔。"""
    tw = max(8, roi[2] // 8)
    th = max(8, roi[3] // 3)
    tiles = []      # (tile_rect, med)
    for iy in range(3):
        for ix in range(8):
            tx = roi[0] + ix * tw
            ty = roi[1] + iy * th
            if tx + tw > 320 or ty + th > 240:
                continue
            tile = (tx, ty, tw, th)
            if any(_rect_overlap(tile, eb) for eb in exclude_boxes):
                continue
            st = img.get_statistics(roi=tile)
            tiles.append((tile, (st.l_median(), st.a_median(), st.b_median())))
    if len(tiles) < 6:
        print("[gnd] !! %s 可用块太少(%d), 物体占了地面区?" % (tag, len(tiles)))
        return None, None
    meds = [t[1] for t in tiles]
    ml = median([m[0] for m in meds])
    ma = median([m[1] for m in meds])
    mb = median([m[2] for m in meds])
    keep = [m for _, m in tiles
            if abs(m[0] - ml) <= 15 and abs(m[1] - ma) <= 10 and abs(m[2] - mb) <= 10]
    dropped = len(meds) - len(keep)
    if dropped:
        print("[gnd] %s 剔除离群块 %d/%d" % (tag, dropped, len(meds)))
    if len(keep) < 4:
        keep = meds
    box = (min(m[0] for m in keep) - GB_L_M, max(m[0] for m in keep) + GB_L_M,
           min(m[1] for m in keep) - GB_AB_M, max(m[1] for m in keep) + GB_AB_M,
           min(m[2] for m in keep) - GB_AB_M, max(m[2] for m in keep) + GB_AB_M)
    return (ml, ma, mb), box

img = sensor.snapshot()
# 采地面前先跑一次模型, 把在场物体的框(修正+外扩6px)从地面采样里排除
_ex = [correct_box(d[2] - 6, d[3] - 6, d[4] + 12, d[5] + 12)
       for d in detect_boxes(img) if d[1] >= 0.15]
ground, gnd_box = make_gnd_box(img, METER_ROI, _ex, "near")   # 近处地面(主, 阈值生成用)
_, gnd_box_far = make_gnd_box(img, METER_ROI_FAR, _ex, "far")  # 远处地面(掠射角色度漂移)
if gnd_box is None:
    raise Exception("近处地面采样失败, 清空地面区重跑")
gnd_boxes = [gnd_box] + ([gnd_box_far] if gnd_box_far else [])
print("[gnd] near med=%s box=%s" % (ground, gnd_box))
print("[gnd] far  box=%s" % (gnd_box_far,))

def collect_one(exclude_slots, allowed_slots=None):
    """采一个物体: 跳过已完成/坏框/非当前阶段槽位, 锁定一个未完成槽位采够 SAMPLE_FRAMES 帧。
    返回 (slot, threshold, med, n) 或 None(超时/失败)。
    exclude_slots: 已标定过的槽位, 检测到会提示换物体。"""
    if allowed_slots is not None:
        allowed_slots = tuple(allowed_slots)
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
    swap_clear_count = SWAP_CLEAR_FRAMES if not exclude_slots else 0
    while len(samples) < SAMPLE_FRAMES:
        idle += 1
        if idle > SWAP_IDLE:
            return None
        img = sensor.snapshot()
        if CALIB_DRAW_DEBUG:
            img.draw_rectangle(METER_ROI, color=(120, 120, 0))
            img.draw_rectangle(METER_ROI_FAR, color=(120, 120, 0))
        dets = sorted(detect_boxes(img), key=lambda d: d[1], reverse=True)
        if not dets:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            if exclude_slots and swap_clear_count < SWAP_CLEAR_FRAMES:
                swap_clear_count += 1
                idle = 0
                hint_tick += 1
                if hint_tick % 8 == 1:
                    print("[collect] swap clear %d/%d wait=%s done=%s" % (
                        swap_clear_count, SWAP_CLEAR_FRAMES,
                        slot_list_names(allowed_slots),
                        slot_list_names(tuple(sorted(exclude_slots)))))
                continue
            continue
        candidates = []
        reject_draws = []
        done_hint_slot = None
        phase_hint_name = None
        bad_hint = None
        for det in dets:
            label, score, x, y, w, h = det
            if score < DET_SCORE or w < MIN_BOX_W or h < MIN_BOX_W:
                reject_draws.append(det)
                if bad_hint is None:
                    bad_hint = (label, score, w)
                continue
            if not label_possible_for_slots(label, allowed_slots):
                reject_draws.append(det)
                phase_hint_name = LABEL_NAMES.get(label, "?")
                continue
            s = sample_det(img, det, False)
            reason = None
            # 网球=高光+绿边+底部暗边合并取样, L/AB 离散都单独放宽
            ab_lim = IQR_AB_MAX_BALL if label == 1 else IQR_AB_MAX
            l_lim = IQR_L_MAX
            gnd_hit = any(in_box((s[6], s[7], s[8]), g) for g in gnd_boxes)
            if s[6] >= SAMPLE_L_CLIP:
                reason = "过曝 Lmed=%d" % s[6]
            elif gnd_hit and not allow_ground_sample(label, s):
                reason = "采到地面"
            elif (s[1] - s[0]) > l_lim or (s[3] - s[2]) > ab_lim or (s[5] - s[4]) > ab_lim:
                reason = "分布过宽 IQR=(%d,%d,%d)" % (s[1]-s[0], s[3]-s[2], s[5]-s[4])
            if reason:
                reject_draws.append(det)
                print("[det] %s score=%.2f med=(%d,%d,%d) 丢弃: %s" % (
                    LABEL_NAMES.get(label, "?"), score, s[6], s[7], s[8], reason))
                continue
            elif gnd_hit:
                print("[det] blue_bag med=(%d,%d,%d) 命中地面盒但放行" % (
                    s[6], s[7], s[8]))
            slot = assign_slot(label, s)
            if slot <= 0:
                reject_draws.append(det)
                continue
            if slot in exclude_slots:
                reject_draws.append(det)
                done_hint_slot = slot
                continue
            if allowed_slots is not None and slot not in allowed_slots:
                reject_draws.append(det)
                phase_hint_name = SLOT_NAMES.get(slot, str(slot))
                continue
            if target_slot >= 0 and (slot != target_slot or label != target_label):
                reject_draws.append(det)
                continue    # 采样中途出现别的物体, 忽略
            candidates.append((slot, label, score, x, y, w, h, s, det))

        if exclude_slots and done_hint_slot is not None:
            swap_clear_count = 0
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                print("[collect] wait swap: old %s still visible; wait=%s samples=%d" % (
                    SLOT_NAMES.get(done_hint_slot, done_hint_slot),
                    slot_list_names(allowed_slots), len(samples)))
            continue
        if exclude_slots and swap_clear_count < SWAP_CLEAR_FRAMES:
            swap_clear_count += 1
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                print("[collect] swap clear %d/%d wait=%s done=%s" % (
                    swap_clear_count, SWAP_CLEAR_FRAMES,
                    slot_list_names(allowed_slots),
                    slot_list_names(tuple(sorted(exclude_slots)))))
            continue

        if not candidates:
            for det in reject_draws:
                draw_model_box(img, det, ok=False)
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            hint_tick += 1
            if done_hint_slot is not None and hint_tick % 30 == 1:
                print("[det] %s 已标定过, 请换下一个物体" % SLOT_NAMES.get(done_hint_slot, done_hint_slot))
            elif phase_hint_name is not None and hint_tick % 30 == 1:
                print("[det] 当前等待 %s, 忽略 %s" % (
                    slot_list_names(allowed_slots), phase_hint_name))
            elif bad_hint is not None and hint_tick % 30 == 1:
                label, score, w = bad_hint
                print("[det] %s score=%.2f box_w=%d 不合格(低分/框小)" % (
                    LABEL_NAMES.get(label, "?"), score, w))
            continue

        picked = max(candidates, key=lambda it: (collect_slot_priority(it[0], exclude_slots, allowed_slots), it[2]))
        draw_model_box(img, picked[8], ok=True)
        if CALIB_DRAW_DEBUG:
            sample_det(img, picked[8], True)
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
                        print("[det] red_bag wait stable %d/%d box=(%d,%d,%d,%d)" % (
                            red_bag_stable_count, RED_BAG_STABLE_FRAMES, x, y, w, h))
                    continue
                red_bag_track_box = box
            elif not red_bag_box_stable(red_bag_track_box, box,
                                        RED_BAG_TRACK_CENTER_PX,
                                        RED_BAG_TRACK_SIZE_PX):
                red_bag_stable_hint += 1
                idle = 0
                if red_bag_stable_hint % 8 == 1:
                    print("[det] red_bag box jump, wait stable box=(%d,%d,%d,%d) base=(%d,%d,%d,%d)" % (
                        x, y, w, h, red_bag_track_box[0], red_bag_track_box[1],
                        red_bag_track_box[2], red_bag_track_box[3]))
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
                    print("[det] brn_bear wait stable %d/%d box=(%d,%d,%d,%d)" % (
                        brn_bear_stable_count, BRN_BEAR_STABLE_FRAMES, x, y, w, h))
                continue
        else:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
        samples.append(s)
        idle = 0
        print("[det] %s score=%.2f box=(%d,%d,%d,%d) med=(%d,%d,%d)  样本 %d/%d" % (
            SLOT_NAMES.get(slot, slot), score, x, y, w, h,
            s[6], s[7], s[8], len(samples), SAMPLE_FRAMES))
    m = [median([s[i] for s in samples]) for i in range(9)]
    t = tighten_slot_threshold(target_slot, build_threshold(m, ground, gnd_box, gnd_boxes, target_slot), m)
    return (target_slot, t, (m[6], m[7], m[8]), len(samples))


def banner(slot, t, med, n, ok):
    print("")
    print("##############################################")
    print("###")
    if ok:
        print("###   %s 标定完成 (%d 帧样本)" % (SLOT_NAMES.get(slot, slot), n))
        print("###")
        print("###   阈值 = %s" % (t,))
        print("###   med = (%d, %d, %d)   exposure = %dus" % (med[0], med[1], med[2], exposure))
    else:
        print("###   !! %s 阈值罩住地面色, 不可用 !!" % SLOT_NAMES.get(slot, slot))
        print("###   阈值 = %s   地面med = %s" % (t, ground))
    print("###")
    print("##############################################")
    print("")


thresholds = {}
slot_meds = {}
max_objects = 1 if CALIB_MODE == "single" else N_OBJECTS

print("")
print("======== ② 采样 (%s 模式) ========" % CALIB_MODE)
print("把第 1 个物体摆进画面... (每个物体需 %d 帧合格样本)" % SAMPLE_FRAMES)

while len(thresholds) < max_objects:
    done_slots = set(thresholds)
    allowed_slots = allowed_collect_slots(done_slots) if CALIB_MODE == "multi" else None
    if allowed_slots is not None:
        print(">>> 当前等待: %s <<<" % slot_list_names(allowed_slots))
    r = collect_one(done_slots, allowed_slots)
    if r is None:
        if thresholds:
            print(">>> %d 帧没等到新物体, 结束采样 <<<" % SWAP_IDLE)
        else:
            print("!!!!! 采样失败: 没采到任何合格样本 !!!!!")
            print("检查: 物体是否够近(框宽>=30px)? [det] 打印的丢弃原因?")
        break
    slot, t, med, n = r
    ok = not in_box(ground, t)
    banner(slot, t, med, n, ok)
    if ok:
        thresholds[slot] = t
        slot_meds[slot] = med
    if len(thresholds) < max_objects:
        allowed_slots = allowed_collect_slots(set(thresholds)) if CALIB_MODE == "multi" else None
        done_msg = ", ".join(SLOT_NAMES[s] for s in sorted(thresholds))
        if allowed_slots is not None:
            print(">>> 请换下一个物体 (下一目标: %s; 已完成: %s) <<<" % (
                slot_list_names(allowed_slots), done_msg))
        else:
            print(">>> 请换下一个物体 (已完成: %s) <<<" % done_msg)

# 互吃 + 冲突分离(多物体时)
slots = sorted(thresholds)
for i in range(len(slots)):
    for j in range(i + 1, len(slots)):
        sa, sb = slots[i], slots[j]
        if sa not in thresholds or sb not in thresholds:
            continue
        t1, t2 = thresholds[sa], thresholds[sb]
        if not (not (t1[3] < t2[2] or t2[3] < t1[2] or t1[5] < t2[4] or t2[5] < t1[4])):
            continue
        if t1[1] < t2[0] or t2[1] < t1[0]:
            continue
        ma, mb = slot_meds[sa], slot_meds[sb]
        gaps = [abs(ma[k] - mb[k]) for k in range(3)]
        k = gaps.index(max(gaps))
        if gaps[k] < 8:
            print("[thr] !! %s<->%s 分不开, 双双废弃" % (SLOT_NAMES[sa], SLOT_NAMES[sb]))
            del thresholds[sa]
            del thresholds[sb]
            continue
        mid = (ma[k] + mb[k]) / 2.0
        lo_i, hi_i = k * 2, k * 2 + 1
        lower, higher = (sa, sb) if ma[k] < mb[k] else (sb, sa)
        tl = list(thresholds[lower]); th2 = list(thresholds[higher])
        tl[hi_i] = min(tl[hi_i], int(mid - 1))
        th2[lo_i] = max(th2[lo_i], int(mid + 1))
        if k == 0 and lower == 4 and higher == 5:
            tl[hi_i] = min(100, tl[hi_i] + BRN_BEAR_L_PAD_HI)
        thresholds[lower] = tuple(tl); thresholds[higher] = tuple(th2)
        print("[thr] 冲突分离 %s<->%s: 通道%s 切于 %.0f" % (
            SLOT_NAMES[sa], SLOT_NAMES[sb], "LAB"[k], mid))

print("=" * 46)
print("最终结果  exposure_us=%d" % exposure)
print("ground=%s" % (gnd_box,))
print("ground_far=%s" % (gnd_box_far,))
for slot in sorted(thresholds):
    print("%s = %s" % (SLOT_NAMES[slot], thresholds[slot]))
print("=" * 46)

if WRITE_FILE and thresholds:
    with open('/sd/color_thr.txt', 'w') as fp:
        fp.write("exposure_us=%d\n" % exposure)
        fp.write("ground=%d,%d,%d,%d,%d,%d\n" % gnd_box)
        if gnd_box_far:
            fp.write("ground2=%d,%d,%d,%d,%d,%d\n" % gnd_box_far)
        for slot in sorted(thresholds):
            fp.write("%d,%d,%d,%d,%d,%d,%d\n" % ((slot,) + thresholds[slot]))
    print("已写 /sd/color_thr.txt")

# ---------- 动态分界线(与 xuezhang/shijue/main.py 保持一致) ----------
BLUE_GROUND_THRESHOLD = [(0, 55, -30, 45, -90, -7)]
CUT_BLOB_MIN_H = 12
CUT_BLOB_BOTTOM_MARGIN = 25
CUT_GAP_BRIDGE = 10
CUT_LEFT_X = 0
CUT_RIGHT_X = 320
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
CUT_UPDATE_INTERVAL = 2
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_Y_MARGIN = 6
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2
CUT_BLOB_DELTA_SMALL = CUT_BLOB_DELTA
CUT_TINY_BLOB_MAX_W = 8
CUT_TINY_BLOB_MAX_H = 8
CUT_TINY_BLOB_MIN_BELOW = 2
CUT_OUTSIDE_SHORT_MAX_H = 12
CUT_OUTSIDE_BOTTOM_MARGIN = 2
cut_left_y = DETECT_Y_MIN
cut_right_y = DETECT_Y_MIN
cut_valid = False
cut_miss = 0

def cut_line_y_at_x(x):
    return cut_left_y   # 水平裁切线, 全画面统一

def tiny_blob_outside_cut(b):
    return False

def cut_delta_for_blob(b):
    return CUT_BLOB_DELTA

def off_margin_for_blob(b):
    return OFF_BOX_MARGIN_SMALL if b.w() < SMALL_BLOB_RELAX_W else OFF_BOX_MARGIN_NEAR

def ground_margin_for_blob(b):
    return -GND_BOX_SHRINK_SMALL if b.w() < SMALL_BLOB_RELAX_W else 0

def ground_margin_for_slot(b, slot):
    if slot == 1:
        return -BLUE_BAG_GND_BOX_SHRINK
    return ground_margin_for_blob(b)

def blob_center_lab(img, b):
    side = max(4, int(min(b.w(), b.h()) * 0.5))
    rx = clamp_int(b.cx() - side // 2, 0, 319)
    ry = clamp_int(b.cy() - side // 2, 0, 239)
    st = img.get_statistics(roi=(rx, ry, min(side, 320 - rx), min(side, 240 - ry)))
    return (st.l_median(), st.a_median(), st.b_median())

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
            lab = blob_center_lab(img, cb)
            if not in_box(lab, core_thr, off_margin_for_blob(cb)):
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

def find_slot_blobs(img, thr, roi, pixels_threshold, area_threshold, slot):
    if slot == 4:
        try:
            return img.find_blobs([thr], roi=roi, pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True,
                                  margin=BRN_BEAR_MERGE_MARGIN)
        except TypeError:
            pass
    return img.find_blobs([thr], roi=roi, pixels_threshold=pixels_threshold,
                          area_threshold=area_threshold, merge=True)

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

def update_cut_line(img):
    global cut_left_y, cut_right_y, cut_valid, cut_miss
    if frame % CUT_UPDATE_INTERVAL != 0:
        return
    strip_h = CUT_SCAN_Y_MAX - CUT_SCAN_Y_MIN
    top_ys = []
    strip_xs = []
    for sx in CUT_STRIP_XS:
        roi = (sx - CUT_STRIP_HALF_W, CUT_SCAN_Y_MIN, CUT_STRIP_HALF_W * 2 + 1, strip_h)
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi, pixels_threshold=CUT_MIN_PIXELS,
                               area_threshold=CUT_MIN_AREA, merge=True)
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            top_ys.append(ty)
            strip_xs.append(sx)

    valid_strips = len(top_ys)
    if valid_strips >= CUT_MIN_VALID_STRIPS:
        if max(strip_xs) - min(strip_xs) < CUT_MIN_VALID_X_SPAN:
            valid_strips = 0

    top_y_min = None
    if valid_strips >= 1:
        top_ys.sort()
        pick_i = (valid_strips - 1) * 2 // 3
        top_y_min = top_ys[pick_i]
        if valid_strips >= 3 and top_ys[pick_i] - top_ys[0] > CUT_BAFFLE_SPREAD_PX:
            top_y_min = top_ys[pick_i]
        elif valid_strips >= 2 and top_ys[1] - top_ys[0] > CUT_SINGLE_STRIP_MAX_LEAD:
            top_y_min = top_ys[1]

    if valid_strips >= CUT_MIN_VALID_STRIPS:
        cut_miss = 0
        if not cut_valid:
            cut_left_y = top_y_min
            cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            delta = top_y_min - cut_left_y
            if delta < -CUT_MAX_STEP_UP:
                top_y_min = cut_left_y - CUT_MAX_STEP_UP
            elif delta > CUT_MAX_STEP_DOWN:
                top_y_min = cut_left_y + CUT_MAX_STEP_DOWN
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
    if cut_valid:
        y_base = clamp_int(min(cut_left_y, cut_right_y) - CUT_Y_MARGIN, DETECT_Y_MIN, 239)
    else:
        y_base = DETECT_Y_MIN
    return (0, y_base, 320, 240 - y_base)

# ======================================================================
# ④ 预览: 单目标锁定(彩色框) + 模型框(黄色) 对照
# ======================================================================
print("======== ④ 预览 (色块单目标=彩色框, 模型=黄框) ========")

def box_iou(a, b):
    ax1, ay1, aw, ah = a
    bx1, by1, bw, bh = b
    iw = min(ax1 + aw, bx1 + bw) - max(ax1, bx1)
    ih = min(ay1 + ah, by1 + bh) - max(ay1, by1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / float(aw * ah + bw * bh - inter)

def box_contains(outer, inner, m=6):
    """outer 是否(带余量)包含 inner。色块框应大致包住模型框。"""
    ox, oy, ow, oh = outer
    ix, iy, iw, ih = inner
    return (ox - m <= ix and oy - m <= iy
            and ix + iw <= ox + ow + m and iy + ih <= oy + oh + m)

DBG_RAW = 0
DBG_SMALL = 1
DBG_SHAPE = 2
DBG_CUT = 3
DBG_CORE = 4
DBG_GND = 5
DBG_OFF = 6
DBG_OK = 7
DBG_LEN = 8

def slots_for_label(label):
    if label == 1:
        return (3,)
    if label == 2:
        return (1, 2)
    if label == 0:
        return (4, 5)
    return ()

def cmp_debug_summary(label, dbg, roi_y):
    slots = [s for s in slots_for_label(label) if s in thresholds]
    if not slots:
        return "无该类阈值"
    total = [0] * DBG_LEN
    parts = []
    for s in slots:
        d = dbg.get(s, [0] * DBG_LEN)
        for i in range(DBG_LEN):
            total[i] += d[i]
        parts.append("%s raw=%d ok=%d small=%d shape=%d cut=%d core=%d gnd=%d off=%d" % (
            SLOT_NAMES.get(s, s), d[DBG_RAW], d[DBG_OK], d[DBG_SMALL], d[DBG_SHAPE],
            d[DBG_CUT], d[DBG_CORE], d[DBG_GND], d[DBG_OFF]))
    if total[DBG_RAW] == 0:
        head = "raw=0 阈值未抓到"
    else:
        reasons = ((DBG_SMALL, "small"), (DBG_SHAPE, "shape"), (DBG_CUT, "cut"),
                   (DBG_CORE, "core"), (DBG_GND, "gnd"), (DBG_OFF, "off"))
        bi, bn = reasons[0]
        for ri, rn in reasons:
            if total[ri] > total[bi]:
                bi, bn = ri, rn
        head = "raw=%d 被过滤: %s=%d" % (total[DBG_RAW], bn, total[bi])
    return "%s; cut_y=%d roi_y=%d; %s" % (head, cut_left_y, roi_y, "; ".join(parts))

clock = time.clock()
frame = 0
model_dets = []
model_lock_box = None
last_best_rect = None
while True:
    clock.tick()
    frame += 1
    img = sensor.snapshot()
    if VIEW_MASK_SLOT in thresholds:
        img.binary([thresholds[VIEW_MASK_SLOT]])
        if PREVIEW_CMP_DEBUG and frame % 30 == 0:
            print("fps=%.1f (mask视图 slot=%d)" % (clock.fps(), VIEW_MASK_SLOT))
        continue

    # 分界线: 每 2 帧更新, 红线画出, 线上方候选拒绝
    if frame % 2 == 0:
        update_cut_line(img)

    # ---- 色块: 检测 ROI 从分界线以下开始(赛道外整体切掉), 全局只锁一个 ----
    # 候选复核与车载版一致(静默拒绝, 不画框): 线上方/地面盒命中/不在本槽盒内
    det_roi = dynamic_detect_roi()
    candidates = []
    cmp_dbg = {}
    for slot, thr in thresholds.items():
        dbg = [0] * DBG_LEN
        cmp_dbg[slot] = dbg
        blobs = find_slot_blobs(img, thr, det_roi, 20, 20, slot)
        dbg[DBG_RAW] = len(blobs)
        for b in blobs:
            if blue_bag_side_gutter_blob(b, slot):
                dbg[DBG_CUT] += 1
                continue
            if blue_bag_bottom_shadow_blob(b, slot):
                dbg[DBG_GND] += 1
                continue
            if slot in (1, 2) and (b.w() < BAG_SMALL_MIN_W or b.h() < BAG_SMALL_MIN_H):
                dbg[DBG_SMALL] += 1
                continue
            if slot == 3 and (b.w() < BALL_SMALL_MIN_W or b.h() < BALL_SMALL_MIN_H):
                dbg[DBG_SMALL] += 1
                continue
            if slot in (4, 5) and b.h() < bear_small_min_h_for_slot(slot):
                dbg[DBG_SMALL] += 1
                continue
            if PREVIEW_STRICT and b.w() >= 280:
                dbg[DBG_SHAPE] += 1
                continue
            shape_skip_w = bear_shape_skip_w_for_slot(slot) if slot in (4, 5) else SMALL_BLOB_SHAPE_SKIP_W
            if PREVIEW_STRICT and b.w() >= shape_skip_w and b.density() < 0.3:
                dbg[DBG_SHAPE] += 1
                continue
            if tiny_blob_outside_cut(b):
                dbg[DBG_CUT] += 1
                continue
            if cut_valid and b.cy() < cut_line_y_at_x(b.cx()) + cut_delta_for_blob(b):
                dbg[DBG_CUT] += 1
                continue
            if slot == 1:
                b = refine_blue_bag_blob(img, b, thr)
                if b is None:
                    dbg[DBG_CORE] += 1
                    continue
            if not blob_size_plausible(b, slot):
                dbg[DBG_SMALL] += 1
                continue
            lab = blob_center_lab(img, b)
            gnd_margin = ground_margin_for_slot(b, slot)
            if any(in_box(lab, g, gnd_margin) for g in gnd_boxes):
                dbg[DBG_GND] += 1
                continue
            if not in_box(lab, thr, off_margin_for_blob(b)):
                dbg[DBG_OFF] += 1
                continue
            dbg[DBG_OK] += 1
            candidates.append((slot, b))
    best = None
    if candidates:
        if last_best_rect is not None:
            lx = last_best_rect[0] + last_best_rect[2] // 2
            ly = last_best_rect[1] + last_best_rect[3] // 2
            best = max(candidates, key=lambda it: it[1].pixels()
                       - ((it[1].cx() - lx) ** 2 + (it[1].cy() - ly) ** 2) // 20)
        else:
            best = max(candidates, key=lambda it: it[1].pixels())
    if best:
        slot, b = best
        last_best_rect = b.rect()
        if PREVIEW_DRAW_DEBUG or PREVIEW_SHOW_COLOR_BOX:
            c = DRAW_COLORS.get(slot, (255, 255, 255))
            img.draw_rectangle(b.rect(), color=c)
            if PREVIEW_DRAW_DEBUG:
                img.draw_string(b.x(), max(0, b.y() - 10), SLOT_NAMES[slot], color=c)
                img.draw_string(b.x(), min(228, b.y() + b.h() + 2),
                                format_world_info(slot, b), color=c)
    else:
        last_best_rect = None

    # ---- 模型框(每 MODEL_EVERY 帧刷新, 预览只画黄框不画修正框) ----
    if PREVIEW_MODEL:
        if frame % MODEL_EVERY == 1:
            # 生产同款分段门槛: 远处小目标允许低分
            dets = [d for d in detect_boxes(img)
                    if d[1] >= model_score_floor(d[2], d[3], d[4], d[5])]
            # 场外过滤(生产由 cut+色块确认保证): 中心在切割线上方的模型框剔除
            if cut_valid:
                dets = [d for d in dets
                        if (d[3] + d[5] // 2) >= cut_line_y_at_x(d[2] + d[4] // 2) + CUT_BLOB_DELTA]
            # 轻量锁定(生产 locked_box 思路): 有上次目标时优先选位置延续的候选,
            # 防止单帧高分误检抢走黄框
            if dets:
                if model_lock_box is not None:
                    lcx = model_lock_box[0] + model_lock_box[2] // 2
                    lcy = model_lock_box[1] + model_lock_box[3] // 2
                    near = [d for d in dets
                            if (d[2] + d[4] // 2 - lcx) ** 2 + (d[3] + d[5] // 2 - lcy) ** 2 <= 80 * 80]
                    pick = max(near, key=lambda d: d[1]) if near else max(dets, key=lambda d: d[1])
                else:
                    pick = max(dets, key=lambda d: d[1])
                model_lock_box = (pick[2], pick[3], pick[4], pick[5])
                model_dets = [pick]
            else:
                model_lock_box = None
                model_dets = []
        if model_dets:
            label, score, x, y, w, h = model_dets[0]
            img.draw_rectangle(x, y, w, h, color=MODEL_BOX_COLOR)
            img.draw_string(x, max(0, y - 10), "%s %.2f" % (LABEL_NAMES.get(label, "?"), score),
                            color=MODEL_BOX_COLOR)
    # 画面元素只保留: 地面采样框 / 切割线 / 模型框 / 色块框
    if PREVIEW_DRAW_DEBUG:
        img.draw_rectangle(METER_ROI, color=(120, 120, 0))
        img.draw_rectangle(METER_ROI_FAR, color=(120, 120, 0))
    if PREVIEW_SHOW_CUT_LINE and cut_valid:
        img.draw_line(CUT_LEFT_X, cut_left_y, CUT_RIGHT_X, cut_right_y, color=(255, 0, 0))

    # ---- 位置一致性输出: 远处模型框会偏大, 只参考中心位置 ----
    if PREVIEW_CMP_DEBUG and frame % 30 == 0:
        if best and model_dets:
            md = max(model_dets, key=lambda d: d[1])
            mbox = correct_box(md[2], md[3], md[4], md[5])   # 用修正后的框对比
            bbox = best[1].rect()
            dx = (bbox[0] + bbox[2] // 2) - (mbox[0] + mbox[2] // 2)
            dy = (bbox[1] + bbox[3] // 2) - (mbox[1] + mbox[3] // 2)
            cd = int(((dx * dx + dy * dy) ** 0.5) + 0.5)
            tag = "OK:中心对齐 cd=%d" % cd if cd <= CMP_CENTER_OK_PX else "!! 中心偏离 cd=%d" % cd
            info = format_world_info(best[0], best[1])
            print("fps=%.1f [cmp] 色块%s%s %s 模型%s(%.2f)%s -> %s" % (
                clock.fps(), SLOT_NAMES[best[0]], bbox,
                info, LABEL_NAMES.get(md[0], "?"), md[1], mbox, tag))
        elif best and not model_dets:
            info = format_world_info(best[0], best[1])
            print("fps=%.1f [cmp] 色块%s%s %s 模型无检出 -> 疑似色块误报" % (
                clock.fps(), SLOT_NAMES[best[0]], best[1].rect(), info))
        elif model_dets and not best:
            md = max(model_dets, key=lambda d: d[1])
            print("fps=%.1f [cmp] 模型%s(%.2f) 色块无目标 -> %s" % (
                clock.fps(), LABEL_NAMES.get(md[0], "?"), md[1],
                cmp_debug_summary(md[0], cmp_dbg, det_roi[1])))
        else:
            print("fps=%.1f [cmp] 双方均无目标" % clock.fps())
