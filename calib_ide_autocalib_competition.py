# ======================================================================
# calib_ide_autocalib_competition.py — OpenART IDE 比赛现场五目标自动标定
# ======================================================================
# 流程: 点运行 → ① 自动曝光(打印迭代过程)
#              → ② 采样阶段: 实时显示模型框(黄框+label+分数), 按分数尝试
#                 同帧多个模型框, 按阶段锁定一个未完成槽位采够 SAMPLE_FRAMES 帧
#              → ③ 每个槽位执行模型框/色块框连续复检，失败自动重采
#              → ④ 五目标完成后写正式阈值并进入运行预览，
#                 VIEW_MASK_SLOT 可切二值 mask 视图
# 用法: 保持地面采样区为空，按“沙袋任意顺序→网球→棕熊→白熊”提示换物；
#       每个目标完成后先移出画面，等待连续 8 帧清场，无需重启脚本。
# 输出: 五槽位全部通过才覆盖 /sd/color_thr.txt；不完整结果写 partial 文件。
# ======================================================================

import sensor, time, tf

SCRIPT_TITLE = "IDE auto-calib competition preview"
SCRIPT_BUILD = "2026-07-19 red-bag-filter-off-v23"
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
L_TARGET        = 38      # 全画面目标亮度；较原来的 40 下调 5%，抑制过亮画面
L_TOL           = 1.0     # 目标附近允许小幅波动，避免曝光来回调整
STEP_MIN        = 0.80    # 单轮曝光调整下限
STEP_MAX        = 1.1    # 单轮曝光调整上限
EXPOSURE_INIT   = 1400
EXPOSURE_MIN    = 100
EXPOSURE_MAX    = 4500
METER_FULL      = True    # True=全画面测光; False=用 METER_ROI 测光
METER_ROI       = (0, 140, 320, 100)  # 地面采样区(也是地面盒取样区):
                                       # 左右各留10px, 直到画面最底(y140~240),
                                       # 覆盖更宽的布面, 地面盒更能代表全场
METER_AVG       = 4       # 每轮测光平均帧数
HILIGHT_LUQ     = 92      # 提前保护白熊/浅色物体高光，超过即压曝光 20%

# ================= ② 模型取样参数 =================
DET_SCORE       = 0.20    # 模型置信度门槛
SAMPLE_FRAMES   = 10      # 需要采够的合格样本帧数
SAMPLE_ROI_W_FRAC = 0.94  # LAB 边界框内略缩，避开物体/背景混合像素
SAMPLE_ROI_H_FRAC = 0.94
BAG_CORE_ROI_W_FRAC = 0.60    # 沙袋核心仅用于判定饱和度/地面冲突，不替代宽阈值
BAG_CORE_ROI_H_FRAC = 0.60
BALL_CORE_ROI_W_FRAC = 0.48   # 圆形目标不能用接近整框的矩形统计，四角会混入地面
BALL_CORE_ROI_H_FRAC = 0.42
BALL_BOTTOM_ROI_W_FRAC = 0.38 # 球体下部内缩条带，补暗边但不碰框底地面
BALL_BOTTOM_ROI_H_FRAC = 0.14
BALL_BOTTOM_ROI_Y_FRAC = 0.62
BALL_BOTTOM_IQR_L_MAX = 60
BALL_BOTTOM_IQR_AB_MAX = 60
BALL_BOTTOM_AB_MED_DELTA = 55
BEAR_SAMPLE_ROI_H_FRAC = 0.86
BEAR_SAMPLE_ROI_Y_FRAC = 0.04
BEAR_GND_RETRY_ROI_W_FRAC = 0.55
BEAR_GND_RETRY_ROI_H_FRAC = 0.30
BEAR_GND_RETRY_Y_FRAC = 0.25
MIN_BOX_W       = 10      # 模型框最小边(px)
COLLECT_TIMEOUT = 300     # 采样阶段最多跑多少帧(采不够就放弃并提示)
# label 顺序: bear / ball / bag。仅在 LAB 边界不可靠时作为兜底框。
BOX_EXPAND_BY_LABEL = ((1.40, 1.65), (1.35, 1.50), (1.50, 1.55))
BOX_MIN_PAD_BY_LABEL = ((4, 5), (3, 3), (5, 4))
BOX_OFF_X       = -1      # 模型框略偏右: 向左平移修正(px, 正数=向右)
BOX_OFF_Y       = 0       # 垂直方向平移修正(px, 正数=向下)

# 从模型框四边向外扫描小块 LAB 中位数。射线越过类别兜底框后，
# 再从外侧连续背景反向确认边界，避免把目标内部阴影误判成背景。
LAB_EDGE_TILE = 4
LAB_EDGE_STEP = 4
LAB_EDGE_SEARCH_FRAC = 0.75
LAB_EDGE_SEARCH_MIN = 16
LAB_EDGE_SEARCH_MAX = 48
LAB_EDGE_SEARCH_EXTRA = 16
LAB_EDGE_MIN_GROW = 4
LAB_EDGE_GUARD_FRAC = 0.40
LAB_EDGE_BOTTOM_GUARD_FRAC = 1.00
LAB_EDGE_BACKGROUND_STABLE_SCALE = 0.75
LAB_EDGE_MIN_RAYS = 2
LAB_EDGE_COHERENCE_PX = 16
LAB_EDGE_PAD = 1
BEAR_LAB_BOTTOM_EXTRA_PX = 4

# ================= ③ 阈值生成参数 =================
IQR_K           = 0.8     # 四分位外扩系数(大=松, 小=紧)
AB_MARGIN       = 5       # A/B 额外余量
AB_SPAN_MAX     = 45      # A/B 盒跨度硬上限
AB_SPAN_MAX_BALL = 75     # 网球底部暗边+高光会拉开 A/B, 只对 ball 放宽
AB_SPAN_MAX_BAG = 60      # 沙袋表面/边缘阴影有方向性, 但仍比网球保守
AB_SPAN_MAX_BRN_BEAR = 75 # 棕熊毛绒暗部/黄亮部跨度大，45 会只留下零散色块
AB_SPAN_MAX_WHT_BEAR = 60 # 白熊同时包含黄白高光和灰色阴影，避免只留下局部小框
L_SPAN_MAX      = 60      # 保留实测 L 时的跨度上限
L_WIDE          = (15, 100)  # 高饱和色 L 放宽范围
NEUTRAL_AB      = 12      # A/B 都离 0 近于此 = 低饱和色, 保留实测 L
L_MARGIN_LOWSAT = 8       # 低饱和色 L 余量
GND_NEAR        = 10      # 槽位盒与地面盒 A/B 接近判定余量
GND_NEAR_BRN_BEAR = 0     # 棕熊仅在 A/B 盒实际重叠时才用 L 切地面
BRN_BEAR_L_BELOW_MED = 14 # 保留本体暗部，但不把远低于本体亮度的投影纳入
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
CALIB_SLOT_PHASES = ((1, 2), (3,), (4,), (5,))
# 沙袋任意顺序 -> 网球 -> 棕熊 -> 白熊。两只熊按阶段强制分槽，
# 避免暖色灯光下白熊呈黄褐色而被 LAB 规则误分到棕熊槽。
SWAP_IDLE       = 450      # multi 模式: 连续这么多帧没等到新物体的合格样本就结束(~15s)
SWAP_CLEAR_FRAMES = 8     # 已完成物体离场后再等几帧, 防换物体过程混采

# 每个槽位采样完成后再用“模型框 vs 新阈值色块框”连续复检。这里只拦截明显错误，
# 不要求两个框像素级重合，避免模型框本身偏小或单帧反光导致误重标。
VERIFY_FRAMES = 10
VERIFY_TIMEOUT = 120
VERIFY_BAD_STREAK_LIMIT = 3
VERIFY_BAD_TOTAL_LIMIT = 4
VERIFY_JUMP_LIMIT = 2
VERIFY_BLOB_LIMITS = ((70, 100), (70, 100), (80, 80), (70, 100), (70, 100))
# RED_BAG_MAX_WIDTH_HEIGHT_X100 = 170  # 已停用
VERIFY_COLOR_Y_MAX = 230
VERIFY_CENTER_MAX_PERCENT = 55
VERIFY_MIN_AREA_PERCENT = 18
VERIFY_MAX_AREA_PERCENT = 450
VERIFY_MIN_SIDE_PERCENT = 30
VERIFY_MIN_OVERLAP_PERCENT = 15
VERIFY_REL_CENTER_JUMP_PERCENT = 32
VERIFY_REL_SIZE_JUMP_PERCENT = 45
VERIFY_AREA_JUMP_PERCENT = 250
VERIFY_SPREAD_CENTER_PERCENT = 40
VERIFY_SPREAD_SIZE_PERCENT = 55

# ================= ⑥ 输出/预览 =================
WRITE_FILE      = True   # True=结果写 /sd/color_thr.txt
RESULT_PATH     = '/sd/color_thr.txt'
PARTIAL_RESULT_PATH = '/sd/color_thr_partial.txt'
VIEW_MASK_SLOT  = 0       # 0=正常预览; 1~5=该槽位二值 mask 视图
PREVIEW_MODEL   = RUN_DEBUG  # debug 预览跑模型对照; run 预览纯色块, 不调用模型
MODEL_EVERY     = 10      # 预览阶段模型检测的帧间隔(模型慢, 不必每帧跑)
CMP_CENTER_OK_PX = 35
DETECT_Y_MIN = 8
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)
if not RUN_DEBUG:
    VIEW_MASK_SLOT = 0
BEAR_SEPARATION_MIN_GAP = 8
BEAR_SEPARATION_MARGIN = 2
BRN_BEAR_MERGE_MARGIN = 12
BRN_BEAR_BALL_SHADOW_MAX_AREA_PERCENT = 55
BRN_BEAR_BALL_SHADOW_X_OVERLAP_PERCENT = 60
BRN_BEAR_BALL_SHADOW_Y_MARGIN = 6
BRN_BEAR_STABLE_FRAMES = 3
BRN_BEAR_STABLE_CENTER_PX = 12
BRN_BEAR_STABLE_SIZE_PX = 12
RED_BAG_STABLE_FRAMES = 3
RED_BAG_STABLE_CENTER_PX = 24
RED_BAG_STABLE_SIZE_PX = 16
RED_BAG_TRACK_CENTER_PX = 42
RED_BAG_TRACK_SIZE_PX = 24
RED_BAG_REBASE_FRAMES = 3
WHT_BEAR_L_MIN = 58
WHT_BEAR_A_ABS_MAX = 18
WHT_BEAR_B_MAX = 32
WHT_BEAR_STABLE_FRAMES = 3
WHT_BEAR_STABLE_CENTER_PX = 12
WHT_BEAR_STABLE_SIZE_PX = 12
WHT_BEAR_GND_L_GAP = 2
WHT_BEAR_CORE_L_PAD = 3
WHT_BEAR_MERGE_MARGIN = 10
WHT_BEAR_BOX_EMA_NEW_NUM = 1
WHT_BEAR_BOX_EMA_DEN = 3
WHT_BEAR_SMOOTH_MAX_JUMP_PX = 55
WHT_BEAR_SMOOTH_MAX_JUMP2 = WHT_BEAR_SMOOTH_MAX_JUMP_PX * WHT_BEAR_SMOOTH_MAX_JUMP_PX
WB_GAINS        = (92.00, 64.00, 101.00)
MODEL_PATH      = '/sd/dataset_25000_exposure.tflite'
# =====================================================

def validate_wb_gains(values):
    if len(values) != 3:
        raise ValueError('wb_gains must contain R,G,B')
    gains = (float(values[0]), float(values[1]), float(values[2]))
    for gain in gains:
        if gain < 0 or gain > 255:
            raise ValueError('wb_gains out of range')
    return gains

def load_wb_gains(path=RESULT_PATH):
    try:
        with open(path, 'r') as fp:
            for line in fp:
                line = line.strip()
                if line.startswith('wb_gains='):
                    gains = validate_wb_gains(
                        line.split('=', 1)[1].split(','))
                    print('[WB] loaded R=%.2f G=%.2f B=%.2f from %s' %
                          (gains[0], gains[1], gains[2], path))
                    return gains
    except Exception as error:
        print('[WB] load failed: ' + str(error))
    print('[WB] using fallback R=%.2f G=%.2f B=%.2f' % WB_GAINS)
    return WB_GAINS

startup_wb_gains = load_wb_gains()

SLOT_NAMES = {1: "blue_bag", 2: "red_bag", 3: "ball", 4: "brn_bear", 5: "wht_bear"}
LABEL_NAMES = {0: "bear", 1: "ball", 2: "bag"}
DRAW_COLORS = {1: (0, 170, 255), 2: (255, 0, 0), 3: (0, 255, 0),
               4: (255, 180, 0), 5: (255, 255, 255)}
MODEL_BOX_COLOR = (255, 255, 0)   # 模型框统一黄色

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_auto_whitebal(False, rgb_gain_db=startup_wb_gains)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_INIT)
sensor.skip_frames(time=800)
sensor.set_vflip(True)

net = tf.load(MODEL_PATH)

print("======== %s ========" % SCRIPT_TITLE)
print("build: %s" % SCRIPT_BUILD)
print("target: %s" % SCRIPT_TARGET)
print("preview runtime: %s" % SCRIPT_RUNTIME)

def clamp_int(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def score_text(value):
    scaled = int(float(value) * 100 + 0.5)
    decimals = scaled % 100
    return str(scaled // 100) + "." + ("0" if decimals < 10 else "") + str(decimals)

def triple_text(a, b, c):
    return "(" + str(a) + "," + str(b) + "," + str(c) + ")"

def rect_text(x, y, w, h):
    return "(" + str(x) + "," + str(y) + "," + str(w) + "," + str(h) + ")"

def median(lst):
    # 该 OpenART 固件在部分嵌套推导式中会把 iterator 留在结果里；
    # 显式复制并排序，确保中位数始终是实际数值。
    s = []
    for value in lst:
        s.append(value)
    for i in range(1, len(s)):
        value = s[i]
        j = i - 1
        while j >= 0 and s[j] > value:
            s[j + 1] = s[j]
            j -= 1
        s[j + 1] = value
    n = len(s)
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) * 0.5

def in_box(lab, box, m=0):
    return (box[0] - m <= lab[0] <= box[1] + m and box[2] - m <= lab[1] <= box[3] + m
            and box[4] - m <= lab[2] <= box[5] + m)

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

def calibration_det_qualified(det):
    return (det[1] >= DET_SCORE and det[4] >= MIN_BOX_W
            and det[5] >= MIN_BOX_W)

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

def correct_box(label, x, y, w, h, extra_pad=0):
    """按类别扩张模型框；LAB 边界不可靠时使用这个保守兜底框。"""
    if 0 <= label < len(BOX_EXPAND_BY_LABEL):
        expand_w, expand_h = BOX_EXPAND_BY_LABEL[label]
        min_pad_x, min_pad_y = BOX_MIN_PAD_BY_LABEL[label]
    else:
        expand_w, expand_h = (1.40, 1.60)
        min_pad_x, min_pad_y = (4, 4)
    cx = x + w / 2.0 + BOX_OFF_X
    cy = y + h / 2.0 + BOX_OFF_Y
    nw = max(int(w * expand_w + 0.5), w + min_pad_x * 2) + extra_pad * 2
    nh = max(int(h * expand_h + 0.5), h + min_pad_y * 2) + extra_pad * 2
    nx = clamp_int(int(cx - nw / 2), 0, 319)
    ny = clamp_int(int(cy - nh / 2), 0, 239)
    return (nx, ny, clamp_int(nw, 1, 320 - nx), clamp_int(nh, 1, 240 - ny))

def lab_distance(a, b):
    return abs(a[0] - b[0]) * 0.5 + abs(a[1] - b[1]) + abs(a[2] - b[2])

def lab_tile_median(img, cx, cy):
    half = LAB_EDGE_TILE // 2
    rx = clamp_int(int(cx) - half, 0, 319)
    ry = clamp_int(int(cy) - half, 0, 239)
    rw = min(LAB_EDGE_TILE, 320 - rx)
    rh = min(LAB_EDGE_TILE, 240 - ry)
    st = img.get_statistics(roi=(rx, ry, rw, rh))
    return (st.l_median(), st.a_median(), st.b_median())

def lab_edge_seed(img, x, y, w, h):
    cx = x + w // 2
    cy = y + h // 2
    ox = max(1, w // 5)
    oy = max(1, h // 5)
    labs = (lab_tile_median(img, cx, cy),
            lab_tile_median(img, cx - ox, cy),
            lab_tile_median(img, cx + ox, cy),
            lab_tile_median(img, cx, cy - oy),
            lab_tile_median(img, cx, cy + oy))
    seed = (median([v[0] for v in labs]),
            median([v[1] for v in labs]),
            median([v[2] for v in labs]))
    noise = median([lab_distance(v, seed) for v in labs])
    edge_jump = clamp_int(int(noise * 2.0 + 8), 12, 36)
    object_limit = clamp_int(int(noise * 3.0 + edge_jump + 6), 20, 72)
    return seed, edge_jump, object_limit

def lab_matches_ground(lab, ground_boxes):
    if not ground_boxes:
        return False
    for ground_box in ground_boxes:
        if ground_box and in_box(lab, ground_box):
            return True
    return False

def scan_lab_edge_ray(img, seed, edge_jump, object_limit, ground_boxes,
                      inside_x, inside_y, edge_x, edge_y, dx, dy, steps,
                      min_grow):
    inside = lab_tile_median(img, inside_x, inside_y)
    labs = []
    coords = []
    last_x = -1
    last_y = -1
    for i in range(steps):
        px = clamp_int(edge_x + dx * LAB_EDGE_STEP * i, 0, 319)
        py = clamp_int(edge_y + dy * LAB_EDGE_STEP * i, 0, 239)
        if px == last_x and py == last_y:
            break
        labs.append(lab_tile_median(img, px, py))
        coords.append(px if dx else py)
        last_x, last_y = px, py
        if ((dx < 0 and px <= 0) or (dx > 0 and px >= 319)
                or (dy < 0 and py <= 0) or (dy > 0 and py >= 239)):
            break
    if len(labs) < 3:
        return None

    # 先尝试从射线最外端找到连续地面。这样亮部->阴影的内部跳变不会
    # 抢在真正的目标->地面边界之前结束扫描。
    first_background = len(labs)
    ground_run = 0
    for i in range(len(labs) - 1, -1, -1):
        if not lab_matches_ground(labs[i], ground_boxes):
            break
        first_background = i
        ground_run += 1

    if ground_run < 2:
        if ground_boxes:
            return None
        # 非地面背景也可用，但必须在最外端形成至少两个连续、稳定的小块，
        # 且整体颜色确实离开了模型框内的目标核心色。
        stable_limit = max(6, int(edge_jump * LAB_EDGE_BACKGROUND_STABLE_SCALE))
        first_background = len(labs) - 1
        stable_run = 1
        for i in range(len(labs) - 2, -1, -1):
            if lab_distance(labs[i], labs[i + 1]) > stable_limit:
                break
            first_background = i
            stable_run += 1
        if stable_run < 2 or first_background <= 0:
            return None
        background_labs = labs[first_background:]
        background = (
            median([v[0] for v in background_labs]),
            median([v[1] for v in background_labs]),
            median([v[2] for v in background_labs]))
        if lab_distance(background, seed) < object_limit:
            return None
    if first_background <= 0:
        return None

    object_lab = labs[first_background - 1]
    background_lab = labs[first_background]
    if (lab_distance(object_lab, background_lab) < edge_jump
            and lab_distance(inside, background_lab) < object_limit):
        return None
    boundary = (coords[first_background - 1] + coords[first_background]) // 2
    if abs(boundary - coords[0]) < min_grow:
        return None
    return boundary

def coherent_lab_edge(values, outward):
    if len(values) < LAB_EDGE_MIN_RAYS:
        return None
    center = int(median(values))
    kept = [v for v in values if abs(v - center) <= LAB_EDGE_COHERENCE_PX]
    if len(kept) < LAB_EDGE_MIN_RAYS:
        return None
    # 边界框要覆盖目标突出部分；在已通过外侧背景确认的射线里取朝外极值。
    return min(kept) if outward < 0 else max(kept)

def refine_model_box_by_lab(img, label, x, y, w, h, ground_boxes):
    """从模型框内部向四边扫描 LAB 跳变；不可靠的边回退到类别扩框。"""
    fallback = correct_box(label, x, y, w, h)
    if w < 6 or h < 6:
        return fallback
    seed, edge_jump, object_limit = lab_edge_seed(img, x, y, w, h)
    fx, fy, fw, fh = fallback
    model_right = x + w - 1
    model_bottom = y + h - 1
    fallback_right = fx + fw - 1
    fallback_bottom = fy + fh - 1
    left_extra = max(0, x - fx)
    right_extra = max(0, fallback_right - model_right)
    top_extra = max(0, y - fy)
    bottom_extra = max(0, fallback_bottom - model_bottom)
    base_search_x = int(w * LAB_EDGE_SEARCH_FRAC)
    base_search_y = int(h * LAB_EDGE_SEARCH_FRAC)
    left_search = clamp_int(max(base_search_x, left_extra + LAB_EDGE_SEARCH_EXTRA),
                            LAB_EDGE_SEARCH_MIN, LAB_EDGE_SEARCH_MAX)
    right_search = clamp_int(max(base_search_x, right_extra + LAB_EDGE_SEARCH_EXTRA),
                             LAB_EDGE_SEARCH_MIN, LAB_EDGE_SEARCH_MAX)
    top_search = clamp_int(max(base_search_y, top_extra + LAB_EDGE_SEARCH_EXTRA),
                           LAB_EDGE_SEARCH_MIN, LAB_EDGE_SEARCH_MAX)
    bottom_search = clamp_int(max(base_search_y, bottom_extra + LAB_EDGE_SEARCH_EXTRA),
                              LAB_EDGE_SEARCH_MIN, LAB_EDGE_SEARCH_MAX)
    left_guard = max(LAB_EDGE_MIN_GROW, int(left_extra * LAB_EDGE_GUARD_FRAC))
    right_guard = max(LAB_EDGE_MIN_GROW, int(right_extra * LAB_EDGE_GUARD_FRAC))
    top_guard = max(LAB_EDGE_MIN_GROW, int(top_extra * LAB_EDGE_GUARD_FRAC))
    bottom_guard = max(
        LAB_EDGE_MIN_GROW, int(bottom_extra * LAB_EDGE_BOTTOM_GUARD_FRAC))
    inset_x = max(1, min(LAB_EDGE_STEP, w // 3))
    inset_y = max(1, min(LAB_EDGE_STEP, h // 3))
    ray_ys = (y + h // 4, y + h // 2, y + h * 3 // 4)
    ray_xs = (x + w // 4, x + w // 2, x + w * 3 // 4)
    left_values = []
    right_values = []
    top_values = []
    bottom_values = []
    for ry in ray_ys:
        edge = scan_lab_edge_ray(
            img, seed, edge_jump, object_limit, ground_boxes,
            x + inset_x, ry, x, ry, -1, 0,
            (left_search + LAB_EDGE_STEP - 1) // LAB_EDGE_STEP + 1,
            left_guard)
        if edge is not None:
            left_values.append(edge)
        edge = scan_lab_edge_ray(
            img, seed, edge_jump, object_limit, ground_boxes,
            model_right - inset_x, ry, model_right, ry, 1, 0,
            (right_search + LAB_EDGE_STEP - 1) // LAB_EDGE_STEP + 1,
            right_guard)
        if edge is not None:
            right_values.append(edge)
    for rx in ray_xs:
        edge = scan_lab_edge_ray(
            img, seed, edge_jump, object_limit, ground_boxes,
            rx, y + inset_y, rx, y, 0, -1,
            (top_search + LAB_EDGE_STEP - 1) // LAB_EDGE_STEP + 1,
            top_guard)
        if edge is not None:
            top_values.append(edge)
        edge = scan_lab_edge_ray(
            img, seed, edge_jump, object_limit, ground_boxes,
            rx, model_bottom - inset_y, rx, model_bottom, 0, 1,
            (bottom_search + LAB_EDGE_STEP - 1) // LAB_EDGE_STEP + 1,
            bottom_guard)
        if edge is not None:
            bottom_values.append(edge)
    left_edge = coherent_lab_edge(left_values, -1)
    right_edge = coherent_lab_edge(right_values, 1)
    top_edge = coherent_lab_edge(top_values, -1)
    bottom_edge = coherent_lab_edge(bottom_values, 1)
    left = fx if left_edge is None else min(x, left_edge - LAB_EDGE_PAD)
    top = fy if top_edge is None else min(y, top_edge - LAB_EDGE_PAD)
    right = fx + fw if right_edge is None else max(x + w, right_edge + LAB_EDGE_PAD + 1)
    bottom = fy + fh if bottom_edge is None else max(y + h, bottom_edge + LAB_EDGE_PAD + 1)
    if label == 0:
        bottom = min(bottom, fy + fh + BEAR_LAB_BOTTOM_EXTRA_PX)
    left = clamp_int(left, 0, 319)
    top = clamp_int(top, 0, 239)
    right = clamp_int(right, left + 1, 320)
    bottom = clamp_int(bottom, top + 1, 240)
    return (left, top, right - left, bottom - top)

H_PIX2WORLD = (
    -0.789473684210523, 0.020532099479467793, 127.59861191440086,
    -1.8064645824409328e-16, 0.5067596876807382, -167.72758820127169,
    -1.2325598412554425e-17, -0.036184210526315652,
)
WORLD_X_LIMIT_CM = 250.0
WORLD_Y_MAX_CM = 300.0

def box_to_world(x, y, w, h):
    px = float(x) + float(w) * 0.5
    py = float(y) + float(h)
    den = H_PIX2WORLD[6] * px + H_PIX2WORLD[7] * py + 1.0
    if -1e-10 < den < 1e-10:
        return (0.0, WORLD_Y_MAX_CM)
    wx = (H_PIX2WORLD[0] * px + H_PIX2WORLD[1] * py + H_PIX2WORLD[2]) / den
    wy = (H_PIX2WORLD[3] * px + H_PIX2WORLD[4] * py + H_PIX2WORLD[5]) / den
    if not (wy > 0.0 and wy <= WORLD_Y_MAX_CM):
        wy = WORLD_Y_MAX_CM
    if wx != wx:
        wx = 0.0
    elif wx < -WORLD_X_LIMIT_CM:
        wx = -WORLD_X_LIMIT_CM
    elif wx > WORLD_X_LIMIT_CM:
        wx = WORLD_X_LIMIT_CM
    return (wx, wy)

TARGET_REAL_WIDTH = [70.0, 70.0, 67.0, 120.0, 120.0]
FOCAL_LENGTH = 167.5
MIN_DETECT_DISTANCE = 50
MAX_DETECT_DISTANCE = 2000
def calculate_distance(pixel_width, color_id=1):
    if pixel_width <= 0:
        return -1
    real_width = TARGET_REAL_WIDTH[color_id - 1] if 1 <= color_id <= len(TARGET_REAL_WIDTH) else 70.0
    distance = (real_width * FOCAL_LENGTH) / pixel_width
    if distance < MIN_DETECT_DISTANCE or distance > MAX_DETECT_DISTANCE:
        return -1
    return int(distance)

def world_info_for_blob(slot, b):
    wx, wy = box_to_world(b.x(), b.y(), b.w(), b.h())
    dist = calculate_distance(b.w(), slot)
    return wx, wy, dist

def format_world_info(slot, b):
    try:
        wx, wy, dist = world_info_for_blob(slot, b)
        return "world=(%.1f,%.1f)cm dist=%dmm" % (wx, wy, dist)
    except Exception as error:
        return "world=n/a err=" + str(error)

def draw_model_box(img, det, ok=True):
    if not CALIB_DRAW_DEBUG:
        return
    label, score, x, y, w, h = det
    img.draw_rectangle(x, y, w, h, color=MODEL_BOX_COLOR)
    img.draw_string(x, max(0, y - 10), "%s %.2f" % (LABEL_NAMES.get(label, "?"), score),
                    color=MODEL_BOX_COLOR)
    img.draw_rectangle(correct_box(label, x, y, w, h), color=(0, 128, 255))
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

def write_exposure_immediately(path, exposure_us):
    """只更新正式配置中的曝光首行，保留已有地面盒和五色阈值。"""
    expected_exposure = "exposure_us=%d" % exposure_us
    old_lines = []
    try:
        with open(path, 'r') as fp:
            for line in fp:
                if not line.strip().startswith('exposure_us='):
                    old_lines.append(line)
    except Exception:
        # 首次标定时文件可能还不存在；先创建只有曝光的一行，完整结果稍后覆盖。
        old_lines = []
    with open(path, 'w') as fp:
        fp.write(expected_exposure + "\n")
        for line in old_lines:
            fp.write(line)
            if not line.endswith("\n"):
                fp.write("\n")
    with open(path, 'r') as fp:
        saved_exposure = fp.readline().strip()
    if saved_exposure != expected_exposure:
        raise Exception("曝光即时写入校验失败: %s" % path)
    print("[exp] 已立即写入 %s: %s" % (path, saved_exposure))

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


# sample_det() 在原 9 项宽覆盖统计后追加紧凑核心的 A/B 四分位与中位数。
CORE_A_LQ_I = 9
CORE_A_UQ_I = 10
CORE_B_LQ_I = 11
CORE_B_UQ_I = 12
CORE_A_MED_I = 13
CORE_B_MED_I = 14


def cap_ab_span(a0, a1, b0, b1, a_med, b_med, span_max):
    if a1 - a0 > span_max:
        a0 = max(a0, a_med - span_max / 2)
        a1 = min(a1, a_med + span_max / 2)
    if b1 - b0 > span_max:
        b0 = max(b0, b_med - span_max / 2)
        b1 = min(b1, b_med + span_max / 2)
    return a0, a1, b0, b1


def core_ab_bounds(m, extrapolate_lower=True):
    if len(m) <= CORE_B_MED_I:
        return None
    alq, auq = m[CORE_A_LQ_I], m[CORE_A_UQ_I]
    blq, buq = m[CORE_B_LQ_I], m[CORE_B_UQ_I]
    a_med, b_med = m[CORE_A_MED_I], m[CORE_B_MED_I]
    lower_k = IQR_K if extrapolate_lower else 0.0
    a0 = alq - lower_k * (auq - alq) - AB_MARGIN
    a1 = auq + IQR_K * (auq - alq) + AB_MARGIN
    b0 = blq - lower_k * (buq - blq) - AB_MARGIN
    b1 = buq + IQR_K * (buq - blq) + AB_MARGIN
    return a0, a1, b0, b1, a_med, b_med


def build_threshold(m, ground, gnd_box, gnd_boxes_for_conflict=None, slot=0):
    llq, luq, alq, auq, blq, buq = m[0], m[1], m[2], m[3], m[4], m[5]
    l_med, a_med, b_med = m[6], m[7], m[8]
    l0 = llq - IQR_K * (luq - llq)
    l1 = luq + IQR_K * (luq - llq)
    a0 = alq - IQR_K * (auq - alq) - AB_MARGIN
    a1 = auq + IQR_K * (auq - alq) + AB_MARGIN
    b0 = blq - IQR_K * (buq - blq) - AB_MARGIN
    b1 = buq + IQR_K * (buq - blq) + AB_MARGIN
    # 红袋和棕熊使用紧凑核心判断饱和度/地面冲突。宽覆盖边缘仍用于最终
    # 检测阈值，但不能因为少量阴影或背景把目标误判成低饱和/地面重叠。
    decision_a0, decision_a1 = a0, a1
    decision_b0, decision_b1 = b0, b1
    decision_a_med, decision_b_med = a_med, b_med
    core_bounds = core_ab_bounds(m, True) if slot in (2, 4) else None
    if core_bounds is not None:
        decision_a0, decision_a1, decision_b0, decision_b1 = core_bounds[0:4]
        decision_a_med, decision_b_med = core_bounds[4], core_bounds[5]
    if slot == 2:
        decision_a0, decision_a1, decision_b0, decision_b1 = cap_ab_span(
            decision_a0, decision_a1, decision_b0, decision_b1,
            decision_a_med, decision_b_med, AB_SPAN_MAX_BAG)
    elif slot == 4:
        decision_a0, decision_a1, decision_b0, decision_b1 = cap_ab_span(
            decision_a0, decision_a1, decision_b0, decision_b1,
            decision_a_med, decision_b_med, AB_SPAN_MAX_BRN_BEAR)

    def axis_dist(lo, hi):
        return 0.0 if lo <= 0 <= hi else min(abs(lo), abs(hi))
    low_sat = (axis_dist(decision_a0, decision_a1) < NEUTRAL_AB
               and axis_dist(decision_b0, decision_b1) < NEUTRAL_AB)
    ground_conflict_ls = []
    ground_conflict_boxes = []
    ground_near = GND_NEAR_BRN_BEAR if slot == 4 else GND_NEAR
    conflict_a0, conflict_a1 = decision_a0, decision_a1
    conflict_b0, conflict_b1 = decision_b0, decision_b1
    if slot != 3:
        for g in (gnd_boxes_for_conflict or ([gnd_box] if gnd_box else [])):
            if not g:
                continue
            ga0, ga1, gb0, gb1 = g[2], g[3], g[4], g[5]
            a_touch = not (conflict_a1 + ground_near < ga0
                           or ga1 + ground_near < conflict_a0)
            b_touch = not (conflict_b1 + ground_near < gb0
                           or gb1 + ground_near < conflict_b0)
            if a_touch and b_touch:
                ground_conflict_ls.append((g[0] + g[1]) * 0.5)
                ground_conflict_boxes.append(g)
    ground_in_ab = bool(ground_conflict_ls)
    if slot == 2:
        print("[thr] red_bag 核心AB判定=(%d,%d,%d,%d) low_sat=%d ground_overlap=%d" % (
            int(conflict_a0), int(conflict_a1),
            int(conflict_b0), int(conflict_b1),
            1 if low_sat else 0, 1 if ground_in_ab else 0))
    elif slot == 4:
        print("[thr] brn_bear AB复核=(%d,%d,%d,%d) ground_overlap=%d" % (
            int(conflict_a0), int(conflict_a1),
            int(conflict_b0), int(conflict_b1), 1 if ground_in_ab else 0))
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
    elif slot == 2:
        # 红袋远处会明显变暗。未与地面核心色冲突时仅放宽暗端，亮端仍用
        # 本次实测 IQR，避免原来的 (15,100) 把无关高亮区域一起纳入。
        old_l0 = l0
        l0 = max(0, min(l0, L_WIDE[0]))
        l1 = min(100, l1)
        print("[thr] red_bag 无地面冲突, L暗端放宽: (%d,%d) -> (%d,%d)" % (
            int(old_l0), int(l1), int(l0), int(l1)))
    else:
        l0, l1 = L_WIDE
    if slot == 3:
        ab_span_max = AB_SPAN_MAX_BALL
    elif slot in (1, 2):
        ab_span_max = AB_SPAN_MAX_BAG
    elif slot == 4:
        ab_span_max = AB_SPAN_MAX_BRN_BEAR
    elif slot == 5:
        ab_span_max = AB_SPAN_MAX_WHT_BEAR
    else:
        ab_span_max = AB_SPAN_MAX
    a0, a1, b0, b1 = cap_ab_span(
        a0, a1, b0, b1, a_med, b_med, ab_span_max)
    if l1 - l0 > L_SPAN_MAX and (l0, l1) != L_WIDE:
        l0 = max(l0, l_med - L_SPAN_MAX / 2)
        l1 = min(l1, l_med + L_SPAN_MAX / 2)
    return (int(l0), int(l1), int(a0), int(a1), int(b0), int(b1))

def tighten_slot_threshold(slot, t, m, ground_boxes=None):
    if slot == 1:
        b1 = min(t[5], int(m[8] + BLUE_BAG_B_MARGIN_HI))
        t = (t[0], t[1], t[2], t[3], t[4], max(t[4], b1))
    elif slot == 4:
        # 棕熊外围阴影只用于 L 覆盖，不能把 A/B 下界向中性或偏蓝方向
        # 外推。核心 Q1 减测量余量作为下界；核心上侧 IQR 仍可补足
        # 黄褐亮部，避免简单缩小总跨度后把 A_max 一并砍掉。
        core_bounds = core_ab_bounds(m, False)
        if core_bounds is not None:
            ca0, ca1, cb0, cb1, ca_med, cb_med = core_bounds
            ca0, ca1, cb0, cb1 = cap_ab_span(
                ca0, ca1, cb0, cb1, ca_med, cb_med,
                AB_SPAN_MAX_BRN_BEAR)
            a0 = max(t[2], int(ca0))
            a1 = max(t[3], int(ca1))
            b0 = max(t[4], int(cb0))
            a0 = min(a0, int(ca_med))
            a1 = max(a1, int(ca_med))
            b0 = min(b0, int(cb_med))
            a0 = max(-128, a0)
            a1 = min(127, a1)
            b0 = max(-128, b0)
            print("[thr] brn_bear 核心AB收紧: broad=(%d,%d,%d,%d) core=(%d,%d,%d,%d) final=(%d,%d,%d,%d)" % (
                t[2], t[3], t[4], t[5],
                int(ca0), int(ca1), int(cb0), int(cb1),
                a0, a1, b0, t[5]))
            t = (t[0], t[1], a0, a1, b0, t[5])
        l0 = max(t[0], int(m[6]) - BRN_BEAR_L_BELOW_MED)
        if l0 > t[0]:
            print("[thr] brn_bear L去阴影: " + str(t[0]) + " -> " + str(l0))
            t = (l0, t[1], t[2], t[3], t[4], t[5])
    elif slot == 5 and ground_boxes:
        l0 = t[0]
        for g in ground_boxes:
            a_overlap = not (t[3] < g[2] or g[3] < t[2])
            b_overlap = not (t[5] < g[4] or g[5] < t[4])
            if not (a_overlap and b_overlap):
                continue
            if m[6] <= (g[0] + g[1]) * 0.5:
                continue
            ground_cut = min(g[1] + WHT_BEAR_GND_L_GAP,
                             int(m[6]) - WHT_BEAR_CORE_L_PAD)
            l0 = max(l0, ground_cut)
        l0 = min(l0, t[1])
        if l0 > t[0]:
            print("[thr] wht_bear L避开地面: %d -> %d" % (t[0], l0))
            t = (l0, t[1], t[2], t[3], t[4], t[5])
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
        if gnd_boxes_for_edge:
            if any([in_box(lab, g) for g in gnd_boxes_for_edge]):
                continue
        if (abs(lab[1] - s[7]) > max_delta
                or abs(lab[2] - s[8]) > max_delta):
            continue
        if try_merge_bag_edge_stat(s, st) and CALIB_DRAW_DEBUG and draw_debug:
            img.draw_rectangle(eroi, color=(0, 180, 0))

def assign_slot(label, m, allowed_slots=None):
    l_med, a_med, b_med = m[6], m[7], m[8]
    if label == 1:
        return 3
    if label == 2:
        return 1 if b_med < -10 else 2
    if label == 0:
        if allowed_slots is not None:
            if 4 in allowed_slots and 5 not in allowed_slots:
                return 4
            if 5 in allowed_slots and 4 not in allowed_slots:
                return 5
        if (l_med >= WHT_BEAR_L_MIN and abs(a_med) <= WHT_BEAR_A_ABS_MAX
                and b_med <= WHT_BEAR_B_MAX):
            return 5
        return 4
    return 0

def slot_list_names(slots):
    names = []
    for slot in slots:
        names.append(SLOT_NAMES.get(slot, str(slot)))
    return "/".join(names)

def allowed_collect_slots(done_slots):
    if not CALIB_SLOT_PHASES:
        return None
    for phase in CALIB_SLOT_PHASES:
        remain = tuple([s for s in phase if s not in done_slots])
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

def sample_det(img, det, draw_debug=True, debug_boxes=None):
    """用模型框内种子寻找 LAB 边界，再在边界框内部取样。"""
    label, score, x, y, w, h = det
    x, y, w, h = refine_model_box_by_lab(img, label, x, y, w, h, gnd_boxes)
    if label == 1:
        rw = max(4, int(w * BALL_CORE_ROI_W_FRAC))
        rh = max(4, int(h * BALL_CORE_ROI_H_FRAC))
    elif label == 0:
        rw = max(4, int(w * SAMPLE_ROI_W_FRAC))
        rh = max(4, int(h * BEAR_SAMPLE_ROI_H_FRAC))
    else:
        rw = max(4, int(w * SAMPLE_ROI_W_FRAC))
        rh = max(4, int(h * SAMPLE_ROI_H_FRAC))
    rx = clamp_int(x + (w - rw) // 2, 0, 319)
    ry = clamp_int((y + int(h * BEAR_SAMPLE_ROI_Y_FRAC))
                   if label == 0 else y + (h - rh) // 2, 0, 239)
    roi = (rx, ry, min(rw, 320 - rx), min(rh, 240 - ry))
    st = img.get_statistics(roi=roi)
    s = [st.l_lq(), st.l_uq(), st.a_lq(), st.a_uq(), st.b_lq(), st.b_uq(),
         st.l_median(), st.a_median(), st.b_median()]
    core_roi = roi
    core_s = s
    if label == 0:
        # 熊类始终额外采中央上部核心。宽框可覆盖四肢/阴影，但 A/B 判定
        # 只相信不贴地的躯干核心，避免少量黑棕背景拉低四分位数。
        rw2 = max(4, int(w * BEAR_GND_RETRY_ROI_W_FRAC))
        rh2 = max(4, int(h * BEAR_GND_RETRY_ROI_H_FRAC))
        rx2 = clamp_int(x + (w - rw2) // 2, 0, 319)
        ry2 = clamp_int(y + int(h * BEAR_GND_RETRY_Y_FRAC), 0, 239)
        roi2 = (rx2, ry2, min(rw2, 320 - rx2), min(rh2, 240 - ry2))
        st2 = img.get_statistics(roi=roi2)
        s2 = [st2.l_lq(), st2.l_uq(), st2.a_lq(), st2.a_uq(), st2.b_lq(), st2.b_uq(),
              st2.l_median(), st2.a_median(), st2.b_median()]
        core_roi = roi2
        core_s = s2
        if (any([in_box((s[6], s[7], s[8]), g) for g in gnd_boxes])
                and not any([in_box((s2[6], s2[7], s2[8]), g) for g in gnd_boxes])):
            roi = roi2
            s = s2
    elif label == 2 and s[8] >= -10:
        # 红袋宽覆盖框会包含褶皱和边带；紧凑核心仅用于判断它是否真的
        # 低饱和或与蓝地重叠，避免边带污染触发错误的 L 收紧。
        crw = max(4, int(w * BAG_CORE_ROI_W_FRAC))
        crh = max(4, int(h * BAG_CORE_ROI_H_FRAC))
        crx = clamp_int(x + (w - crw) // 2, 0, 319)
        cry = clamp_int(y + (h - crh) // 2, 0, 239)
        core_roi = (crx, cry, min(crw, 320 - crx), min(crh, 240 - cry))
        cst = img.get_statistics(roi=core_roi)
        core_s = [cst.l_lq(), cst.l_uq(), cst.a_lq(), cst.a_uq(),
                  cst.b_lq(), cst.b_uq(), cst.l_median(),
                  cst.a_median(), cst.b_median()]
    if CALIB_DRAW_DEBUG and draw_debug:
        img.draw_rectangle((x, y, w, h), color=(0, 255, 255))  # 青框=LAB 边界框
        img.draw_rectangle(roi, color=(0, 255, 0))   # 绿框=实际取样区
        if core_roi != roi:
            img.draw_rectangle(core_roi, color=(255, 0, 255))  # 紫框=核心色取样区
    if debug_boxes is not None:
        debug_boxes.append(((x, y, w, h), (0, 255, 255)))
        debug_boxes.append((roi, (0, 255, 0)))
        if core_roi != roi:
            debug_boxes.append((core_roi, (255, 0, 255)))
    if label == 1:
        # 圆形目标的整框四角天然是地面。只补采球体内部的下部窄条带，
        # 并先做地面/离散度检查；中位数仍取中央核心区，防阈值中心漂向地面。
        bw = max(4, int(w * BALL_BOTTOM_ROI_W_FRAC))
        bh = max(3, int(h * BALL_BOTTOM_ROI_H_FRAC))
        bx = clamp_int(x + (w - bw) // 2, 0, 319)
        by = clamp_int(y + int(h * BALL_BOTTOM_ROI_Y_FRAC), 0, 239)
        broi = (bx, by, min(bw, 320 - bx), min(bh, 240 - by))
        st2 = img.get_statistics(roi=broi)
        bottom_lab = (st2.l_median(), st2.a_median(), st2.b_median())
        bottom_iqr = (st2.l_uq() - st2.l_lq(),
                      st2.a_uq() - st2.a_lq(),
                      st2.b_uq() - st2.b_lq())
        bottom_ground = any([in_box(bottom_lab, g) for g in gnd_boxes])
        bottom_clean = (not bottom_ground
                        and bottom_iqr[0] <= BALL_BOTTOM_IQR_L_MAX
                        and bottom_iqr[1] <= BALL_BOTTOM_IQR_AB_MAX
                        and bottom_iqr[2] <= BALL_BOTTOM_IQR_AB_MAX
                        and abs(bottom_lab[1] - s[7]) <= BALL_BOTTOM_AB_MED_DELTA
                        and abs(bottom_lab[2] - s[8]) <= BALL_BOTTOM_AB_MED_DELTA)
        strip_color = (0, 180, 0) if bottom_clean else (255, 0, 0)
        if CALIB_DRAW_DEBUG and draw_debug:
            img.draw_rectangle(broi, color=strip_color)
        if debug_boxes is not None:
            debug_boxes.append((broi, strip_color))
        if bottom_clean:
            merge_stat_tuple(s, st2)
        else:
            print("[ball] 下部条带未并入 med=(%d,%d,%d) IQR=(%d,%d,%d) ground=%d" % (
                bottom_lab[0], bottom_lab[1], bottom_lab[2],
                bottom_iqr[0], bottom_iqr[1], bottom_iqr[2],
                1 if bottom_ground else 0))
    elif label == 2:
        # LAB 边界框已经覆盖模型框外侧；边带只在不属于蓝地且接近核心色时并入。
        merge_bag_edge_strips(img, s, roi, gnd_boxes, draw_debug)
    return tuple(s) + (core_s[2], core_s[3], core_s[4], core_s[5],
                       core_s[7], core_s[8])

# ======================================================================
# 主流程
# ======================================================================
exposure = calibrate_exposure()
if WRITE_FILE:
    write_exposure_immediately(RESULT_PATH, exposure)

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
            if any([_rect_overlap(tile, eb) for eb in exclude_boxes]):
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
    # 某些 OpenART 固件的 min/max 不会消费生成器，而会把生成器本身返回。
    # 显式循环求范围，避免后续出现 "generator - float"。
    l_min = l_max = keep[0][0]
    a_min = a_max = keep[0][1]
    b_min = b_max = keep[0][2]
    for m in keep[1:]:
        l_min = min(l_min, m[0])
        l_max = max(l_max, m[0])
        a_min = min(a_min, m[1])
        a_max = max(a_max, m[1])
        b_min = min(b_min, m[2])
        b_max = max(b_max, m[2])
    box = (l_min - GB_L_M, l_max + GB_L_M,
           a_min - GB_AB_M, a_max + GB_AB_M,
           b_min - GB_AB_M, b_max + GB_AB_M)
    return (ml, ma, mb), box

img = sensor.snapshot()
# 采地面前先跑一次模型, 把在场物体的框(修正+外扩6px)从地面采样里排除
_ex = [correct_box(d[0], d[2], d[3], d[4], d[5], 6)
       for d in detect_boxes(img) if d[1] >= 0.15]
ground, gnd_box = make_gnd_box(img, METER_ROI, _ex, "near")   # 近处地面(主, 阈值生成用)
_, gnd_box_far = make_gnd_box(img, METER_ROI_FAR, _ex, "far")  # 远处地面(掠射角色度漂移)
if gnd_box is None:
    raise Exception("近处地面采样失败, 清空地面区重跑")
gnd_boxes = [gnd_box] + ([gnd_box_far] if gnd_box_far else [])
print("[gnd] near med=%s box=%s" % (ground, gnd_box))
print("[gnd] far  box=%s" % (gnd_box_far,))

def collect_one(exclude_slots, allowed_slots=None, require_swap_clear=True):
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
    red_bag_jump_box = None
    red_bag_jump_count = 0
    need_swap_clear = bool(exclude_slots) and require_swap_clear
    swap_clear_count = 0 if need_swap_clear else SWAP_CLEAR_FRAMES
    while len(samples) < SAMPLE_FRAMES:
        idle += 1
        if idle > SWAP_IDLE:
            return None
        img = sensor.snapshot()
        if CALIB_DRAW_DEBUG:
            img.draw_rectangle(METER_ROI, color=(120, 120, 0))
            img.draw_rectangle(METER_ROI_FAR, color=(120, 120, 0))
        dets = sorted(detect_boxes(img), key=lambda d: d[1], reverse=True)
        qualified_present = False
        for det in dets:
            if calibration_det_qualified(det):
                qualified_present = True
                break
        if not qualified_present:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            red_bag_jump_box = None
            red_bag_jump_count = 0
            if need_swap_clear and swap_clear_count < SWAP_CLEAR_FRAMES:
                swap_clear_count += 1
                idle = 0
                hint_tick += 1
                if hint_tick % 8 == 1:
                    print("[collect] swap clear " + str(swap_clear_count)
                          + "/" + str(SWAP_CLEAR_FRAMES)
                          + " wait=" + slot_list_names(allowed_slots)
                          + " done=" + slot_list_names(tuple(sorted(exclude_slots))))
                continue
            continue
        candidates = []
        reject_draws = []
        done_hint_slot = None
        phase_hint_name = None
        bad_hint = None
        for det in dets:
            label, score, x, y, w, h = det
            if not calibration_det_qualified(det):
                reject_draws.append(det)
                if bad_hint is None:
                    bad_hint = (label, score, w)
                continue
            if not label_possible_for_slots(label, allowed_slots):
                reject_draws.append(det)
                phase_hint_name = LABEL_NAMES.get(label, "?")
                continue
            sample_boxes = []
            s = sample_det(img, det, False, sample_boxes)
            reason = None
            # 网球=高光+绿边+底部暗边合并取样, L/AB 离散都单独放宽
            ab_lim = IQR_AB_MAX_BALL if label == 1 else IQR_AB_MAX
            l_lim = IQR_L_MAX
            gnd_hit = any([in_box((s[6], s[7], s[8]), g) for g in gnd_boxes])
            if s[6] >= SAMPLE_L_CLIP:
                reason = "过曝 Lmed=%d" % s[6]
            elif gnd_hit and not allow_ground_sample(label, s):
                reason = "采到地面"
            elif (s[1] - s[0]) > l_lim or (s[3] - s[2]) > ab_lim or (s[5] - s[4]) > ab_lim:
                reason = "分布过宽 IQR=" + triple_text(
                    s[1] - s[0], s[3] - s[2], s[5] - s[4])
            if reason:
                reject_draws.append(det)
                print("[det] " + LABEL_NAMES.get(label, "?")
                      + " score=" + score_text(score)
                      + " med=" + triple_text(s[6], s[7], s[8])
                      + " 丢弃: " + str(reason))
                continue
            elif gnd_hit:
                print("[det] blue_bag med=" + triple_text(s[6], s[7], s[8])
                      + " 命中地面盒但放行")
            slot = assign_slot(label, s, allowed_slots)
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
            candidates.append((slot, label, score, x, y, w, h, s, det, sample_boxes))

        if need_swap_clear and done_hint_slot is not None:
            swap_clear_count = 0
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            red_bag_track_box = None
            red_bag_jump_box = None
            red_bag_jump_count = 0
            samples = []
            target_slot = -1
            target_label = -1
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                print("[collect] wait swap: old "
                      + str(SLOT_NAMES.get(done_hint_slot, done_hint_slot))
                      + " still visible; wait=" + slot_list_names(allowed_slots)
                      + " samples=" + str(len(samples)))
            continue
        if need_swap_clear and swap_clear_count < SWAP_CLEAR_FRAMES:
            swap_clear_count = 0
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            red_bag_track_box = None
            red_bag_jump_box = None
            red_bag_jump_count = 0
            samples = []
            target_slot = -1
            target_label = -1
            idle = 0
            hint_tick += 1
            if hint_tick % 8 == 1:
                print("[collect] 请先移走上一物体, 空场确认 0/"
                      + str(SWAP_CLEAR_FRAMES) + "; 下一目标="
                      + slot_list_names(allowed_slots))
            continue

        if not candidates:
            for det in reject_draws:
                draw_model_box(img, det, ok=False)
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
            red_bag_jump_box = None
            red_bag_jump_count = 0
            hint_tick += 1
            if done_hint_slot is not None and hint_tick % 30 == 1:
                print("[det] " + str(SLOT_NAMES.get(done_hint_slot, done_hint_slot))
                      + " 已标定过, 请换下一个物体")
            elif phase_hint_name is not None and hint_tick % 30 == 1:
                print("[det] 当前等待 " + slot_list_names(allowed_slots)
                      + ", 忽略 " + str(phase_hint_name))
            elif bad_hint is not None and hint_tick % 30 == 1:
                label, score, w = bad_hint
                print("[det] " + LABEL_NAMES.get(label, "?")
                      + " score=" + score_text(score)
                      + " box_w=" + str(w) + " 不合格(低分/框小)")
            continue

        picked = max(candidates, key=lambda it: (collect_slot_priority(it[0], exclude_slots, allowed_slots), it[2]))
        draw_model_box(img, picked[8], ok=True)
        if CALIB_DRAW_DEBUG:
            for sample_box, sample_color in picked[9]:
                img.draw_rectangle(sample_box, color=sample_color)
        slot, label, score, x, y, w, h, s, det, sample_boxes = picked
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
                        print("[det] red_bag wait stable "
                              + str(red_bag_stable_count) + "/"
                              + str(RED_BAG_STABLE_FRAMES)
                              + " box=" + rect_text(x, y, w, h))
                    continue
                red_bag_track_box = box
            elif not red_bag_box_stable(red_bag_track_box, box,
                                        RED_BAG_TRACK_CENTER_PX,
                                        RED_BAG_TRACK_SIZE_PX):
                if red_bag_box_stable(red_bag_jump_box, box):
                    red_bag_jump_count += 1
                else:
                    red_bag_jump_count = 1
                red_bag_jump_box = box
                red_bag_stable_hint += 1
                idle = 0
                if red_bag_jump_count < RED_BAG_REBASE_FRAMES:
                    if red_bag_stable_hint % 8 == 1:
                        print("[det] red_bag box jump, rebase "
                              + str(red_bag_jump_count) + "/"
                              + str(RED_BAG_REBASE_FRAMES)
                              + " box=" + rect_text(x, y, w, h)
                              + " base=" + rect_text(
                                  red_bag_track_box[0], red_bag_track_box[1],
                                  red_bag_track_box[2], red_bag_track_box[3]))
                    continue
                print("[det] red_bag rebase " + rect_text(
                    red_bag_track_box[0], red_bag_track_box[1],
                    red_bag_track_box[2], red_bag_track_box[3])
                    + " -> " + rect_text(x, y, w, h) + ", 样本重新计数")
                samples = []
                red_bag_track_box = box
                red_bag_jump_box = None
                red_bag_jump_count = 0
            else:
                red_bag_jump_box = None
                red_bag_jump_count = 0
            red_bag_track_box = box
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
        elif slot == 4 or slot == 5:
            box = (x, y, w, h)
            if slot == 4:
                stable = brn_bear_box_stable(brn_bear_stable_box, box)
                stable_frames = BRN_BEAR_STABLE_FRAMES
            else:
                stable = red_bag_box_stable(
                    brn_bear_stable_box, box,
                    WHT_BEAR_STABLE_CENTER_PX, WHT_BEAR_STABLE_SIZE_PX)
                stable_frames = WHT_BEAR_STABLE_FRAMES
            if stable:
                brn_bear_stable_count += 1
            else:
                brn_bear_stable_count = 1
            brn_bear_stable_box = box
            if brn_bear_stable_count < stable_frames:
                brn_bear_stable_hint += 1
                idle = 0
                if brn_bear_stable_hint % 8 == 1:
                    print("[det] " + str(SLOT_NAMES.get(slot, slot))
                          + " wait stable " + str(brn_bear_stable_count)
                          + "/" + str(stable_frames)
                          + " box=" + rect_text(x, y, w, h))
                continue
        else:
            brn_bear_stable_box = None
            brn_bear_stable_count = 0
            red_bag_stable_box = None
            red_bag_stable_count = 0
        samples.append(s)
        idle = 0
        print("[det] " + str(SLOT_NAMES.get(slot, slot))
              + " score=" + score_text(score)
              + " box=" + rect_text(x, y, w, h)
              + " med=" + triple_text(s[6], s[7], s[8])
              + " 样本 " + str(len(samples)) + "/" + str(SAMPLE_FRAMES))
    m = []
    for channel in range(len(samples[0])):
        channel_values = []
        for sample in samples:
            channel_values.append(sample[channel])
        m.append(median(channel_values))
    print("[thr] %s 宽统计=(%d,%d,%d,%d,%d,%d) med=(%d,%d,%d)" % (
        SLOT_NAMES.get(target_slot, target_slot),
        int(m[0]), int(m[1]), int(m[2]), int(m[3]), int(m[4]), int(m[5]),
        int(m[6]), int(m[7]), int(m[8])))
    if len(m) > CORE_B_MED_I:
        print("[thr] %s 核心AB=(%d,%d,%d,%d) med=(%d,%d)" % (
            SLOT_NAMES.get(target_slot, target_slot),
            int(m[CORE_A_LQ_I]), int(m[CORE_A_UQ_I]),
            int(m[CORE_B_LQ_I]), int(m[CORE_B_UQ_I]),
            int(m[CORE_A_MED_I]), int(m[CORE_B_MED_I])))
    t = tighten_slot_threshold(
        target_slot,
        build_threshold(m, ground, gnd_box, gnd_boxes, target_slot),
        m, gnd_boxes)
    med = (float(m[6]), float(m[7]), float(m[8]))
    return (target_slot, t, med, len(samples))


def verify_expected_label(slot):
    if slot == 3:
        return 1
    if slot == 1 or slot == 2:
        return 2
    if slot == 4 or slot == 5:
        return 0
    return -1


def verify_rect_intersection(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[0] + a[2], b[0] + b[2])
    y2 = min(a[1] + a[3], b[1] + b[3])
    if x2 <= x1 or y2 <= y1:
        return 0
    return (x2 - x1) * (y2 - y1)


def verify_pick_model(dets, expected_label, previous_box):
    best = None
    best_score = None
    for det in dets:
        if det[0] != expected_label or not calibration_det_qualified(det):
            continue
        score = int(det[1] * 10000)
        if previous_box is not None:
            pcx = previous_box[0] + previous_box[2] // 2
            pcy = previous_box[1] + previous_box[3] // 2
            dcx = det[2] + det[4] // 2
            dcy = det[3] + det[5] // 2
            dx = dcx - pcx
            dy = dcy - pcy
            dist2 = dx * dx + dy * dy
            if dist2 <= 80 * 80:
                score += 100000
            score -= dist2
        if best is None or score > best_score:
            best = det
            best_score = score
    return best


def verify_valid_color_blob(blob, slot):
    w = blob.w()
    h = blob.h()
    pixels_threshold, area_threshold = VERIFY_BLOB_LIMITS[slot - 1]
    if (w <= 0 or h <= 0 or blob.pixels() < pixels_threshold
            or w * h < area_threshold):
        return False
    if slot == 3:
        return (w * 100 >= h * 45 and w * 100 <= h * 185
                and blob.density() >= 0.35)
    if slot == 4 or slot == 5:
        return (w * 100 >= h * 30 and w * 100 <= h * 250
                and blob.density() >= 0.25)
    # if slot == 2:  # 红沙包专用长宽比过滤已停用
    #     return (w * 100 >= h * 60
    #             and w * 100 <= h * RED_BAG_MAX_WIDTH_HEIGHT_X100
    #             and blob.density() >= 0.40)
    return (w * 100 >= h * 60 and w * 100 <= h * 180
            and blob.density() >= 0.40)


def verify_find_blobs(img, slot, threshold):
    pixels_threshold, area_threshold = VERIFY_BLOB_LIMITS[slot - 1]
    roi = (0, DETECT_Y_MIN, 320, VERIFY_COLOR_Y_MAX - DETECT_Y_MIN)
    blobs = None
    try:
        if slot == 4 or slot == 5:
            margin = (BRN_BEAR_MERGE_MARGIN if slot == 4
                      else WHT_BEAR_MERGE_MARGIN)
            try:
                blobs = img.find_blobs(
                    [threshold], roi=roi, pixels_threshold=pixels_threshold,
                    area_threshold=area_threshold, merge=True,
                    margin=margin)
            except TypeError:
                blobs = img.find_blobs(
                    [threshold], roi=roi, pixels_threshold=pixels_threshold,
                    area_threshold=area_threshold, merge=True)
        else:
            blobs = img.find_blobs(
                [threshold], roi=roi, pixels_threshold=pixels_threshold,
                area_threshold=area_threshold, merge=False)
    except Exception as error:
        print("[verify] find_blobs 失败: " + str(error))
        return []
    valid = []
    if blobs:
        for blob in blobs:
            if verify_valid_color_blob(blob, slot):
                valid.append(blob)
    return valid


def verify_pick_blob(blobs, slot, previous_box, model_box):
    best = None
    best_score = None
    mcx = model_box[0] + model_box[2] // 2
    mcy = model_box[1] + model_box[3] // 2
    for blob in blobs:
        box = blob.rect()
        overlap = verify_rect_intersection(model_box, box)
        if overlap <= 0:
            continue
        dx = blob.cx() - mcx
        dy = blob.cy() - mcy
        score = overlap * 1000 + blob.pixels() * 4 - dx * dx - dy * dy
        if best is None or score > best_score:
            best = blob
            best_score = score
    if best is not None:
        return best

    if slot == 5 and previous_box is not None:
        pcx = previous_box[0] + previous_box[2] // 2
        pcy = previous_box[1] + previous_box[3] // 2
        for blob in blobs:
            box = blob.rect()
            dx = blob.cx() - pcx
            dy = blob.cy() - pcy
            dist2 = dx * dx + dy * dy
            inter = verify_rect_intersection(previous_box, box)
            union = (previous_box[2] * previous_box[3]
                     + box[2] * box[3] - inter)
            iou_scaled = inter * 100000 // max(1, union)
            if dist2 > WHT_BEAR_SMOOTH_MAX_JUMP2 and iou_scaled < 5000:
                continue
            score = iou_scaled - dist2 + blob.pixels() // 8
            if best is None or score > best_score:
                best = blob
                best_score = score
    if best is not None:
        return best

    best_distance = None
    for blob in blobs:
        distance = 240 - (blob.y() + blob.h())
        if (best is None or distance < best_distance
                or (distance == best_distance
                    and (blob.x() < best.x()
                         or (blob.x() == best.x()
                             and blob.pixels() > best.pixels())))):
            best = blob
            best_distance = distance
    return best


def verify_box_signature(model_box, color_box):
    mx, my, mw, mh = model_box
    bx, by, bw, bh = color_box
    mcx2 = mx * 2 + mw
    mcy2 = my * 2 + mh
    bcx2 = bx * 2 + bw
    bcy2 = by * 2 + bh
    model_area = max(1, mw * mh)
    return (int((bcx2 - mcx2) * 50 / max(1, mw)),
            int((bcy2 - mcy2) * 50 / max(1, mh)),
            int(bw * 100 / max(1, mw)),
            int(bh * 100 / max(1, mh)),
            int(bw * bh * 100 / model_area))


def verify_signature_jump(previous, current):
    if previous is None:
        return None
    if (abs(current[0] - previous[0]) > VERIFY_REL_CENTER_JUMP_PERCENT
            or abs(current[1] - previous[1]) > VERIFY_REL_CENTER_JUMP_PERCENT):
        return "色块相对模型中心突跳"
    if (abs(current[2] - previous[2]) > VERIFY_REL_SIZE_JUMP_PERCENT
            or abs(current[3] - previous[3]) > VERIFY_REL_SIZE_JUMP_PERCENT):
        return "色块宽高突跳"
    area_lo = min(previous[4], current[4])
    area_hi = max(previous[4], current[4])
    if area_hi * 100 > max(1, area_lo) * VERIFY_AREA_JUMP_PERCENT:
        return "色块面积突跳"
    return None


def verify_box_mismatch(model_box, color_box):
    mx, my, mw, mh = model_box
    bx, by, bw, bh = color_box
    model_area = max(1, mw * mh)
    color_area = max(1, bw * bh)
    inter = verify_rect_intersection(model_box, color_box)
    union = model_area + color_area - inter
    iou_percent = int(inter * 100 / max(1, union))
    area_percent = int(color_area * 100 / model_area)
    dx2 = (bx * 2 + bw) - (mx * 2 + mw)
    dy2 = (by * 2 + bh) - (my * 2 + mh)
    center_distance = int(((dx2 * dx2 + dy2 * dy2) ** 0.5) / 2 + 0.5)
    center_limit = max(12, max(mw, mh) * VERIFY_CENTER_MAX_PERCENT // 100)
    min_area = min(model_area, color_area)

    reason = None
    if inter == 0 and center_distance > center_limit:
        reason = "模型框与色块框完全不同"
    elif (inter * 100 < min_area * VERIFY_MIN_OVERLAP_PERCENT
          and center_distance > center_limit):
        reason = "模型框与色块框重叠过低"
    elif area_percent < VERIFY_MIN_AREA_PERCENT:
        reason = "色块框面积只剩模型框很小一部分"
    elif area_percent > VERIFY_MAX_AREA_PERCENT:
        reason = "色块框吞入大块背景"
    elif (bw * 100 < mw * VERIFY_MIN_SIDE_PERCENT
          or bh * 100 < mh * VERIFY_MIN_SIDE_PERCENT):
        reason = "色块框只覆盖目标局部"
    return reason, iou_percent, area_percent, center_distance


def verify_signature_spread(sig_min, sig_max):
    if sig_min is None or sig_max is None:
        return "没有可用的色块稳定性样本"
    if (sig_max[0] - sig_min[0] > VERIFY_SPREAD_CENTER_PERCENT
            or sig_max[1] - sig_min[1] > VERIFY_SPREAD_CENTER_PERCENT):
        return "色块相对模型中心来回跳"
    if (sig_max[2] - sig_min[2] > VERIFY_SPREAD_SIZE_PERCENT
            or sig_max[3] - sig_min[3] > VERIFY_SPREAD_SIZE_PERCENT):
        return "色块宽高来回跳"
    if sig_max[4] * 100 > max(1, sig_min[4]) * VERIFY_AREA_JUMP_PERCENT:
        return "色块面积来回跳"
    return None


def verify_slot_threshold(slot, threshold):
    expected_label = verify_expected_label(slot)
    compared = 0
    good = 0
    bad_total = 0
    bad_streak = 0
    jumps = 0
    ticks = 0
    previous_model_box = None
    previous_signature = None
    sig_min = None
    sig_max = None
    previous_color_box = None
    last_reason = "复检超时"
    print("[verify] 开始复检 %s: 目标保持不动，共需 %d 个对照帧" % (
        SLOT_NAMES.get(slot, slot), VERIFY_FRAMES))

    while compared < VERIFY_FRAMES and ticks < VERIFY_TIMEOUT:
        ticks += 1
        img = sensor.snapshot()
        pick = verify_pick_model(detect_boxes(img), expected_label, previous_model_box)
        if pick is None:
            if ticks % 15 == 0:
                print("[verify] 等待模型检出 %s (%d/%d)" % (
                    SLOT_NAMES.get(slot, slot), compared, VERIFY_FRAMES))
            continue
        previous_model_box = (pick[2], pick[3], pick[4], pick[5])
        model_box = refine_model_box_by_lab(
            img, pick[0], pick[2], pick[3], pick[4], pick[5], gnd_boxes)
        verify_blobs = verify_find_blobs(img, slot, threshold)
        blob = verify_pick_blob(
            verify_blobs, slot, previous_color_box, model_box)
        compared += 1

        if CALIB_DRAW_DEBUG:
            draw_model_box(img, pick, True)
            img.draw_rectangle(model_box, color=(0, 255, 255))

        color_box = None
        if blob is None:
            reason = "模型有目标但新阈值找不到色块"
            iou_percent = 0
            area_percent = 0
            center_distance = 999
            jump_reason = None
        else:
            color_box = blob.rect()
            previous_color_box = color_box
            if CALIB_DRAW_DEBUG:
                img.draw_rectangle(color_box, color=DRAW_COLORS.get(slot, (255, 255, 255)))
            reason, iou_percent, area_percent, center_distance = verify_box_mismatch(
                model_box, color_box)
            signature = verify_box_signature(model_box, color_box)
            jump_reason = None if reason else verify_signature_jump(
                previous_signature, signature)
            if reason is None:
                previous_signature = signature
                if sig_min is None:
                    sig_min = list(signature)
                    sig_max = list(signature)
                else:
                    for i in range(len(signature)):
                        sig_min[i] = min(sig_min[i], signature[i])
                        sig_max[i] = max(sig_max[i], signature[i])

        frame_reason = reason if reason else jump_reason
        if frame_reason:
            bad_total += 1
            bad_streak += 1
            last_reason = frame_reason
            if jump_reason:
                jumps += 1
            print("[verify] !! %s %d/%d: %s; iou=%d%% area=%d%% cd=%d" % (
                SLOT_NAMES.get(slot, slot), compared, VERIFY_FRAMES,
                frame_reason, iou_percent, area_percent, center_distance))
            print("[verify] boxes model=" + str(model_box)
                  + " color=" + str(color_box)
                  + " candidates=" + str(len(verify_blobs)))
        else:
            good += 1
            bad_streak = 0
            print("[verify] OK %s %d/%d iou=%d%% area=%d%% cd=%d" % (
                SLOT_NAMES.get(slot, slot), compared, VERIFY_FRAMES,
                iou_percent, area_percent, center_distance))

        if (bad_streak >= VERIFY_BAD_STREAK_LIMIT
                or bad_total >= VERIFY_BAD_TOTAL_LIMIT
                or jumps >= VERIFY_JUMP_LIMIT):
            print("[verify] FAIL %s: bad=%d streak=%d jumps=%d" % (
                SLOT_NAMES.get(slot, slot), bad_total, bad_streak, jumps))
            return False, last_reason

    if compared < VERIFY_FRAMES:
        return False, "复检期间模型有效帧不足"
    if good < VERIFY_FRAMES - VERIFY_BAD_TOTAL_LIMIT + 1:
        return False, last_reason
    spread_reason = verify_signature_spread(sig_min, sig_max)
    if spread_reason:
        return False, spread_reason
    print("[verify] PASS %s: good=%d bad=%d jumps=%d" % (
        SLOT_NAMES.get(slot, slot), good, bad_total, jumps))
    return True, None


def banner(slot, t, med, n, ok, failure_reason=None):
    print("")
    print("##############################################")
    print("###")
    if ok:
        print("###   %s 标定完成 (%d 帧样本)" % (SLOT_NAMES.get(slot, slot), n))
        print("###")
        print("###   阈值 = %s" % (t,))
        print("###   med = (%d, %d, %d)   exposure = %dus" % (med[0], med[1], med[2], exposure))
    elif failure_reason:
        print("###   !! %s 复检失败，当前阈值作废 !!" % SLOT_NAMES.get(slot, slot))
        print("###   原因 = %s" % failure_reason)
        print("###   阈值 = %s" % (t,))
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

retry_slot = 0
while len(thresholds) < max_objects:
    done_slots = set(thresholds)
    if retry_slot:
        allowed_slots = (retry_slot,)
        print(">>> 复检未通过，保持 %s 在画面中，立即重新标定 <<<" %
              SLOT_NAMES.get(retry_slot, retry_slot))
    else:
        allowed_slots = allowed_collect_slots(done_slots) if CALIB_MODE == "multi" else None
    if allowed_slots is not None:
        print(">>> 当前等待: %s <<<" % slot_list_names(allowed_slots))
    r = collect_one(done_slots, allowed_slots, not retry_slot)
    if r is None:
        if thresholds:
            print(">>> %d 帧没等到新物体, 结束采样 <<<" % SWAP_IDLE)
        else:
            print("!!!!! 采样失败: 没采到任何合格样本 !!!!!")
            print("检查: 物体是否够近(框宽>=30px)? [det] 打印的丢弃原因?")
        break
    slot, t, med, n = r
    ground_ok = not in_box(ground, t)
    verify_ok = False
    verify_reason = None
    if ground_ok:
        verify_ok, verify_reason = verify_slot_threshold(slot, t)
    ok = ground_ok and verify_ok
    banner(slot, t, med, n, ok, verify_reason)
    if ok:
        thresholds[slot] = t
        slot_meds[slot] = med
        retry_slot = 0
    else:
        retry_slot = slot
        if not ground_ok:
            print(">>> 阈值命中地面，保持当前物体，立即重新采样 <<<")
        else:
            print(">>> 复检失败，当前阈值未保存，立即重新采样 <<<")
        continue
    if len(thresholds) < max_objects:
        allowed_slots = allowed_collect_slots(set(thresholds)) if CALIB_MODE == "multi" else None
        done_msg = ", ".join([SLOT_NAMES[s] for s in sorted(thresholds)])
        if allowed_slots is not None:
            print(">>> 请先清空画面；空场确认 8/8 后再放 %s (已完成: %s) <<<" % (
                slot_list_names(allowed_slots), done_msg))
        else:
            print(">>> 请先清空画面再放下一物体 (已完成: %s) <<<" % done_msg)

# 互吃 + 冲突分离(多物体时)
def scalar_median(value, slot, channel):
    unwrap_count = 0
    while isinstance(value, (tuple, list)):
        if len(value) != 1:
            raise Exception("slot " + str(slot) + " channel " + str(channel)
                            + " 中位数不是标量: " + str(value))
        value = value[0]
        unwrap_count += 1
        if unwrap_count > 3:
            raise Exception("slot " + str(slot) + " 中位数嵌套异常")
    try:
        return float(value)
    except Exception:
        raise Exception("slot " + str(slot) + " channel " + str(channel)
                        + " 中位数无法转数字: " + str(value))

slots = sorted(thresholds)
for i in range(len(slots)):
    for j in range(i + 1, len(slots)):
        sa, sb = slots[i], slots[j]
        if sa not in thresholds or sb not in thresholds:
            continue
        if sa == 4 and sb == 5:
            continue
        t1, t2 = thresholds[sa], thresholds[sb]
        if not (not (t1[3] < t2[2] or t2[3] < t1[2] or t1[5] < t2[4] or t2[5] < t1[4])):
            continue
        if t1[1] < t2[0] or t2[1] < t1[0]:
            continue
        ma, mb = slot_meds[sa], slot_meds[sb]
        try:
            ma0 = scalar_median(ma[0], sa, 0)
            ma1 = scalar_median(ma[1], sa, 1)
            ma2 = scalar_median(ma[2], sa, 2)
            mb0 = scalar_median(mb[0], sb, 0)
            mb1 = scalar_median(mb[1], sb, 1)
            mb2 = scalar_median(mb[2], sb, 2)
        except Exception as error:
            print("[thr] 跳过 " + str(sa) + "<->" + str(sb)
                  + " 冲突分离: " + str(error))
            continue
        gap0 = abs(ma0 - mb0)
        gap1 = abs(ma1 - mb1)
        gap2 = abs(ma2 - mb2)
        k = 0
        gap = gap0
        va, vb = ma0, mb0
        if gap1 > gap:
            k = 1
            gap = gap1
            va, vb = ma1, mb1
        if gap2 > gap:
            k = 2
            gap = gap2
            va, vb = ma2, mb2
        if gap < 8:
            print("[thr] !! %s<->%s 分不开, 双双废弃" % (SLOT_NAMES[sa], SLOT_NAMES[sb]))
            del thresholds[sa]
            del thresholds[sb]
            continue
        mid = (va + vb) * 0.5
        lo_i, hi_i = k * 2, k * 2 + 1
        lower, higher = (sa, sb) if va < vb else (sb, sa)
        tl = list(thresholds[lower]); th2 = list(thresholds[higher])
        tl[hi_i] = min(tl[hi_i], int(mid - 1))
        th2[lo_i] = max(th2[lo_i], int(mid + 1))
        thresholds[lower] = tuple(tl); thresholds[higher] = tuple(th2)
        print("[thr] 冲突分离 %s<->%s: 通道%s 切于 %.0f" % (
            SLOT_NAMES[sa], SLOT_NAMES[sb], "LAB"[k], mid))


def threshold_boxes_overlap(t1, t2):
    for channel in range(3):
        lo_i = channel * 2
        hi_i = lo_i + 1
        if t1[hi_i] < t2[lo_i] or t2[hi_i] < t1[lo_i]:
            return False
    return True


def threshold_ranges_valid(threshold):
    for channel in range(3):
        lo_i = channel * 2
        if threshold[lo_i] > threshold[lo_i + 1]:
            return False
    return True


def threshold_channel_contains(threshold, channel, value):
    lo_i = channel * 2
    return threshold[lo_i] <= value <= threshold[lo_i + 1]


def separate_bear_thresholds():
    if 4 not in thresholds or 5 not in thresholds:
        return
    brown_med = []
    white_med = []
    try:
        for channel in range(3):
            brown_med.append(scalar_median(slot_meds[4][channel], 4, channel))
            white_med.append(scalar_median(slot_meds[5][channel], 5, channel))
    except Exception as error:
        print("[bear] !! 中位数异常，棕白熊阈值作废: " + str(error))
        del thresholds[4]
        del thresholds[5]
        return

    if not threshold_boxes_overlap(thresholds[4], thresholds[5]):
        print("[bear] PASS 阈值已经分离 brn=" + str(thresholds[4])
              + " wht=" + str(thresholds[5]) + " overlap=0")
        return

    # 棕熊 A/B 阈值来自躯干核心，但 slot_meds 的 A/B 来自更宽取样区。
    # 只允许中位数已落在各自阈值内的通道参与切分，避免用受阴影污染的
    # 宽区 A/B 中位数切坏已经通过 10 帧色块复检的核心阈值。
    split_channel = -1
    split_gap = -1
    for channel in range(3):
        if not threshold_channel_contains(
                thresholds[4], channel, brown_med[channel]):
            continue
        if not threshold_channel_contains(
                thresholds[5], channel, white_med[channel]):
            continue
        gap = abs(brown_med[channel] - white_med[channel])
        if split_channel < 0 or gap > split_gap:
            split_channel = channel
            split_gap = gap
    if split_channel < 0:
        print("[bear] !! 没有可安全切分的 LAB 通道，两个阈值均不写入")
        print("[bear] brn=" + str(thresholds[4])
              + " med=" + str(tuple(brown_med)))
        print("[bear] wht=" + str(thresholds[5])
              + " med=" + str(tuple(white_med)))
        del thresholds[4]
        del thresholds[5]
        return
    if split_gap < BEAR_SEPARATION_MIN_GAP:
        print("[bear] !! 棕白熊 LAB 中位数过近 gap=%d，两个阈值均不写入" % int(split_gap))
        del thresholds[4]
        del thresholds[5]
        return

    brown_value = brown_med[split_channel]
    white_value = white_med[split_channel]
    lower_slot, higher_slot = ((4, 5) if brown_value < white_value else (5, 4))
    cut = int((brown_value + white_value) * 0.5)
    lo_i = split_channel * 2
    hi_i = lo_i + 1
    lower_threshold = list(thresholds[lower_slot])
    higher_threshold = list(thresholds[higher_slot])
    lower_threshold[hi_i] = min(
        lower_threshold[hi_i], cut - BEAR_SEPARATION_MARGIN)
    higher_threshold[lo_i] = max(
        higher_threshold[lo_i], cut + BEAR_SEPARATION_MARGIN)
    thresholds[lower_slot] = tuple(lower_threshold)
    thresholds[higher_slot] = tuple(higher_threshold)

    brown_med_box = (brown_med[0], brown_med[1], brown_med[2])
    white_med_box = (white_med[0], white_med[1], white_med[2])
    ranges_ok = (threshold_ranges_valid(thresholds[4])
                 and threshold_ranges_valid(thresholds[5]))
    split_meds_ok = (threshold_channel_contains(
                         thresholds[4], split_channel,
                         brown_med[split_channel])
                     and threshold_channel_contains(
                         thresholds[5], split_channel,
                         white_med[split_channel]))
    disjoint_ok = not threshold_boxes_overlap(thresholds[4], thresholds[5])
    # 仅作诊断，不作为失败条件：宽取样区 A/B 中位数可能合理地落在按
    # 躯干核心收紧后的棕熊阈值之外，实际覆盖已由逐槽 10 帧复检确认。
    full_meds_hint = (in_box(brown_med_box, thresholds[4])
                      and in_box(white_med_box, thresholds[5]))
    valid = ranges_ok and split_meds_ok and disjoint_ok
    print("[bear] 强制分离 channel=%s cut=%d gap=%d brown_med=(%d,%d,%d) white_med=(%d,%d,%d)" % (
        "LAB"[split_channel], cut, int(split_gap),
        int(brown_med[0]), int(brown_med[1]), int(brown_med[2]),
        int(white_med[0]), int(white_med[1]), int(white_med[2])))
    print("[bear] audit ranges=" + str(1 if ranges_ok else 0)
          + " split_meds=" + str(1 if split_meds_ok else 0)
          + " disjoint=" + str(1 if disjoint_ok else 0)
          + " full_meds_hint=" + str(1 if full_meds_hint else 0))
    print("[bear] final brn=" + str(thresholds[4])
          + " wht=" + str(thresholds[5]))
    if not valid:
        print("[bear] !! 分离后审计失败，两个阈值均不写入")
        del thresholds[4]
        del thresholds[5]
        return
    print("[bear] PASS brn=%s wht=%s overlap=0" % (
        thresholds[4], thresholds[5]))


separate_bear_thresholds()

print("=" * 46)
print("最终结果  exposure_us=%d" % exposure)
print("wb_gains=(%.2f, %.2f, %.2f)" % startup_wb_gains)
print("ground=%s" % (gnd_box,))
print("ground_far=%s" % (gnd_box_far,))
for slot in sorted(thresholds):
    print("%s = %s" % (SLOT_NAMES[slot], thresholds[slot]))
print("=" * 46)

def format_calibration_lab(values, tag):
    try:
        count = len(values)
    except Exception:
        raise Exception("%s 不是 LAB 六元组" % tag)
    if count != 6:
        raise Exception("%s 应有 6 项, 实际 %d 项" % (tag, count))
    text = ""
    for i in range(6):
        if i:
            text += ","
        text += str(int(values[i]))
    return text

def format_wb_gains(values):
    gains = validate_wb_gains(values)
    return "%.2f,%.2f,%.2f" % gains

def write_calibration_result(path, exposure_us, wb_gains,
                             near_ground, far_ground, rows):
    expected_exposure = "exposure_us=" + str(int(exposure_us))
    expected_wb = "wb_gains=" + format_wb_gains(wb_gains)
    with open(path, 'w') as fp:
        fp.write(expected_exposure + "\n")
        fp.write(expected_wb + "\n")
        fp.write("ground=" + format_calibration_lab(near_ground, "ground") + "\n")
        if far_ground:
            fp.write("ground2=" + format_calibration_lab(far_ground, "ground2") + "\n")
        for slot in sorted(rows):
            fp.write(str(int(slot)) + "," + format_calibration_lab(
                rows[slot], "slot " + str(slot)) + "\n")
    with open(path, 'r') as fp:
        saved_exposure = fp.readline().strip()
        saved_wb = fp.readline().strip()
    if saved_exposure != expected_exposure:
        raise Exception("曝光写入校验失败: %s" % path)
    if saved_wb != expected_wb:
        raise Exception("白平衡写入校验失败: %s" % path)
    print("已写并校验 %s: %s, %s, colors=%d/5" % (
        path, saved_exposure, saved_wb, len(rows)))

if WRITE_FILE:
    if len(thresholds) == 5:
        write_calibration_result(
            RESULT_PATH, exposure, startup_wb_gains,
            gnd_box, gnd_box_far, thresholds)
    else:
        write_calibration_result(
            PARTIAL_RESULT_PATH, exposure, startup_wb_gains,
            gnd_box, gnd_box_far, thresholds)
        print("正式配置未覆盖: 需要完整 5 个颜色槽位，当前=%d" % len(thresholds))

# ---------- 动态分界线(与 main.py/minimain.py 的 ground 平均方式一致) ----------
def average_ground_threshold(ground_a, ground_b):
    if ground_a and ground_b:
        averaged = []
        for i in range(6):
            averaged.append((ground_a[i] + ground_b[i]) // 2)
        return tuple(averaged)
    return ground_a if ground_a else ground_b

BLUE_GROUND_THRESHOLD = [average_ground_threshold(gnd_box, gnd_box_far)]
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
CUT_Y_MARGIN = 10
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2
cut_left_y = DETECT_Y_MIN
cut_right_y = DETECT_Y_MIN
cut_valid = False
cut_miss = 0

def cut_line_y_at_x(x):
    return cut_left_y   # 水平裁切线, 全画面统一

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

def stabilize_white_bear_box(previous, current):
    if previous is None:
        return current
    pcx = previous[0] + previous[2] // 2
    pcy = previous[1] + previous[3] // 2
    ccx = current[0] + current[2] // 2
    ccy = current[1] + current[3] // 2
    dx = pcx - ccx
    dy = pcy - ccy
    if (dx * dx + dy * dy > WHT_BEAR_SMOOTH_MAX_JUMP2
            and box_iou(previous, current) < 0.05):
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

RUNTIME_BLOB_LIMITS = VERIFY_BLOB_LIMITS
RUNTIME_COLOR_Y_MAX = VERIFY_COLOR_Y_MAX
RT_RAW = 0
RT_AMBIG = 1
RT_SMALL = 2
RT_SHAPE = 3
RT_CUT = 4
RT_OK = 5
RT_LEN = 6

def runtime_color_id_from_code(code):
    if code <= 0 or code & (code - 1):
        return 0
    color_id = 1
    while code > 1:
        code >>= 1
        color_id += 1
    return color_id if color_id <= 5 else 0

def runtime_valid_color_blob(blob, color_id):
    w = blob.w()
    h = blob.h()
    pixels_threshold, area_threshold = RUNTIME_BLOB_LIMITS[color_id - 1]
    if (w <= 0 or h <= 0 or blob.pixels() < pixels_threshold
            or w * h < area_threshold):
        return False
    if color_id == 3:
        return (w * 100 >= h * 45 and w * 100 <= h * 185
                and blob.density() >= 0.35)
    if color_id == 4 or color_id == 5:
        return (w * 100 >= h * 30 and w * 100 <= h * 250
                and blob.density() >= 0.25)
    # if color_id == 2:  # 红沙包专用长宽比过滤已停用
    #     return (w * 100 >= h * 60
    #             and w * 100 <= h * RED_BAG_MAX_WIDTH_HEIGHT_X100
    #             and blob.density() >= 0.40)
    return (w * 100 >= h * 60 and w * 100 <= h * 180
            and blob.density() >= 0.40)

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

def runtime_merged_bear_blobs(img, roi, slot):
    if slot not in (4, 5) or slot not in thresholds:
        return None
    pixels_threshold, area_threshold = RUNTIME_BLOB_LIMITS[slot - 1]
    margin = (BRN_BEAR_MERGE_MARGIN if slot == 4
              else WHT_BEAR_MERGE_MARGIN)
    try:
        try:
            return img.find_blobs([thresholds[slot]], roi=roi,
                                  pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True,
                                  margin=margin)
        except TypeError:
            return img.find_blobs([thresholds[slot]], roi=roi,
                                  pixels_threshold=pixels_threshold,
                                  area_threshold=area_threshold, merge=True)
    except Exception:
        return None

def runtime_preview_candidates(img, roi):
    dbg = [0] * RT_LEN
    if len(thresholds) != 5:
        return [], dbg
    x, y, w, h = roi
    y2 = min(y + h, RUNTIME_COLOR_Y_MAX)
    if y2 <= y:
        return [], dbg
    roi = (x, y, w, y2 - y)
    runtime_thresholds = []
    for slot in range(1, 6):
        runtime_thresholds.append(thresholds[slot])
    try:
        blobs = img.find_blobs(runtime_thresholds, roi=roi,
                               pixels_threshold=70, area_threshold=80,
                               merge=False)
    except Exception:
        blobs = None
    brown_blobs = runtime_merged_bear_blobs(img, roi, 4)
    white_blobs = runtime_merged_bear_blobs(img, roi, 5)
    if not blobs and not brown_blobs and not white_blobs:
        return [], dbg
    pairs = []
    if blobs:
        for blob in blobs:
            dbg[RT_RAW] += 1
            color_id = runtime_color_id_from_code(blob.code())
            if color_id <= 0:
                dbg[RT_AMBIG] += 1
                continue
            if ((color_id == 4 and brown_blobs)
                    or (color_id == 5 and white_blobs)):
                continue
            pairs.append((color_id, blob))
    if brown_blobs:
        for blob in brown_blobs:
            dbg[RT_RAW] += 1
            pairs.append((4, blob))
    if white_blobs:
        for blob in white_blobs:
            dbg[RT_RAW] += 1
            pairs.append((5, blob))
    ball_shadow_refs = []
    for color_id, blob in pairs:
        if color_id == 3 and runtime_valid_color_blob(blob, 3):
            ball_shadow_refs.append(blob)
    candidates = []
    for color_id, blob in pairs:
        pixels_threshold, area_threshold = RUNTIME_BLOB_LIMITS[color_id - 1]
        if (blob.pixels() < pixels_threshold
                or blob.w() * blob.h() < area_threshold):
            dbg[RT_SMALL] += 1
            continue
        if (cut_valid and blob.y() + blob.h()
                < cut_line_y_at_x(blob.cx()) + CUT_BLOB_DELTA):
            dbg[RT_CUT] += 1
            continue
        if not runtime_valid_color_blob(blob, color_id):
            dbg[RT_SHAPE] += 1
            continue
        if color_id == 4 and brown_blob_is_ball_shadow(blob, ball_shadow_refs):
            dbg[RT_SHAPE] += 1
            continue
        dbg[RT_OK] += 1
        candidates.append((color_id, blob))
    return candidates, dbg

def runtime_preview_summary(dbg, roi_y):
    if len(thresholds) != 5:
        return "配置不完整: %d/5" % len(thresholds)
    return ("raw=%d ok=%d ambiguous=%d small=%d shape=%d cut=%d; cut_y=%d roi_y=%d" % (
        dbg[RT_RAW], dbg[RT_OK], dbg[RT_AMBIG], dbg[RT_SMALL],
        dbg[RT_SHAPE], dbg[RT_CUT], cut_left_y, roi_y))

clock = time.clock()
frame = 0
model_dets = []
model_lock_box = None
model_refined_box = None
preview_track_slot = 0
preview_track_box = None
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

    # ---- 色块: 与正式 main.py/minimain.py 相同的一次五阈值检测和过滤 ----
    det_roi = dynamic_detect_roi()
    candidates, runtime_dbg = runtime_preview_candidates(img, det_roi)
    best = None
    best_distance = None
    best_track_score = None
    if preview_track_slot == 5 and preview_track_box is not None:
        for item in candidates:
            if item[0] != 5:
                continue
            blob = item[1]
            box = blob.rect()
            pcx = preview_track_box[0] + preview_track_box[2] // 2
            pcy = preview_track_box[1] + preview_track_box[3] // 2
            dx = blob.cx() - pcx
            dy = blob.cy() - pcy
            dist2 = dx * dx + dy * dy
            iou = box_iou(box, preview_track_box)
            if dist2 > WHT_BEAR_SMOOTH_MAX_JUMP2 and iou < 0.05:
                continue
            score = int(iou * 100000) - dist2 + blob.pixels() // 8
            if best is None or score > best_track_score:
                best = item
                best_track_score = score
    if best is None:
        for item in candidates:
            blob = item[1]
            distance = 240 - (blob.y() + blob.h())
            if (best is None or distance < best_distance or
                    (distance == best_distance and
                     (blob.x(), -blob.pixels()) <
                     (best[1].x(), -best[1].pixels()))):
                best = item
                best_distance = distance
    if best:
        slot, b = best
        raw_preview_box = b.rect()
        previous_preview_box = (preview_track_box
                                if preview_track_slot == slot else None)
        preview_track_box = (stabilize_white_bear_box(
            previous_preview_box, raw_preview_box)
            if slot == 5 else raw_preview_box)
        preview_track_slot = slot
        if PREVIEW_DRAW_DEBUG or PREVIEW_SHOW_COLOR_BOX:
            c = DRAW_COLORS.get(slot, (255, 255, 255))
            img.draw_rectangle(preview_track_box, color=c)
            if PREVIEW_DRAW_DEBUG:
                img.draw_string(preview_track_box[0], max(0, preview_track_box[1] - 10),
                                SLOT_NAMES[slot], color=c)
                img.draw_string(preview_track_box[0],
                                min(228, preview_track_box[1] + preview_track_box[3] + 2),
                                format_world_info(slot, b), color=c)
    else:
        preview_track_slot = 0
        preview_track_box = None

    # ---- 模型框(每 MODEL_EVERY 帧刷新, 预览只画黄框不画修正框) ----
    if PREVIEW_MODEL:
        if frame % MODEL_EVERY == 1:
            # 生产同款分段门槛: 远处小目标允许低分
            dets = []
            for det in detect_boxes(img):
                if det[1] >= model_score_floor(det[2], det[3], det[4], det[5]):
                    dets.append(det)
            # 场外过滤(生产由 cut+色块确认保证): 中心在切割线上方的模型框剔除
            if cut_valid:
                field_dets = []
                for det in dets:
                    if ((det[3] + det[5] // 2)
                            >= cut_line_y_at_x(det[2] + det[4] // 2) + CUT_BLOB_DELTA):
                        field_dets.append(det)
                dets = field_dets
            # 轻量锁定(生产 locked_box 思路): 有上次目标时优先选位置延续的候选,
            # 防止单帧高分误检抢走黄框
            if dets:
                if model_lock_box is not None:
                    lcx = model_lock_box[0] + model_lock_box[2] // 2
                    lcy = model_lock_box[1] + model_lock_box[3] // 2
                    near = []
                    for det in dets:
                        if ((det[2] + det[4] // 2 - lcx) ** 2
                                + (det[3] + det[5] // 2 - lcy) ** 2 <= 80 * 80):
                            near.append(det)
                    pick = max(near, key=lambda d: d[1]) if near else max(dets, key=lambda d: d[1])
                else:
                    pick = max(dets, key=lambda d: d[1])
                model_lock_box = (pick[2], pick[3], pick[4], pick[5])
                model_dets = [pick]
                model_refined_box = refine_model_box_by_lab(
                    img, pick[0], pick[2], pick[3], pick[4], pick[5], gnd_boxes)
            else:
                model_lock_box = None
                model_dets = []
                model_refined_box = None
        if model_dets:
            label, score, x, y, w, h = model_dets[0]
            img.draw_rectangle(x, y, w, h, color=MODEL_BOX_COLOR)
            if model_refined_box is not None:
                img.draw_rectangle(model_refined_box, color=(0, 255, 255))
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
            mbox = (model_refined_box if model_refined_box is not None
                    else correct_box(md[0], md[2], md[3], md[4], md[5]))
            bbox = preview_track_box if preview_track_box is not None else best[1].rect()
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
                runtime_preview_summary(runtime_dbg, det_roi[1])))
        else:
            print("fps=%.1f [cmp] 双方均无目标" % clock.fps())
