import sensor, image, time, tf
from machine import UART


# Camera setup kept from the original program.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(True)
WB_GAINS = (101.00, 64.00, 97.00)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)
sensor.skip_frames(time=800)

uart = UART(12, baudrate=115200)
clock = time.clock()
net = tf.load('/sd/80lite0.5shi.tflite')


# OpenART model order: 0=bear, 1=ball, 2=bag.
LABELS = ['bear', 'ball', 'bag']
COLORS = [
    (255, 180, 0),
    (0, 255, 0),
    (0, 170, 255),
]

# The 3-class model cannot distinguish bag/bear colors. These IDs are only
# protocol defaults; a host command may select another compatible color ID.
# 1=blue bag, 2=red bag, 3=ball, 4=brown bear, 5=white bear.
MODEL_LABEL_TO_DEFAULT_COLOR_ID = [4, 3, 1]

NEAR_SCORE_THRESHOLD = 0.80
MID_SCORE_THRESHOLD = 0.60
FAR_SCORE_THRESHOLD = 0.40
NEAR_DISTANCE_CM = 12.0
MID_DISTANCE_CM = 18.0
FAR_DISTANCE_CM = 25.0

NEAR_FALLBACK_SCORE_THRESHOLD = 0.52
MID_FALLBACK_SCORE_THRESHOLD = 0.42
FAR_FALLBACK_SCORE_THRESHOLD = 0.30
LOCK_FALLBACK_SCORE_THRESHOLD = 0.20

LOCK_CONFIRM_FRAMES = 5
LOCK_LOST_FRAMES = 5
LOCK_HOLD_FRAMES = 5
LOCK_MATCH_IOU = 0.05
LOCK_MAX_CENTER_DIST2 = 80 * 80
LOCK_RELAXED_CENTER_DIST2 = 130 * 130

POS_NO_BOUNDARY = 0x00


# Inverse-perspective calibration kept from the original program.
CALIB_PIXEL = [
    [60, 220],
    [260, 220],
    [100, 140],
    [220, 140],
]
CALIB_WORLD = [
    [-7.5, 7.5],
    [7.5, 7.5],
    [-7.5, 22.5],
    [7.5, 22.5],
]


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


def box_to_world(x, y, w, h):
    if H_pix2world is None:
        return (0.0, 0.0)
    return pixel_to_world(x + w / 2.0, y + h, H_pix2world)


def threshold_by_distance(distance_cm, near_value, mid_value, far_value):
    if distance_cm <= NEAR_DISTANCE_CM:
        return near_value
    if distance_cm >= FAR_DISTANCE_CM:
        return far_value
    if distance_cm <= MID_DISTANCE_CM:
        ratio = ((distance_cm - NEAR_DISTANCE_CM) /
                 (MID_DISTANCE_CM - NEAR_DISTANCE_CM))
        return near_value + (mid_value - near_value) * ratio
    ratio = ((distance_cm - MID_DISTANCE_CM) /
             (FAR_DISTANCE_CM - MID_DISTANCE_CM))
    return mid_value + (far_value - mid_value) * ratio


def model_score_threshold_for_box(x, y, w, h):
    _, world_y = box_to_world(x, y, w, h)
    return threshold_by_distance(
        world_y,
        NEAR_SCORE_THRESHOLD,
        MID_SCORE_THRESHOLD,
        FAR_SCORE_THRESHOLD,
    )


def fallback_score_threshold_for_box(x, y, w, h):
    _, world_y = box_to_world(x, y, w, h)
    return threshold_by_distance(
        world_y,
        NEAR_FALLBACK_SCORE_THRESHOLD,
        MID_FALLBACK_SCORE_THRESHOLD,
        FAR_FALLBACK_SCORE_THRESHOLD,
    )


def send_world_data(color_id, wx_mm, wy_mm, pixel_width):
    data = bytearray(12)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = color_id & 0xFF
    data[3] = wx_mm & 0xFF
    data[4] = (wx_mm >> 8) & 0xFF
    data[5] = wy_mm & 0xFF
    data[6] = (wy_mm >> 8) & 0xFF
    data[7] = pixel_width & 0xFF
    data[8] = (pixel_width >> 8) & 0xFF
    data[9] = 0x00
    data[10] = POS_NO_BOUNDARY
    data[11] = sum(data[2:11]) & 0xFF
    uart.write(data)


def send_world_no_target():
    data = bytearray(12)
    data[0] = 0xAA
    data[1] = 0x55
    data[9] = 0x00
    data[10] = POS_NO_BOUNDARY
    data[11] = sum(data[2:11]) & 0xFF
    uart.write(data)


def color_id_to_model_label(color_id):
    if color_id == 1 or color_id == 2:
        return 2
    if color_id == 3:
        return 1
    if color_id == 4 or color_id == 5:
        return 0
    return -1


def color_id_for_model_label(label):
    if target_color_id > 0 and color_id_to_model_label(target_color_id) == label:
        return target_color_id
    if 0 <= label < len(MODEL_LABEL_TO_DEFAULT_COLOR_ID):
        return MODEL_LABEL_TO_DEFAULT_COLOR_ID[label]
    return 0


def reset_lock():
    global locked_label, locked_box, lock_count, lock_active, lost_count
    locked_label = -1
    locked_box = None
    lock_count = 0
    lock_active = False
    lost_count = 0


def receive_command_from_host():
    global target_color_id

    if uart.any() < 4:
        return
    data = uart.read(4)
    if data is None or len(data) < 4:
        return
    if data[0] != 0xAA or data[1] != 0x55:
        return

    command = data[2]
    checksum_recv = data[3]
    param = 0
    if command == 0x03:
        if uart.any() < 1:
            return
        param_data = uart.read(1)
        if param_data is None or len(param_data) < 1:
            return
        param = param_data[0]
        checksum_calc = (command + param) & 0xFF
    else:
        checksum_calc = command & 0xFF

    if checksum_calc != checksum_recv:
        return

    if command == 0x02:
        target_color_id = 0
        reset_lock()
    elif command == 0x03 and 1 <= param <= 5:
        target_color_id = param
        reset_lock()


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
    dx = (ax1 + aw // 2) - (bx1 + bw // 2)
    dy = (ay1 + ah // 2) - (by1 + bh // 2)
    return dx * dx + dy * dy


def same_target(a, b):
    return (box_iou(a, b) >= LOCK_MATCH_IOU or
            center_dist2(a, b) <= LOCK_MAX_CENTER_DIST2)


def relaxed_same_target(a, b):
    return (box_iou(a, b) >= LOCK_MATCH_IOU or
            center_dist2(a, b) <= LOCK_RELAXED_CENTER_DIST2)


H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)
target_color_id = 0

locked_label = -1
locked_box = None
lock_count = 0
lock_active = False
lost_count = 0

frame_count = 0
last_print_time = time.ticks_ms()


while True:
    clock.tick()
    frame_count += 1
    receive_command_from_host()

    img = sensor.snapshot()
    model_image = img.copy(0.75, 1)
    candidates = []
    fallback_candidates = []
    target_label = color_id_to_model_label(target_color_id)

    for obj in tf.detect(net, model_image):
        x1, y1, x2, y2, label, score = obj
        label = int(label)
        score = float(score)
        if target_label >= 0 and label != target_label:
            continue

        w = int((x2 - x1) * img.width())
        h = int((y2 - y1) * img.height())
        x = int(x1 * img.width())
        y = int(y1 * img.height())
        if w <= 0 or h <= 0:
            continue

        candidate = (label, score, x, y, w, h)
        normal_threshold = model_score_threshold_for_box(x, y, w, h)
        if lock_active:
            fallback_threshold = LOCK_FALLBACK_SCORE_THRESHOLD
        else:
            fallback_threshold = fallback_score_threshold_for_box(x, y, w, h)

        if score > normal_threshold:
            candidates.append(candidate)
        elif score >= fallback_threshold:
            fallback_candidates.append(candidate)

    candidates.sort(key=lambda item: item[1], reverse=True)
    fallback_candidates.sort(key=lambda item: item[1], reverse=True)

    best = None
    source = 'model'

    if lock_active and locked_box:
        best_match = None
        best_match_score = -1.0
        for candidate in candidates + fallback_candidates:
            label, score, x, y, w, h = candidate
            if label != locked_label:
                continue
            candidate_box = (x, y, w, h)
            if not relaxed_same_target(candidate_box, locked_box):
                continue
            match_score = (box_iou(candidate_box, locked_box) * 10.0 -
                           center_dist2(candidate_box, locked_box) / 1000.0)
            if match_score > best_match_score:
                best_match_score = match_score
                best_match = candidate

        if best_match:
            best = best_match
            if best_match in fallback_candidates:
                source = 'model_low'
            lost_count = 0
        else:
            lost_count += 1
            if lost_count > LOCK_LOST_FRAMES:
                reset_lock()

    if best is None and lock_active and locked_box and lost_count <= LOCK_HOLD_FRAMES:
        x, y, w, h = locked_box
        best = (locked_label, 0.0, x, y, w, h)
        source = 'hold'

    if best is None and candidates and not lock_active:
        best = candidates[0]
    if best is None and fallback_candidates and not lock_active:
        best = fallback_candidates[0]
        source = 'model_low'

    if best is not None:
        label, score, x, y, w, h = best
        current_box = (x, y, w, h)

        if locked_box and label == locked_label and same_target(current_box, locked_box):
            lock_count += 1
        else:
            locked_label = label
            lock_count = 1
        locked_box = current_box
        if lock_count >= LOCK_CONFIRM_FRAMES:
            lock_active = True

        world_x, world_y = box_to_world(x, y, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_color_id = color_id_for_model_label(label)
        send_world_data(send_color_id, wx_mm, wy_mm, w)

        color = COLORS[label] if 0 <= label < len(COLORS) else (255, 255, 255)
        if 0 <= label < len(LABELS):
            text = '%s %.2f' % (LABELS[label], score)
        else:
            text = '%d %.2f' % (label, score)
        img.draw_rectangle((x, y, w, h), color=color, thickness=2)
        img.draw_string(x, max(0, y - 15), text, color=color, scale=1)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 300:
            threshold = model_score_threshold_for_box(x, y, w, h)
            print('fps=%.2f,src=%s,cid=%d,label=%d,score=%.2f,thr=%.2f,wcm=(%.1f,%.1f),wmm=(%d,%d),box=(%d,%d,%d,%d)' %
                  (clock.fps(), source, send_color_id, label, score, threshold,
                   world_x, world_y, wx_mm, wy_mm, x, y, w, h))
            last_print_time = now
    else:
        send_world_no_target()
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 500:
            print('fps=%.2f,src=none,cid=0,wx=0,wy=0' % clock.fps())
            last_print_time = now
