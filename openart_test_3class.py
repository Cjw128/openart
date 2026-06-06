import sensor, image, time, tf
from machine import UART


# Camera setup. Keep the same white balance/exposure values you have been using.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_framerate(60)

WB_GAINS = (101.00, 64.00, 97.00)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=1300)
sensor.set_auto_gain(False, gain_db=0)
sensor.skip_frames(time=800)

uart = UART(12, baudrate=115200)
clock = time.clock()
net = tf.load('/sd/dataset_10900_flur.tflite')


# Model labels: 0=bear, 1=ball, 2=bag.
LABELS = ['bear', 'ball', 'bag']
COLORS = [
    (255, 180, 0),
    (0, 255, 0),
    (0, 170, 255),
]

# Main-control protocol color IDs from the reference main.py:
# 1=blue bag, 2=red bag, 3=ball, 4=brown bear, 5=white bear.
MODEL_LABEL_TO_DEFAULT_COLOR_ID = [4, 3, 1]

SCORE_THRESHOLD = 0.40
# If nothing passes the normal threshold, still use the best lower-score box.
# This avoids flickering to "no target" when the correct box is present but weak.
FALLBACK_SCORE_THRESHOLD = 0.30
LOCK_FALLBACK_SCORE_THRESHOLD = 0.20
LOCK_CONFIRM_FRAMES = 5
LOCK_LOST_FRAMES = 5
LOCK_HOLD_FRAMES = 5
LOCK_MATCH_IOU = 0.05
LOCK_MAX_CENTER_DIST2 = 80 * 80
LOCK_RELAXED_CENTER_DIST2 = 130 * 130

COLOR_SWITCH_Y = 120
COLOR_LOST_FRAMES = 5
COLOR_TRACK_MARGIN = 45
COLOR_MIN_PIXELS = 100
COLOR_MIN_AREA = 100

# LAB thresholds copied from the referenced color-blob program.
COLOR_THRESHOLDS = [
    (1, (23, 96, -49, 4, -53, -30)),     # blue bag
    (2, (10, 80, 22, 122, -17, 93)),     # red bag
    (3, (50, 100, -128, -20, 18, 127)),  # tennis ball
    (4, (20, 63, 30, -1, 50, 0)),        # brown bear
    (5, (53, 100, -10, 14, -11, 12)),    # white bear
]

YELLOW_THRESHOLD = [(66, 95, 5, -27, 40, 95)]
YELLOW_ROI_LEFT = (10, 0, 20, 240)
YELLOW_ROI_RIGHT = (290, 0, 20, 240)
YELLOW_DETECT_INTERVAL = 5
YELLOW_MIN_PIXELS = 30

POS_NO_BOUNDARY = 0x00
POS_RIGHT_SIDE = 0x01
POS_CROSSED = 0x02
YELLOW_LOST_THRESHOLD = 2


# Inverse-perspective calibration copied from the referenced main.py.
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


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


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


def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00):
    data = bytearray(12)
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
    data[11] = sum(data[2:11]) & 0xFF
    uart.write(data)


def send_world_no_target(yellow_flag=False, pos_flag=0x00):
    data = bytearray(12)
    data[0] = 0xAA
    data[1] = 0x55
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
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


def receive_command_from_host():
    global openart_mode, target_color_id
    global locked_label, locked_box, lock_count, lock_active, lost_count
    global color_track_active, color_track_label, color_track_box, color_lost_count

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

    if command == 0x01:
        openart_mode = 1
    elif command == 0x02:
        openart_mode = 0
        target_color_id = 0
        locked_label = -1
        locked_box = None
        lock_count = 0
        lock_active = False
        lost_count = 0
        color_track_active = False
        color_track_label = -1
        color_track_box = None
        color_lost_count = 0
    elif command == 0x03:
        if 1 <= param <= 5:
            target_color_id = param
            color_track_active = False
            color_track_label = color_id_to_model_label(param)
            color_track_box = None
            color_lost_count = 0


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


def same_target(a, b):
    return box_iou(a, b) >= LOCK_MATCH_IOU or center_dist2(a, b) <= LOCK_MAX_CENTER_DIST2


def relaxed_same_target(a, b):
    return box_iou(a, b) >= LOCK_MATCH_IOU or center_dist2(a, b) <= LOCK_RELAXED_CENTER_DIST2


def make_roi_from_box(box):
    if not box:
        return (0, 0, 320, 240)
    x, y, w, h = box
    x0 = clamp(x - COLOR_TRACK_MARGIN, 0, 319)
    y0 = clamp(y - COLOR_TRACK_MARGIN, 0, 239)
    x1 = clamp(x + w + COLOR_TRACK_MARGIN, x0 + 1, 320)
    y1 = clamp(y + h + COLOR_TRACK_MARGIN, y0 + 1, 240)
    return (x0, y0, x1 - x0, y1 - y0)


def threshold_items_for_label(label):
    items = []
    for color_id, threshold in COLOR_THRESHOLDS:
        if target_color_id > 0 and color_id != target_color_id:
            continue
        if color_id_to_model_label(color_id) == label:
            items.append((color_id, threshold))
    return items


def valid_color_blob(blob, label):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False

    aspect = w / h
    if label == 1:
        if aspect < 0.60 or aspect > 1.40:
            return False
        if blob.density() < 0.50:
            return False
    elif label == 0:
        if aspect < 0.30 or aspect > 2.50:
            return False
        if blob.pixels() < 120:
            return False
    else:
        if aspect < 0.60 or aspect > 1.80:
            return False
        if blob.density() < 0.60:
            return False
    return True


def find_color_target(img, label, last_box):
    items = threshold_items_for_label(label)
    if not items:
        return None

    roi = make_roi_from_box(last_box)
    candidates = []
    for color_id, threshold in items:
        try:
            blobs = img.find_blobs([threshold], roi=roi,
                                   pixels_threshold=COLOR_MIN_PIXELS,
                                   area_threshold=COLOR_MIN_AREA,
                                   merge=True)
        except TypeError:
            blobs = None
        if not blobs:
            continue
        for blob in blobs:
            if valid_color_blob(blob, label):
                candidates.append((color_id, blob))

    if not candidates:
        return None

    if last_box:
        def score_item(item):
            b = item[1]
            b_box = (b.x(), b.y(), b.w(), b.h())
            return b.pixels() - center_dist2(b_box, last_box) // 20
        return max(candidates, key=score_item)

    return max(candidates, key=lambda item: item[1].pixels())


def color_id_for_model_label(label):
    if target_color_id > 0 and color_id_to_model_label(target_color_id) == label:
        return target_color_id
    if 0 <= label < len(MODEL_LABEL_TO_DEFAULT_COLOR_ID):
        return MODEL_LABEL_TO_DEFAULT_COLOR_ID[label]
    return 0


def update_yellow_detection(img):
    global yellow_detected, yellow_lost_count, yellow_boundary_y

    try:
        left = img.find_blobs(YELLOW_THRESHOLD, roi=YELLOW_ROI_LEFT,
                              pixels_threshold=YELLOW_MIN_PIXELS,
                              area_threshold=20, merge=True)
        right = img.find_blobs(YELLOW_THRESHOLD, roi=YELLOW_ROI_RIGHT,
                               pixels_threshold=YELLOW_MIN_PIXELS,
                               area_threshold=20, merge=True)
    except TypeError:
        left = None
        right = None

    if left and right:
        lb = max(left, key=lambda b: b.pixels())
        rb = max(right, key=lambda b: b.pixels())
        yellow_detected = True
        yellow_lost_count = 0
        yellow_boundary_y = (lb.cy() + rb.cy()) // 2
    else:
        yellow_detected = False
        if openart_mode == 0:
            yellow_boundary_y = 0


def current_pos_flag():
    global yellow_lost_count, openart_mode

    if openart_mode == 1:
        if not yellow_detected:
            yellow_lost_count += 1
            if yellow_lost_count >= YELLOW_LOST_THRESHOLD:
                openart_mode = 2
                return POS_CROSSED
            return POS_NO_BOUNDARY
        yellow_lost_count = 0
        return POS_NO_BOUNDARY

    if openart_mode == 2:
        return POS_CROSSED

    if yellow_detected:
        return POS_RIGHT_SIDE
    return POS_NO_BOUNDARY


H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)
openart_mode = 0
target_color_id = 0

locked_label = -1
locked_box = None
lock_count = 0
lock_active = False
lost_count = 0

color_track_active = False
color_track_label = -1
color_track_box = None
color_lost_count = 0

yellow_detected = False
yellow_lost_count = 0
yellow_boundary_y = 0
frame_count = 0
last_print_time = time.ticks_ms()


while True:
    clock.tick()
    frame_count += 1
    receive_command_from_host()

    img = sensor.snapshot()
    if frame_count % YELLOW_DETECT_INTERVAL == 0:
        update_yellow_detection(img)

    best = None
    source = 'model'
    send_color_id = 0

    if color_track_active:
        found = find_color_target(img, color_track_label, color_track_box)
        if found:
            send_color_id, blob = found
            label = color_track_label
            scores = 1.0
            x1 = blob.x()
            y1 = blob.y()
            w = blob.w()
            h = blob.h()
            best = (label, scores, x1, y1, w, h)
            color_track_box = (x1, y1, w, h)
            color_lost_count = 0
            source = 'color'
        else:
            color_lost_count += 1
            if color_lost_count > COLOR_LOST_FRAMES:
                color_track_active = False
                color_track_label = -1
                color_track_box = None
                color_lost_count = 0

    if not best:
        img1 = img.copy(0.75, 1)
        candidates = []
        fallback_candidates = []
        target_label = color_id_to_model_label(target_color_id) if target_color_id > 0 else -1
        low_score_threshold = LOCK_FALLBACK_SCORE_THRESHOLD if lock_active else FALLBACK_SCORE_THRESHOLD

        for obj in tf.detect(net, img1):
            x1, y1, x2, y2, label, scores = obj
            label = int(label)
            scores = float(scores)
            if target_label >= 0 and label != target_label:
                continue

            w = x2 - x1
            h = y2 - y1
            x1 = int(x1 * img.width())
            y1 = int(y1 * img.height())
            w = int(w * img.width())
            h = int(h * img.height())
            candidate = (label, scores, x1, y1, w, h)
            if scores > SCORE_THRESHOLD:
                candidates.append(candidate)
            elif scores >= low_score_threshold:
                fallback_candidates.append(candidate)

        candidates.sort(key=lambda item: item[1], reverse=True)
        fallback_candidates.sort(key=lambda item: item[1], reverse=True)

        if lock_active and locked_box:
            best_match = None
            best_match_score = -1.0
            for cand in candidates + fallback_candidates:
                label, scores, x1, y1, w, h = cand
                if label != locked_label:
                    continue
                cand_box = (x1, y1, w, h)
                if not relaxed_same_target(cand_box, locked_box):
                    continue
                match_score = box_iou(cand_box, locked_box) * 10.0 - center_dist2(cand_box, locked_box) / 1000.0
                if match_score > best_match_score:
                    best_match_score = match_score
                    best_match = cand

            if best_match:
                best = best_match
                if best_match[1] < SCORE_THRESHOLD:
                    source = 'model_low'
                lost_count = 0
            else:
                lost_count += 1
                if lost_count > LOCK_LOST_FRAMES:
                    locked_label = -1
                    locked_box = None
                    lock_count = 0
                    lock_active = False
                    lost_count = 0

        if not best and lock_active and locked_box and lost_count <= LOCK_HOLD_FRAMES:
            x1, y1, w, h = locked_box
            best = (locked_label, 0.0, x1, y1, w, h)
            source = 'hold'

        if not best and candidates and not lock_active:
            best = candidates[0]
        if not best and fallback_candidates and not lock_active:
            best = fallback_candidates[0]
            source = 'model_low'

    pos_flag = current_pos_flag()

    if best:
        label, scores, x1, y1, w, h = best
        cur_box = (x1, y1, w, h)
        if send_color_id == 0:
            send_color_id = color_id_for_model_label(label)

        if locked_box and label == locked_label and same_target(cur_box, locked_box):
            lock_count += 1
        else:
            locked_label = label
            lock_count = 1
        locked_box = cur_box
        if lock_count >= LOCK_CONFIRM_FRAMES:
            lock_active = True

        cx = x1 + w // 2
        cy = y1 + h // 2
        x2 = x1 + w
        y2 = y1 + h

        if source != 'hold' and (not color_track_active) and cy >= COLOR_SWITCH_Y:
            color_track_active = True
            color_track_label = label
            color_track_box = cur_box
            color_lost_count = 0

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag)

        color = COLORS[label] if label < len(COLORS) else (255, 255, 255)
        text = '%s %.2f' % (LABELS[label], scores) if label < len(LABELS) else '%d %.2f' % (label, scores)
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_string(x1, max(0, y1 - 15), text, color=color, scale=1)
        if yellow_boundary_y > 0:
            img.draw_line(0, yellow_boundary_y, 320, yellow_boundary_y, color=(255, 255, 0), thickness=2)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 300:
            print('fps=%.2f,src=%s,cid=%d,label=%d,score=%.2f,wx=%d,wy=%d,x1=%d,y1=%d,x2=%d,y2=%d,cx=%d,cy=%d' %
                  (clock.fps(), source, send_color_id, label, scores, wx_mm, wy_mm, x1, y1, x2, y2, cx, cy))
            last_print_time = now
    else:
        send_world_no_target(yellow_detected, POS_NO_BOUNDARY)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 500:
            print('fps=%.2f,src=none,cid=0,wx=0,wy=0' % clock.fps())
            last_print_time = now
