import sensor, gc, time, math
try:
    import tf
except Exception:
    tf = None
from machine import UART
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(True)
def snapshot_frame():
    return sensor.snapshot().replace(hmirror=True)
WB_GAINS = (101.00, 64.00, 97.00)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.skip_frames(time=500)
sensor.set_auto_exposure(False, exposure_us=1200)
sensor.set_auto_gain(False, gain_db=0)
uart = UART(12, baudrate=115200)
all_color_thresholds = [
    (34, 100, -41, 5, -72, -17),
    (10, 80, 22, 122, -17, 93),
    (50, 100, -128, -27, 20, 127),
    (21, 52, -77, 25, 1, 99),
    (51, 100, -5, 5, -38, 18)
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
        ground_threshold = _average_ground_threshold(ground_rows)
        if len(rows) == 5:
            loaded_rows = []
            for slot in range(1, 6):
                loaded_rows.append(rows[slot])
            return (loaded_rows, exposure, ground_threshold)
        return None, exposure, ground_threshold
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
lost_frame_count = 0
MAX_LOST_FRAMES = 30
DETECT_Y_MIN = 8
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)
COLOR_DETECT_Y_MAX = 230
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = ([_loaded_ground_threshold]
                         if _loaded_ground_threshold
                         else [(25, 62, -3, 57, -96, 127)])
CUT_BLOB_MIN_H = 12
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
CUT_UPDATE_INTERVAL = 4
CUT_MIN_PIXELS = 8
CUT_MIN_AREA = 8
CUT_ROI_Y_OFFSET = -10
CUT_EMA_ALPHA = 0.35
CUT_MAX_MISS = 10
CUT_BLOB_DELTA = 2
TRACK_MAX_JUMP_PX = 90
TRACK_MAX_JUMP2 = TRACK_MAX_JUMP_PX * TRACK_MAX_JUMP_PX
TRACK_MIN_IOU = 0.05
BRN_BEAR_MERGE_MARGIN = 12
WHT_BEAR_MERGE_MARGIN = 10
MODEL_PATH = '/sd/dataset_25000_exposure.tflite'
MODEL_COLOR_IDS = ((4, 5), (3,), (1, 2))
MODEL_LAB_IDS = ((3, 4, 5), (3, 4, 5), (1, 2))
MODEL_CONTACT_OFF_X = (-1, -1, -1)
MODEL_CONTACT_OFF_Y = (0, 0, 0)
MODEL_NEAR_SCALE_W = (1.40, 1.35, 1.50)
MODEL_NEAR_SCALE_H = (1.65, 1.50, 1.55)
MODEL_LOCK_CONFIRM_FRAMES = 2
MODEL_LOST_FRAMES = 4
MODEL_MATCH_CENTER2 = 90 * 90
MODEL_PENDING_CENTER2 = 90 * 90
COLOR_CONFIRM_FRAMES = 2
COLOR_ROI_INSET_X_PERCENT = 5
COLOR_ROI_INSET_TOP_PERCENT = 5
COLOR_ROI_INSET_BOTTOM_PERCENT = 10
COLOR_EVIDENCE_MIN_PIXELS = 8
COLOR_EVIDENCE_MIN_COVER_X1000 = 15
COLOR_WINNER_RATIO_X100 = 125
COLOR_WINNER_MARGIN_X1000 = 10
CONTACT_JITTER_PX = 1.0
CONTACT_JITTER2 = CONTACT_JITTER_PX * CONTACT_JITTER_PX
CONTACT_REJECT_JUMP2 = TRACK_MAX_JUMP2
CONTACT_PREDICT_LIMIT = 90.0
CONTACT_LEAD_MAX_MS = 80
CONTACT_LEAD_EXTRA_MS = 12
CONTACT_VELOCITY_ALPHA = 1.0
LATENCY_TEST = False
GC_CHECK_INTERVAL = 10
GC_FORCE_INTERVAL = 30
GC_MIN_FREE = 48 * 1024
model_net = None
model_fb = None
model_runtime_enabled = True
model_copy_to_fb_supported = True
model_infer_error_count = 0
model_lock = [-1, None, -1, None, 0, 0]
model_color = [0, 0, 0, False]
model_track = [False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, -1, 0, 0]
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
FRONT_SCAN_MIN_PIXELS = 60
FRONT_SCAN_STABLE_FRAMES = 6
FRONT_SCAN_MAX_FRAMES = 12
front_scan_last_current_id = 0
front_scan_last_mask = -1
front_scan_last_count = 0
front_scan_stable_count = 0
front_scan_total_count = 0
RETURN_YELLOW_PACKET_ID = 0xC8
RETURN_YELLOW_THRESHOLD = [(51, 91, -32, 36, 1, 118)]
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
MODE_SEARCH = 0
MODE_RETURN = 3
openart_mode = MODE_SEARCH
def reset_target_tracking_state():
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
    reset_hybrid_tracking()
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
    if code <= 0 or code & (code - 1):
        return 0
    color_id = 1
    while code > 1:
        code >>= 1
        color_id += 1
    if color_id > len(all_color_thresholds):
        return 0
    return color_id
def find_color_blobs_once(img, roi, fixed_color_id=0, pixels_threshold_override=0):
    if fixed_color_id > 0:
        thresholds = _color_threshold_groups[fixed_color_id - 1]
        pixels_threshold, area_threshold = _color_blob_limits[fixed_color_id - 1]
        merge = True
    else:
        thresholds = all_color_thresholds
        pixels_threshold = MULTICOLOR_MIN_PIXELS
        area_threshold = MULTICOLOR_MIN_AREA
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
def color_id_to_model_label(color_id):
    if color_id == 1 or color_id == 2:
        return 2
    if color_id == 3:
        return 1
    if color_id == 4 or color_id == 5:
        return 0
    return -1
def model_labels_compatible(first, second):
    return (first == second or
            (0 <= first <= 1 and 0 <= second <= 1))
def reset_model_track():
    model_track[:] = [False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, -1, 0, 0]
def reset_hybrid_tracking():
    model_lock[:] = [-1, None, -1, None, 0, 0]
    model_color[:] = [0, 0, 0, False]
    reset_model_track()
def restore_host_hybrid_lock():
    color_id = target_color_id if host_color_id_received else 0
    reset_hybrid_tracking()
    if color_id > 0:
        model_color[0] = color_id
        model_color[3] = True
def apply_host_hybrid_color(color_id):
    label = color_id_to_model_label(color_id)
    if (model_lock[1] is None or
            not model_labels_compatible(model_lock[0], label)):
        reset_hybrid_tracking()
    else:
        model_lock[2:6] = [-1, None, 0, 0]
    model_color[:] = [color_id, 0, 0, True]
def disable_model_runtime(reason):
    global model_runtime_enabled, model_net, model_fb
    if model_runtime_enabled:
        print('[MODEL ALARM] ' + reason + '; target output disabled')
    model_runtime_enabled = False
    had_buffer = model_fb is not None
    model_net = None
    model_fb = None
    if had_buffer:
        try:
            sensor.dealloc_extra_fb()
        except Exception:
            pass
    gc.collect()
    reset_hybrid_tracking()
def init_model_runtime():
    global model_net, model_fb
    if not model_runtime_enabled:
        return
    if tf is None:
        disable_model_runtime('tf module unavailable')
        return
    try:
        model_net = tf.load(MODEL_PATH)
        model_fb = sensor.alloc_extra_fb(240, 240, sensor.RGB565)
        print('[MODEL] loaded ' + MODEL_PATH)
    except Exception as error:
        disable_model_runtime('load failed: ' + str(error))
def raw_model_box(x, y, w, h):
    x = clamp_int(x, 0, 319)
    y = clamp_int(y, 0, 239)
    w = clamp_int(w, 1, 320 - x)
    h = clamp_int(h, 1, 240 - y)
    return (x, y, w, h)
def model_color_roi(box):
    x, y, w, h = box
    inset_x = max(1, w * COLOR_ROI_INSET_X_PERCENT // 100)
    inset_top = max(1, h * COLOR_ROI_INSET_TOP_PERCENT // 100)
    inset_bottom = max(1, h * COLOR_ROI_INSET_BOTTOM_PERCENT // 100)
    x += inset_x
    y += inset_top
    w -= inset_x * 2
    h -= inset_top + inset_bottom
    y2 = min(y + h, COLOR_DETECT_Y_MAX)
    return (x, y, w, y2 - y) if w > 0 and y2 > y else None
def model_proximity(box):
    cy = box[1] + box[3] // 2
    return clamp_int(max((cy - 70) * 100 // 60,
                         (box[2] - 20) * 100 // 35), 0, 100)
def corrected_output_box(label, box):
    x, y, w, h = box
    proximity = model_proximity(box) / 100.0
    scale_w = 1.0 + (MODEL_NEAR_SCALE_W[label] - 1.0) * proximity
    scale_h = 1.0 + (MODEL_NEAR_SCALE_H[label] - 1.0) * proximity
    width = int(w * scale_w + 0.5)
    height = int(h * scale_h + 0.5)
    center_x = x + w * 0.5 + MODEL_CONTACT_OFF_X[label]
    center_y = y + h * 0.5
    left = int(center_x - width * 0.5 + 0.5)
    top = int(center_y - height * 0.5 + 0.5)
    x0 = clamp_int(left, 0, 319)
    y0 = clamp_int(top, 0, 239)
    x1 = clamp_int(left + width, x0 + 1, 320)
    y1 = clamp_int(top + height, y0 + 1, 240)
    return (x0, y0, x1 - x0, y1 - y0)
def model_box_matches(a, b, center_limit2):
    return (a is not None and b is not None and
            (box_iou(a, b) >= TRACK_MIN_IOU or
             center_dist2(a, b) <= center_limit2))
def model_copy_frame(img):
    global model_copy_to_fb_supported
    if not model_copy_to_fb_supported or model_fb is None:
        raise RuntimeError('preallocated model framebuffer unavailable')
    try:
        return img.copy(0.75, 1, copy_to_fb=model_fb)
    except TypeError:
        model_copy_to_fb_supported = False
        raise RuntimeError('copy_to_fb unsupported')
def run_model_best(img):
    global model_infer_error_count
    desired_label = color_id_to_model_label(target_color_id)
    if desired_label < 0 and model_lock[1] is not None:
        desired_label = model_lock[0]
    try:
        sample_time = time.ticks_ms()
        objects = tf.detect(model_net, model_copy_frame(img))
        best = None
        best_rank = None
        locked = model_lock[1] is not None
        anchor = model_lock[1]
        for obj in objects:
            x1, y1, x2, y2, label_value, score_value = obj
            label = int(label_value)
            score = float(score_value)
            if (label < 0 or label >= len(MODEL_COLOR_IDS) or
                    (desired_label >= 0 and
                     not model_labels_compatible(label, desired_label))):
                continue
            x = int(float(x1) * img.width())
            y = int(float(y1) * img.height())
            w = int((float(x2) - float(x1)) * img.width())
            h = int((float(y2) - float(y1)) * img.height())
            if w <= 0 or h <= 0:
                continue
            box = raw_model_box(x, y, w, h)
            if (ENABLE_DYNAMIC_CUT and dynamic_cut_valid and
                    box[1] + box[3] < dynamic_cut_left_y + CUT_BLOB_DELTA):
                continue
            if locked and not model_box_matches(box, anchor, MODEL_MATCH_CENTER2):
                continue
            if locked:
                rank = (int(box_iou(box, anchor) * 100000) -
                        center_dist2(box, anchor) + int(score * 1000))
                confirm = 1
            else:
                confirm = MODEL_LOCK_CONFIRM_FRAMES
                rank = int(score * 100000)
            if best is None or rank > best_rank:
                best = (label, box, confirm, sample_time)
                best_rank = rank
        model_infer_error_count = 0
        return best
    except Exception as error:
        model_infer_error_count += 1
        if model_infer_error_count >= 3:
            disable_model_runtime('three inference failures: ' + str(error))
        return None
def find_color_evidence_blobs(img, roi, color_id, pixels_threshold):
    thresholds = _color_threshold_groups[color_id - 1]
    try:
        if color_id == 4 or color_id == 5:
            margin = BRN_BEAR_MERGE_MARGIN if color_id == 4 else WHT_BEAR_MERGE_MARGIN
            try:
                return img.find_blobs(thresholds, roi=roi,
                                      pixels_threshold=pixels_threshold,
                                      area_threshold=pixels_threshold,
                                      merge=True, margin=margin)
            except TypeError:
                pass
        return img.find_blobs(thresholds, roi=roi,
                              pixels_threshold=pixels_threshold,
                              area_threshold=pixels_threshold, merge=True)
    except Exception:
        return None
def classify_model_color(img, label, box):
    if label < 0 or label >= len(MODEL_COLOR_IDS) or box is None:
        return 0
    roi = model_color_roi(box)
    if roi is None:
        return 0
    roi_area = roi[2] * roi[3]
    minimum = max(COLOR_EVIDENCE_MIN_PIXELS,
                  roi_area * COLOR_EVIDENCE_MIN_COVER_X1000 // 1000)
    best_id = 0
    best_pixels = 0
    second_pixels = 0
    for color_id in MODEL_LAB_IDS[label]:
        blobs = find_color_evidence_blobs(img, roi, color_id, minimum)
        pixels = 0
        if blobs:
            for blob in blobs:
                pixels += blob.pixels()
        if pixels > best_pixels:
            second_pixels = best_pixels
            best_id = color_id
            best_pixels = pixels
        elif pixels > second_pixels:
            second_pixels = pixels
    if best_pixels < minimum:
        return 0
    if second_pixels:
        margin = max(COLOR_EVIDENCE_MIN_PIXELS,
                     roi_area * COLOR_WINNER_MARGIN_X1000 // 1000)
        if (best_pixels * 100 < second_pixels * COLOR_WINNER_RATIO_X100 or
                best_pixels - second_pixels < margin):
            return 0
    return best_id
def confirm_model_color(observed_id):
    if observed_id <= 0:
        model_color[1] = 0
        model_color[2] = 0
        return
    if model_color[3]:
        return
    if observed_id == model_color[0] and model_color[0] > 0:
        model_color[1] = 0
        model_color[2] = 0
        return
    if observed_id == model_color[1]:
        model_color[2] += 1
    else:
        model_color[1] = observed_id
        model_color[2] = 1
    if model_color[2] >= COLOR_CONFIRM_FRAMES:
        changed = model_color[0] > 0 and model_color[0] != observed_id
        model_color[0] = observed_id
        model_color[1] = 0
        model_color[2] = 0
        if changed:
            reset_model_track()
def box_from_center(center_x, center_y, width, height):
    width = clamp_int(int(width + 0.5), 1, 320)
    height = clamp_int(int(height + 0.5), 1, 240)
    left = int(center_x - width * 0.5 + 0.5)
    top = int(center_y - height * 0.5 + 0.5)
    x0 = clamp_int(left, 0, 319)
    y0 = clamp_int(top, 0, 239)
    x1 = clamp_int(left + width, x0 + 1, 320)
    y1 = clamp_int(top + height, y0 + 1, 240)
    return (x0, y0, x1 - x0, y1 - y0)
def track_lead_offset():
    raw_age = time.ticks_diff(time.ticks_ms(), model_track[11])
    age = clamp_int(raw_age + CONTACT_LEAD_EXTRA_MS, 0, CONTACT_LEAD_MAX_MS)
    if LATENCY_TEST and frame_count % 20 == 0:
        print('[LAT]', raw_age, age)
    dx = model_track[9] * age
    dy = model_track[10] * age
    distance2 = dx * dx + dy * dy
    if distance2 > CONTACT_PREDICT_LIMIT * CONTACT_PREDICT_LIMIT:
        scale = CONTACT_PREDICT_LIMIT / math.sqrt(distance2)
        dx *= scale
        dy *= scale
    return dx, dy
def coordinate_box_from_track(dx, dy):
    return box_from_center(model_track[5] + dx,
                           model_track[6] + dy - model_track[13] * 0.5,
                           model_track[12], model_track[13])
def output_box_from_track(dx, dy):
    return box_from_center(model_track[1] + dx, model_track[2] + dy,
                           model_track[3], model_track[4])
def observe_model_box(label, box, sample_time):
    raw_x = box[0] + box[2] * 0.5 + MODEL_CONTACT_OFF_X[label]
    raw_y = box[1] + box[3] + MODEL_CONTACT_OFF_Y[label]
    display = corrected_output_box(label, box)
    center_x = display[0] + display[2] * 0.5
    center_y = display[1] + display[3] * 0.5
    if not model_track[0]:
        model_track[:] = [True, center_x, center_y, float(display[2]),
                          float(display[3]), raw_x, raw_y, raw_x, raw_y,
                          0.0, 0.0, sample_time, box[2], box[3]]
        return True
    raw_dx = raw_x - model_track[7]
    raw_dy = raw_y - model_track[8]
    raw_distance2 = raw_dx * raw_dx + raw_dy * raw_dy
    if (raw_distance2 > CONTACT_REJECT_JUMP2 and model_lock[1] is not None and
            box_iou(box, model_lock[1]) < TRACK_MIN_IOU):
        return False
    elapsed = time.ticks_diff(sample_time, model_track[11])
    if elapsed <= 0:
        elapsed = 1
    if raw_distance2 <= CONTACT_JITTER2:
        model_track[9] = 0.0
        model_track[10] = 0.0
    else:
        vx = raw_dx / elapsed
        vy = raw_dy / elapsed
        model_track[9] += (vx - model_track[9]) * CONTACT_VELOCITY_ALPHA
        model_track[10] += (vy - model_track[10]) * CONTACT_VELOCITY_ALPHA
    model_track[7] = raw_x
    model_track[8] = raw_y
    contact_dx = raw_x - model_track[5]
    contact_dy = raw_y - model_track[6]
    contact_distance2 = contact_dx * contact_dx + contact_dy * contact_dy
    if contact_distance2 > CONTACT_JITTER2:
        keep = CONTACT_JITTER_PX / math.sqrt(contact_distance2)
        model_track[5] = raw_x - contact_dx * keep
        model_track[6] = raw_y - contact_dy * keep
    model_track[1] = center_x
    model_track[2] = center_y
    model_track[3] = display[2]
    model_track[4] = display[3]
    model_track[11] = sample_time
    model_track[12] = box[2]
    model_track[13] = box[3]
    return True
def accept_model_candidate(candidate):
    if candidate is None:
        return False
    label, box, confirm_frames, sample_time = candidate
    if model_lock[1] is not None:
        if not observe_model_box(label, box, sample_time):
            return False
        model_lock[0] = label
        model_lock[1] = box
        model_lock[5] = 0
        return True
    if (model_lock[3] is not None and label == model_lock[2] and
            model_box_matches(box, model_lock[3], MODEL_PENDING_CENTER2)):
        model_lock[4] += 1
    else:
        model_lock[2] = label
        model_lock[4] = 1
    model_lock[3] = box
    if model_lock[4] < confirm_frames:
        return False
    model_lock[0] = label
    model_lock[1] = box
    model_lock[2:6] = [-1, None, 0, 0]
    reset_model_track()
    return observe_model_box(label, box, sample_time)
def set_color_tracking(color_id, box):
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    color_track_active = True
    color_track_box = box
    color_track_color_id = color_id
    color_lost_count = 0
def maybe_collect(frame_index):
    if (frame_index % GC_CHECK_INTERVAL == 0 and
            (frame_index % GC_FORCE_INTERVAL == 0 or gc.mem_free() < GC_MIN_FREE)):
        gc.collect()
def process_model_only_target(img, frame_index, run_model):
    global color_track_active, color_track_box, color_track_color_id, color_lost_count
    if not model_runtime_enabled or openart_mode != MODE_SEARCH:
        color_track_active = False
        color_track_box = None
        color_track_color_id = 0
        color_lost_count = 0
        return None
    observed = False
    if run_model:
        candidate = run_model_best(img)
        observed = accept_model_candidate(candidate)
        if model_lock[1] is not None and not observed:
            model_lock[5] += 1
            if model_lock[5] > MODEL_LOST_FRAMES:
                restore_host_hybrid_lock()
                color_track_active = False
                color_track_box = None
                color_track_color_id = 0
                color_lost_count = 0
                return None
        elif candidate is None and model_lock[1] is None:
            model_lock[2:5] = [-1, None, 0]
    if model_lock[1] is None:
        return None
    if not run_model and not model_color[3] and model_color[0] <= 0:
        confirm_model_color(classify_model_color(
            img, model_lock[0], model_lock[1]))
    color_id = model_color[0]
    if (color_id <= 0 or not model_labels_compatible(
            color_id_to_model_label(color_id), model_lock[0]) or
            not model_track[0]):
        return None
    lead_dx, lead_dy = track_lead_offset()
    output_box = output_box_from_track(lead_dx, lead_dy)
    coordinate_box = coordinate_box_from_track(lead_dx, lead_dy)
    set_color_tracking(color_id, output_box)
    return (color_id, output_box, coordinate_box, 3 if observed else 2)
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
    data = _tx_return_yellow_buf
    status = RETURN_STATUS_STOP if stop_requested else 0
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
    top = RETURN_STOP_ROI[1]
    bottom = top + RETURN_STOP_ROI[3] - 1
    return top - RETURN_STOP_HORIZONTAL_GUARD <= y <= bottom + RETURN_STOP_HORIZONTAL_GUARD
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
    best_x = None
    best_pixels = -1
    if blobs:
        for blob in blobs:
            w = blob.w()
            h = blob.h()
            if (h < RETURN_STOP_MIN_BLOB_H or
                    w * 100 > h * RETURN_STOP_MAX_WIDTH_HEIGHT_X100):
                continue
            x = blob.cx()
            if best_x is None or x > best_x or (x == best_x and blob.pixels() > best_pixels):
                best_x = x
                best_pixels = blob.pixels()
    return best_x
def detect_return_yellow_y(img):
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_YELLOW_ROI,
            pixels_threshold=RETURN_YELLOW_MIN_PIXELS,
            area_threshold=RETURN_YELLOW_MIN_AREA, merge=True)
    except Exception:
        return None
    best_y = None
    best_top = 241
    best_pixels = -1
    if blobs:
        for blob in blobs:
            top = blob.y()
            if (best_y is None or top < best_top or
                    (top == best_top and blob.pixels() > best_pixels)):
                best_y = blob.cy()
                best_top = top
                best_pixels = blob.pixels()
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
H_PIX2WORLD = (
    0.5835117773019284, -0.0026766595289079054, -92.13597430406873,
    9.349246523159213e-17, -0.33832976445396207, 118.77730192719507,
    5.8936449814456205e-18, 0.01820128479657392,
)
def box_to_world(x, y, w, h):
    px = x + w * 0.5
    py = y + h
    den = H_PIX2WORLD[6] * px + H_PIX2WORLD[7] * py + 1.0
    if -1e-10 < den < 1e-10:
        return None
    wx = (H_PIX2WORLD[0] * px + H_PIX2WORLD[1] * py + H_PIX2WORLD[2]) / den
    wy = (H_PIX2WORLD[3] * px + H_PIX2WORLD[4] * py + H_PIX2WORLD[5]) / den
    if not (wy > 0.0 and wy <= WORLD_Y_MAX_CM):
        return None
    if not (-WORLD_X_LIMIT_CM <= wx <= WORLD_X_LIMIT_CM):
        return None
    return (wx, wy)
_tx_world_buf = bytearray(16)
_tx_world_no_target_buf = bytearray(16)
_tx_world_buf[0] = _tx_world_no_target_buf[0] = 0xAA
_tx_world_buf[1] = _tx_world_no_target_buf[1] = 0x55
def send_world_data(color_id, wx_mm, wy_mm, pw):
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
    uart.write(_tx_world_no_target_buf)
def receive_command_from_host():
    global lost_frame_count, openart_mode
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
        if command == 0x03:
            if 1 <= param <= len(all_color_thresholds):
                target_color_id = param
                host_color_id_received = True
                lost_frame_count = 0
                color_track_active = False
                color_track_box = None
                color_track_color_id = 0
                color_lost_count = 0
                apply_host_hybrid_color(param)
        elif command == 0x01:
            openart_mode = MODE_SEARCH
            reset_return_yellow_state()
        elif command == 0x04:
            pass
        elif command == 0x05:
            pass
        elif command == 0x06:
            openart_mode = MODE_SEARCH
            reset_return_yellow_state()
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x07:
            openart_mode = MODE_RETURN
            front_scan_requested = False
            reset_front_scan_state()
            reset_return_yellow_state()
        elif command == 0x00 or command == 0x02:
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_return_yellow_state()
        return
frame_count = 0
init_model_runtime()
while True:
    frame_count += 1
    receive_command_from_host()
    img = snapshot_frame()
    if openart_mode == MODE_RETURN:
        process_return_yellow(img)
        maybe_collect(frame_count)
        continue
    lab_frame = (frame_count % CUT_UPDATE_INTERVAL == 0)
    if (model_lock[1] is not None and not model_color[3]
            and model_color[0] <= 0
            and frame_count % 2 == 0):
        lab_frame = True
    if lab_frame:
        update_dynamic_cut(img, frame_count)
    if process_front_scan_request(img):
        maybe_collect(frame_count)
        continue
    result = process_model_only_target(img, frame_count, not lab_frame)
    has_target = result is not None
    if has_target:
        send_color_id, output_box, coordinate_box, source = result
        x1, y1, w, h = output_box
        coord_x, coord_y, coord_w, coord_h = coordinate_box
    if has_target:
        world_point = box_to_world(coord_x, coord_y, coord_w, coord_h)
        if world_point is None:
            has_target = False
    if has_target:
        lost_frame_count = 0
        world_x, world_y = world_point
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
                restore_host_hybrid_lock()
            else:
                reset_target_tracking_state()
        send_world_no_target()
    maybe_collect(frame_count)
