# v1.1.0 MASTER: optional ID2-first gate; motion-tolerant 3-in-5 first lock.
# ==================== QUICK MATCH SETTINGS ====================
# Edit this block first when changing cameras, models, or SD files.
WB_GAINS = (92.00, 64.00, 101.00)
MODEL_PATH = '/sd/80lite0.5SS.tflite'
COLOR_THR_PATH = '/sd/color_thr.txt'
EXPOSURE_INIT = 880
EXPOSURE_MIN = 100
EXPOSURE_MAX = 4500
ENABLE_COMPLETED_COLOR_EXCLUSION = False
ID2_ABSOLUTE_PRIORITY = True
# Optional instance-level gate. With no 0x09 anchor command, behavior remains
# identical to the legacy color-only 0x03 mode.
ENABLE_TARGET_ANCHOR_LOCK = True
# ================== END QUICK MATCH SETTINGS ==================
import sensor, gc, math
try:
    import tf
except Exception:
    tf = None
from machine import UART
try:
    from machine import WDT
except Exception:
    WDT = None
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
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_hmirror(False)
sensor.set_vflip(True)
def snapshot_frame():
    return sensor.snapshot().replace(hmirror=True)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
def validate_exposure(value):
    if value < EXPOSURE_MIN or value > EXPOSURE_MAX:
        raise ValueError('exposure_us out of range')
    return value
def load_startup_exposure(path=COLOR_THR_PATH):
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('exposure_us='):
                    exposure = validate_exposure(
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
uart = UART(12, baudrate=115200)
all_color_thresholds = [
    (34, 100, -41, 4, -72, -22),
    (10, 80, 22, 122, -17, 93),
    (50, 100, -128, -27, 20, 127),
    (21, 52, -77, 25, 6, 99),
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
def _load_calibrated_params(path=COLOR_THR_PATH):
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
COLOR_MIN_PIXELS = 70
COLOR_MIN_AREA = 100
TENNIS_MIN_PIXELS = 80
TENNIS_MIN_AREA = 80
# RED_BAG_MAX_WIDTH_HEIGHT_X100 = 170  # 已停用：红沙包恢复通用沙袋形状规则
NEAR_NOISE_Y_MIN = 170
NEAR_NOISE_BOX_AREA = 400
_color_blob_limits = (
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (TENNIS_MIN_PIXELS, TENNIS_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
    (COLOR_MIN_PIXELS, COLOR_MIN_AREA),
)
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
CUT_BLOB_BOTTOM_MARGIN = 25
CUT_GAP_BRIDGE = 10
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
TRACK_MIN_IOU = 0.05
BRN_BEAR_MERGE_MARGIN = 12
WHT_BEAR_MERGE_MARGIN = 10
MODEL_COLOR_IDS = ((4, 5), (3,), (1, 2))
MODEL_CONTACT_OFF_X = (-1, -1, -1)
MODEL_CONTACT_OFF_Y = (0, 0, 0)
MODEL_NEAR_SCALE_W = (1.40, 1.35, 1.50)
MODEL_NEAR_SCALE_H = (1.65, 1.50, 1.55)
MODEL_SCORE_NEAR = 0.52
MODEL_SCORE_MID = 0.42
MODEL_SCORE_FAR = 0.30
MODEL_SCORE_LOCKED = 0.20
MODEL_DISTANCE_NEAR_CM = 12.0
MODEL_DISTANCE_MID_CM = 18.0
MODEL_DISTANCE_FAR_CM = 25.0
MODEL_LOCK_CONFIRM_FRAMES = 5
MODEL_LOST_FRAMES = 5
MODEL_HOLD_FRAMES = 5
MODEL_REFRESH_INTERVAL = 4
MODEL_MIN_BOX_SIDE = 4
MODEL_MIN_BOX_AREA = 24
MODEL_MATCH_CENTER2 = 130 * 130
MODEL_PENDING_CENTER2 = 80 * 80
FIRST_LOCK_SCORE_MIN = 0.30
FIRST_LOCK_WINDOW_FRAMES = 5
FIRST_LOCK_REQUIRED_HITS = 3
FIRST_LOCK_MATCH_CENTER_PX = 36
FIRST_LOCK_MATCH_CENTER2 = FIRST_LOCK_MATCH_CENTER_PX * FIRST_LOCK_MATCH_CENTER_PX
FIRST_LOCK_SIZE_DELTA_PERCENT = 50
FIRST_LOCK_NEARER_MARGIN_CM = 0.0
HOST_FORCED_FIRST_LOCK_SCORE_MIN = 0.25
HOST_FORCED_FIRST_LOCK_WINDOW_FRAMES = 5
HOST_FORCED_FIRST_LOCK_REQUIRED_HITS = 3
HOST_FORCED_FIRST_LOCK_MATCH_CENTER_PX = 36
HOST_FORCED_FIRST_LOCK_MATCH_CENTER2 = (
    HOST_FORCED_FIRST_LOCK_MATCH_CENTER_PX *
    HOST_FORCED_FIRST_LOCK_MATCH_CENTER_PX)
HOST_FORCED_FIRST_LOCK_SIZE_DELTA_PERCENT = 50
HOST_FORCED_COLOR_SAMPLE_MAX_IQR = (65, 70, 85)
TENNIS_TRACK_MIN_PIXELS = 30
TENNIS_TRACK_MIN_AREA = 36
ENABLE_TENNIS_LINE_FILTER = False
TENNIS_MODEL_ASPECT_MIN_X100 = 20
TENNIS_MODEL_ASPECT_MAX_X100 = 500
TENNIS_LINE_CONTEXT_PAD_PERCENT = 120
TENNIS_LINE_CONTEXT_MIN_PAD = 18
TENNIS_LINE_MIN_PIXELS = 12
TENNIS_LINE_MIN_AREA = 18
TENNIS_LINE_MERGE_MARGIN = 4
TENNIS_LINE_ASPECT_X100 = 500
TENNIS_LINE_ELONGATION_MIN = 0.95
TENNIS_LINE_DIAGONAL_MAX_DENSITY = 0.12
TENNIS_LINE_EXTEND_X100 = 300
TENNIS_LINE_MODEL_OVERLAP_PERCENT = 60
COLOR_CONFIRM_FRAMES = 3
COLOR_ROI_INSET_X_PERCENT = 5
COLOR_ROI_INSET_TOP_PERCENT = 5
COLOR_ROI_INSET_BOTTOM_PERCENT = 10
COLOR_SAMPLE_INSET_X_PERCENT = 20
COLOR_SAMPLE_INSET_TOP_PERCENT = 15
COLOR_SAMPLE_INSET_BOTTOM_PERCENT = 25
COLOR_SAMPLE_MAX_IQR = (50, 55, 65)
COLOR_CLASS_DISTANCE_MARGIN = 60
COLOR_DYNAMIC_IQR_EXPAND_X100 = 75
COLOR_DYNAMIC_MARGIN = (5, 4, 4)
COLOR_DYNAMIC_MIN_SPANS = (
    (28, 18, 20), (24, 24, 24), (30, 28, 35),
    (24, 20, 24), (24, 18, 20),
)
COLOR_DYNAMIC_MAX_SPANS = (
    (75, 55, 65), (65, 65, 65), (70, 70, 90),
    (60, 75, 75), (55, 55, 55),
)
COLOR_DYNAMIC_BASE_EXPAND = (100, 128, 128)
COLOR_DYNAMIC_UPDATE_ALPHA_X100 = 30
COLOR_DYNAMIC_PENDING_CENTER_MAX = (12, 10, 12)
COLOR_TRACK_MODEL_PAD_PERCENT = 50
COLOR_TRACK_LOCAL_PAD_PERCENT = 45
COLOR_TRACK_MIN_PAD = 6
COLOR_TRACK_MAX_MISSES = 2
COLOR_OUTPUT_HOLD_FRAMES = MODEL_HOLD_FRAMES
COLOR_TRACK_MIN_COVER_X1000 = 20
COLOR_TRACK_GATE_OVERLAP_PERCENT = 70
COLOR_TRACK_AREA_MIN_PERCENT = 40
COLOR_TRACK_AREA_MAX_PERCENT = 250
COLOR_TRACK_CENTER_SCALE_X100 = 90
OUTPUT_SMOOTH_ALPHA_X100 = 70
OUTPUT_SMOOTH_RESET_CENTER2 = 36 * 36
CONTACT_JITTER_PX = 1.0
CONTACT_JITTER2 = CONTACT_JITTER_PX * CONTACT_JITTER_PX
CONTACT_REJECT_JUMP2 = MODEL_MATCH_CENTER2
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
               0.0, 0.0, 0, 0]
model_last_frame = -MODEL_REFRESH_INTERVAL
model_last_score = 0.0
first_lock_reset_cycle_active = False
first_lock_pending_label = -1
first_lock_pending_box = None
first_lock_pending_hits = 0
first_lock_pending_samples = 0
first_lock_pending_boxes = []
first_lock_pending_scores = []
first_lock_pending_color_ids = []
first_lock_pending_color_thresholds = []
adaptive_color_thresholds = [None, None, None, None, None]
color_adapt_pending_id = 0
color_adapt_pending_threshold = None
color_adapt_pending_count = 0
color_blob_box = None
color_coordinate_anchor_blob_x = None
color_coordinate_anchor_blob_y = None
color_coordinate_anchor_output_box = None
color_coordinate_anchor_box = None
dynamic_cut_left_y = DETECT_Y_MIN
dynamic_cut_valid = False
dynamic_cut_miss_count = 0
dynamic_detect_roi = DETECT_ROI
target_color_id = 0
host_color_id_received = False
completed_color_mask = 0
target_anchor_active = False
target_anchor_color_id = 0
target_anchor_x_cm = 0.0
target_anchor_y_cm = 0.0
target_anchor_radius_cm = 0.0
target_anchor_sequence = 0
color_track_active = False
color_track_box = None
color_track_coordinate_box = None
color_track_color_id = 0
color_lost_count = 0
_cmd_rx_buf = bytearray()
front_scan_requested = False
CMD_CLEAR_COMPLETED = 0x08
TARGET_ANCHOR_COMMAND = 0x09
TARGET_ANCHOR_FRAME_SIZE = 11
FRONT_SCAN_PACKET_ID = 0xC7
FRONT_SCAN_EXCLUDE_IOU = 0.20
FRONT_SCAN_EXCLUDE_CENTER_PX = 35
FRONT_SCAN_EXCLUDE_CENTER2 = FRONT_SCAN_EXCLUDE_CENTER_PX * FRONT_SCAN_EXCLUDE_CENTER_PX
FRONT_SCAN_Y_MAX = 150
FRONT_SCAN_SCORE_MIN = FIRST_LOCK_SCORE_MIN
FRONT_SCAN_STABLE_FRAMES = 6
FRONT_SCAN_MAX_FRAMES = 12
front_scan_last_current_id = 0
front_scan_last_mask = -1
front_scan_last_count = 0
front_scan_stable_count = 0
front_scan_total_count = 0
yellow_threshold = [(27, 100, -55, 16, 21, 105)]
ENABLE_YELLOW_DRAW = True
TENNIS_LINE_THRESHOLD = yellow_threshold
RETURN_YELLOW_PACKET_ID = 0xC8
RETURN_YELLOW_THRESHOLD = yellow_threshold
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
YELLOW_ROI_TOP = (0, 90, 320, 20)
YELLOW_ROI_BOTTOM = (0, 130, 320, 20)
YELLOW_ENTER_PIXELS = 70
YELLOW_KEEP_PIXELS = 20
YELLOW_CARRY_CONFIRM_FRAMES = 2
YELLOW_MIN_FIT_DX = 30
YELLOW_MAX_FIT_SLOPE_X100 = 100
YELLOW_TARGET_OVERLAP_PERCENT = 60
YELLOW_BOTTOM_Y = 220
YELLOW_LOST_THRESHOLD = 2
yellow_line_k = 0.0
yellow_line_b = 0.0
yellow_bottom_visible = False
yellow_detected = False
yellow_tracking = False
yellow_lost_count = 0
yellow_seen_in_carry = False
yellow_bottom_reached_in_carry = False
yellow_carry_confirm_count = 0
YELLOW_CARRY_IGNORE_FRAMES = 4
carry_start_frame = -1
MODE_SEARCH = 0
MODE_CARRY = 1
MODE_WAIT_TURN = 2
MODE_RETURN = 3
openart_mode = MODE_SEARCH
POS_NO_BOUNDARY = 0x00
POS_RIGHT_SIDE  = 0x01
POS_CROSSED     = 0x02
def color_id_completed(color_id):
    return (1 <= color_id <= len(all_color_thresholds) and
            bool(completed_color_mask & (1 << (color_id - 1))))
def color_id_available_for_search(color_id):
    if color_id < 1 or color_id > len(all_color_thresholds):
        return False
    if (ENABLE_COMPLETED_COLOR_EXCLUSION and
            color_id_completed(color_id)):
        return False
    if ID2_ABSOLUTE_PRIORITY and not color_id_completed(2):
        return color_id == 2
    return True
def host_forced_target_active():
    return (host_color_id_received and
            1 <= target_color_id <= len(all_color_thresholds) and
            color_id_available_for_search(target_color_id))
def mark_color_completed(color_id):
    global completed_color_mask
    if color_id_available_for_search(color_id):
        completed_color_mask |= 1 << (color_id - 1)
def clear_completed_carry_state():
    global completed_color_mask
    completed_color_mask = 0
def active_selected_color_id():
    if host_color_id_received and 1 <= target_color_id <= len(all_color_thresholds):
        return target_color_id
    if color_track_active and 1 <= color_track_color_id <= len(all_color_thresholds):
        return color_track_color_id
    if 1 <= model_color[0] <= len(all_color_thresholds):
        return model_color[0]
    return 0
def clear_target_anchor():
    global target_anchor_active, target_anchor_color_id
    global target_anchor_x_cm, target_anchor_y_cm
    global target_anchor_radius_cm, target_anchor_sequence
    target_anchor_active = False
    target_anchor_color_id = 0
    target_anchor_x_cm = 0.0
    target_anchor_y_cm = 0.0
    target_anchor_radius_cm = 0.0
    target_anchor_sequence = 0
def update_target_anchor(color_id, x_mm, y_mm, radius_cm, sequence):
    global target_anchor_active, target_anchor_color_id
    global target_anchor_x_cm, target_anchor_y_cm
    global target_anchor_radius_cm, target_anchor_sequence
    same_instance = (target_anchor_active and
                     target_anchor_color_id == color_id and
                     target_anchor_sequence == sequence)
    target_anchor_active = True
    target_anchor_color_id = color_id
    target_anchor_x_cm = x_mm * 0.1
    target_anchor_y_cm = y_mm * 0.1
    target_anchor_radius_cm = float(radius_cm)
    target_anchor_sequence = sequence
    return same_instance
def target_anchor_enabled_for(color_id):
    return (ENABLE_TARGET_ANCHOR_LOCK and target_anchor_active and
            color_id == target_anchor_color_id and
            target_anchor_radius_cm > 0.0)
def target_anchor_world_distance2(color_id, world_point):
    if not target_anchor_enabled_for(color_id) or world_point is None:
        return None
    dx = world_point[0] - target_anchor_x_cm
    dy = world_point[1] - target_anchor_y_cm
    distance2 = dx * dx + dy * dy
    if distance2 > target_anchor_radius_cm * target_anchor_radius_cm:
        return None
    return distance2
def target_anchor_candidate_distance2(label, box):
    if not target_anchor_enabled_for(target_color_id):
        return None
    if (label < 0 or label >= len(MODEL_COLOR_IDS) or
            target_anchor_color_id not in MODEL_COLOR_IDS[label]):
        return None
    return target_anchor_world_distance2(
        target_color_id, box_to_world(box[0], box[1], box[2], box[3]))
def target_anchor_model_rank(label, box, base_rank):
    if not target_anchor_enabled_for(target_color_id):
        return base_rank
    distance2 = target_anchor_candidate_distance2(label, box)
    if distance2 is None:
        return None
    return (-distance2, base_rank)
def apply_target_anchor_command(color_id, x_mm, y_mm, radius_cm, sequence):
    global lost_frame_count, target_color_id, host_color_id_received
    if color_id < 1 or color_id > len(all_color_thresholds):
        return False
    if not color_id_available_for_search(color_id):
        reset_target_tracking_state()
        return False
    same_target = host_color_id_received and target_color_id == color_id
    anchor_requested = ENABLE_TARGET_ANCHOR_LOCK and radius_cm > 0
    if anchor_requested:
        same_instance = update_target_anchor(
            color_id, x_mm, y_mm, radius_cm, sequence)
    else:
        clear_target_anchor()
        same_instance = same_target
    target_color_id = color_id
    host_color_id_received = True
    lost_frame_count = 0
    if anchor_requested:
        if not same_instance:
            reset_hybrid_tracking()
            model_color[3] = True
    elif not same_target:
        apply_host_hybrid_color(color_id)
    return same_instance
def reset_target_tracking_state():
    global lost_frame_count
    global target_color_id, host_color_id_received
    global color_track_active, color_track_box, color_track_coordinate_box
    global color_track_color_id, color_lost_count
    lost_frame_count = 0
    target_color_id = 0
    host_color_id_received = False
    color_track_active = False
    color_track_box = None
    color_track_coordinate_box = None
    color_track_color_id = 0
    color_lost_count = 0
    clear_target_anchor()
    reset_hybrid_tracking()
def reset_yellow_state():
    global yellow_lost_count, yellow_seen_in_carry, yellow_tracking, yellow_detected
    global yellow_bottom_reached_in_carry, yellow_carry_confirm_count
    global yellow_bottom_visible, yellow_line_k, yellow_line_b
    yellow_lost_count = 0
    yellow_seen_in_carry = False
    yellow_bottom_reached_in_carry = False
    yellow_carry_confirm_count = 0
    yellow_tracking = False
    yellow_detected = False
    yellow_bottom_visible = False
    yellow_line_k = 0.0
    yellow_line_b = 0.0
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
WORLD_X_LIMIT_CM = 250.0
WORLD_Y_MAX_CM = 164.0
CAMERA_GROUND_MESH_PATH = '/sd/camera_ground_mesh.txt'
CAMERA_GROUND_MESH_ROLE = 'master'
GROUND_IMAGE_W = 320
GROUND_IMAGE_H = 240
GROUND_TRIANGLE_EPSILON = 1e-6
GROUND_HOMOGRAPHY_EPSILON = 1e-6
GROUND_OUTSIDE_DEADBAND_PX = 1.5
GROUND_REQUIRED_NEAR_Y_CM = 6.0
GROUND_CENTER_X_ON_IMAGE = True

MESH_INVALID = 0
MESH_VALID = 1
MESH_TOO_NEAR = 2
MESH_TOO_FAR = 3
MESH_LEFT = 4
MESH_RIGHT = 5
BOUNDARY_TO_STATUS = {
    1: MESH_TOO_NEAR,
    2: MESH_TOO_FAR,
    3: MESH_LEFT,
    4: MESH_RIGHT,
}

# Generated from ground_mesh_24_points_template.csv. The SD mesh replaces
# this global fallback with local 28-point triangle interpolation.
DEFAULT_GROUND_FALLBACK_H = (
    -17.7638385071, 0.28145619199, 2975.34856703,
    0.177873489376, 11.811411406, -3947.19080263,
    0.0130205434305, -0.715824718611, 1.0,
)

ground_mesh_triangles = ()
ground_mesh_boundaries = ()
ground_mesh_near_v_max = -1.0
ground_fallback_h = DEFAULT_GROUND_FALLBACK_H
_mesh_last_triangle = 0
ground_center_x_cache = [False] * GROUND_IMAGE_H

def _parse_ground_float_list(text, count):
    parts = text.split(',')
    if len(parts) != count:
        return None
    values = []
    for part in parts:
        value = float(part.strip())
        if value != value or value <= -1e9 or value >= 1e9:
            return None
        values.append(value)
    return values

def load_ground_projection(path=CAMERA_GROUND_MESH_PATH):
    try:
        rows = {}
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split('=', 1)
                if len(parts) != 2:
                    return None
                key = parts[0].strip()
                if key in rows:
                    return None
                if key.startswith('point') and key[5:].isdigit():
                    # pointN 行只用于生成 triangleN/boundaryN，运行时不读取，
                    # 仅占位保留重复键检测，不保留字符串本体
                    rows[key] = None
                else:
                    rows[key] = parts[1].strip()

        if int(rows.get('version', '0')) != 4:
            return None
        if rows.get('model') != 'triangle_mesh':
            return None
        if (rows.get('role') != CAMERA_GROUND_MESH_ROLE or
                rows.get('units') != 'cm'):
            return None
        if (int(rows.get('image_w', '0')) != GROUND_IMAGE_W or
                int(rows.get('image_h', '0')) != GROUND_IMAGE_H):
            return None
        if (int(rows.get('software_hmirror', '-1')) != 1 or
                int(rows.get('sensor_vflip', '-1')) != 1 or
                int(rows.get('lens_corr', '-1')) != 0):
            return None

        point_count = int(rows.get('point_count', '0'))
        grid_rows = int(rows.get('grid_rows', '0'))
        grid_columns = int(rows.get('grid_columns', '0'))
        if (grid_rows < 4 or grid_rows > 16 or grid_columns != 4 or
                point_count != grid_rows * grid_columns):
            return None

        max_x_cm = float(rows.get('max_x_cm', '0'))
        max_y_cm = float(rows.get('max_y_cm', '0'))
        if not (max_x_cm > 0.0 and max_x_cm <= WORLD_X_LIMIT_CM):
            return None
        if abs(max_y_cm - WORLD_Y_MAX_CM) > 1e-6:
            return None
        calibrated_y_min_cm = float(rows.get('calibrated_y_min_cm', '0'))
        calibrated_y_max_cm = float(rows.get('calibrated_y_max_cm', '0'))
        if not (0.0 < calibrated_y_min_cm < calibrated_y_max_cm):
            return None
        if calibrated_y_min_cm > GROUND_REQUIRED_NEAR_Y_CM + 1e-6:
            return None
        if abs(calibrated_y_max_cm - WORLD_Y_MAX_CM) > 1e-6:
            return None

        triangle_count = int(rows.get('triangle_count', '0'))
        expected_triangle_count = (grid_rows - 1) * (grid_columns - 1) * 2
        if triangle_count != expected_triangle_count or triangle_count > 96:
            return None
        triangles = []
        orientation_sign = 0
        for index in range(triangle_count):
            triangle_key = 'triangle{}'.format(index)
            values = _parse_ground_float_list(rows.get(triangle_key, ''), 12)
            if values is None:
                return None
            # 该行文本已解析完毕，立即释放，压低导入期峰值内存
            rows[triangle_key] = None
            u0, v0, x0, y0 = values[0], values[1], values[2], values[3]
            u1, v1, x1, y1 = values[4], values[5], values[6], values[7]
            u2, v2, x2, y2 = values[8], values[9], values[10], values[11]
            if (u0 < 0.0 or u0 >= GROUND_IMAGE_W or
                    v0 < 0.0 or v0 >= GROUND_IMAGE_H or
                    u1 < 0.0 or u1 >= GROUND_IMAGE_W or
                    v1 < 0.0 or v1 >= GROUND_IMAGE_H or
                    u2 < 0.0 or u2 >= GROUND_IMAGE_W or
                    v2 < 0.0 or v2 >= GROUND_IMAGE_H):
                return None
            if (abs(x0) > max_x_cm or abs(x1) > max_x_cm or
                    abs(x2) > max_x_cm or
                    y0 <= 0.0 or y0 > max_y_cm or
                    y1 <= 0.0 or y1 > max_y_cm or
                    y2 <= 0.0 or y2 > max_y_cm):
                return None
            denominator = ((v1 - v2) * (u0 - u2) +
                           (u2 - u1) * (v0 - v2))
            if not (denominator < -1.0 or denominator > 1.0):
                return None
            world_area2 = ((x1 - x0) * (y2 - y0) -
                           (x2 - x0) * (y1 - y0))
            if not (world_area2 < -1e-4 or world_area2 > 1e-4):
                return None
            sign = 1 if denominator * world_area2 > 0.0 else -1
            if orientation_sign == 0:
                orientation_sign = sign
            elif sign != orientation_sign:
                return None
            values.append(1.0 / denominator)
            triangles.append(tuple(values))

        boundary_count = int(rows.get('boundary_count', '0'))
        expected_boundary_count = (2 * (grid_rows - 1) +
                                   2 * (grid_columns - 1))
        if boundary_count != expected_boundary_count:
            return None
        boundaries = []
        boundary_counts = {1: 0, 2: 0, 3: 0, 4: 0}
        near_v_max = -1.0
        for index in range(boundary_count):
            values = _parse_ground_float_list(
                rows.get('boundary{}'.format(index), ''), 5)
            if values is None:
                return None
            boundary_code = int(values[0])
            if (values[0] != boundary_code or
                    boundary_code not in BOUNDARY_TO_STATUS):
                return None
            u0, v0, u1, v1 = values[1], values[2], values[3], values[4]
            if (u0 < 0.0 or u0 >= GROUND_IMAGE_W or
                    v0 < 0.0 or v0 >= GROUND_IMAGE_H or
                    u1 < 0.0 or u1 >= GROUND_IMAGE_W or
                    v1 < 0.0 or v1 >= GROUND_IMAGE_H):
                return None
            du = u1 - u0
            dv = v1 - v0
            length2 = du * du + dv * dv
            if length2 <= 1.0:
                return None
            boundaries.append((BOUNDARY_TO_STATUS[boundary_code],
                               u0, v0, u1, v1, length2))
            boundary_counts[boundary_code] += 1
            if boundary_code == 1:
                near_v_max = max(near_v_max, v0, v1)
        if (boundary_counts[1] != grid_columns - 1 or
                boundary_counts[2] != grid_columns - 1 or
                boundary_counts[3] != grid_rows - 1 or
                boundary_counts[4] != grid_rows - 1):
            return None

        if rows.get('fallback_model') != 'homography':
            return None
        fallback_h = _parse_ground_float_list(
            rows.get('fallback_h', ''), 9)
        if fallback_h is None:
            return None
        fallback_fit_max = float(rows.get('fallback_fit_max_cm', '1000'))
        if fallback_fit_max < 0.0 or fallback_fit_max > 10.0:
            return None
        determinant = (
            fallback_h[0] * (fallback_h[4] * fallback_h[8] -
                             fallback_h[5] * fallback_h[7]) -
            fallback_h[1] * (fallback_h[3] * fallback_h[8] -
                             fallback_h[5] * fallback_h[6]) +
            fallback_h[2] * (fallback_h[3] * fallback_h[7] -
                             fallback_h[4] * fallback_h[6])
        )
        if (-GROUND_HOMOGRAPHY_EPSILON < determinant <
                GROUND_HOMOGRAPHY_EPSILON):
            return None
        return (tuple(triangles), tuple(boundaries),
                near_v_max, tuple(fallback_h))
    except Exception:
        return None

def mesh_ground_pixel_to_world(px, py):
    # Resume the scan at the last matched triangle: consecutive contact
    # points almost always stay inside the same mesh cell.
    global _mesh_last_triangle
    triangles = ground_mesh_triangles
    triangle_count = len(triangles)
    if triangle_count == 0:
        return None
    epsilon = -GROUND_TRIANGLE_EPSILON
    start = _mesh_last_triangle
    if start >= triangle_count:
        start = 0
    for offset in range(triangle_count):
        index = start + offset
        if index >= triangle_count:
            index -= triangle_count
        triangle = triangles[index]
        # A 4-target assignment compiles to BUILD_TUPLE + UNPACK_SEQUENCE,
        # i.e. one heap tuple per triangle; index the tuple directly and
        # read the world coordinates only after the point is accepted.
        u2 = triangle[8]
        v2 = triangle[9]
        pu = px - u2
        pv = py - v2
        inverse_denominator = triangle[12]
        a = ((triangle[5] - v2) * pu +
             (u2 - triangle[4]) * pv) * inverse_denominator
        if a < epsilon:
            continue
        u0 = triangle[0]
        v0 = triangle[1]
        b = ((v2 - v0) * pu +
             (u0 - u2) * pv) * inverse_denominator
        if b < epsilon:
            continue
        c = 1.0 - a - b
        if c >= epsilon:
            _mesh_last_triangle = index
            wx = a * triangle[2] + b * triangle[6] + c * triangle[10]
            wy = a * triangle[3] + b * triangle[7] + c * triangle[11]
            if wy > WORLD_Y_MAX_CM:
                wy = WORLD_Y_MAX_CM
            if wy <= 0.0:
                return None
            if wx != wx:
                wx = 0.0
            elif wx < -WORLD_X_LIMIT_CM:
                wx = -WORLD_X_LIMIT_CM
            elif wx > WORLD_X_LIMIT_CM:
                wx = WORLD_X_LIMIT_CM
            return (wx, wy)
    return None

def nearest_ground_boundary(u, v):
    best_status = MESH_INVALID
    best_u = 0.0
    best_v = 0.0
    best_distance2 = 1e30
    for edge in ground_mesh_boundaries:
        status, u0, v0, u1, v1, length2 = edge
        du = u1 - u0
        dv = v1 - v0
        position = ((u - u0) * du + (v - v0) * dv) / length2
        if position < 0.0:
            position = 0.0
        elif position > 1.0:
            position = 1.0
        nearest_u = u0 + position * du
        nearest_v = v0 + position * dv
        delta_u = u - nearest_u
        delta_v = v - nearest_v
        distance2 = delta_u * delta_u + delta_v * delta_v
        if distance2 < best_distance2:
            best_distance2 = distance2
            best_status = status
            best_u = nearest_u
            best_v = nearest_v
    return (best_status, best_u, best_v)

def classify_ground_outside(u, v):
    nearest = nearest_ground_boundary(u, v)
    if v > ground_mesh_near_v_max + GROUND_OUTSIDE_DEADBAND_PX:
        return (MESH_TOO_NEAR, nearest[1], nearest[2])
    return nearest

def ground_homography_pixel_to_world(h, px, py):
    denominator = h[6] * px + h[7] * py + h[8]
    if (-GROUND_HOMOGRAPHY_EPSILON < denominator <
            GROUND_HOMOGRAPHY_EPSILON):
        return None
    wx = (h[0] * px + h[1] * py + h[2]) / denominator
    wy = (h[3] * px + h[4] * py + h[5]) / denominator
    if wx != wx or wy != wy:
        return None
    return (wx, wy)

def ground_far_limit_x(h, px):
    far_y = WORLD_Y_MAX_CM
    v_denominator = h[4] - far_y * h[7]
    if (-GROUND_HOMOGRAPHY_EPSILON < v_denominator <
            GROUND_HOMOGRAPHY_EPSILON):
        return None
    far_v = (far_y * (h[6] * px + h[8]) -
             (h[3] * px + h[5])) / v_denominator
    projected = ground_homography_pixel_to_world(h, px, far_v)
    return None if projected is None else projected[0]

def fallback_ground_pixel_to_world(px, py, mesh_status,
                                   boundary_u=None, boundary_v=None):
    h = ground_fallback_h
    correction_x = 0.0
    correction_y = 0.0
    boundary_world = None
    if boundary_u is not None and boundary_v is not None:
        boundary_world = mesh_ground_pixel_to_world(boundary_u, boundary_v)
        boundary_h = ground_homography_pixel_to_world(h, boundary_u, boundary_v)
        if boundary_world is not None and boundary_h is not None:
            correction_x = boundary_world[0] - boundary_h[0]
            correction_y = boundary_world[1] - boundary_h[1]

    use_far_limit = mesh_status == MESH_TOO_FAR
    projected = ground_homography_pixel_to_world(h, px, py)
    if projected is not None:
        wx = projected[0] + correction_x
        wy = projected[1] + correction_y
        if (mesh_status != MESH_TOO_FAR and
                -WORLD_X_LIMIT_CM <= wx <= WORLD_X_LIMIT_CM and
                0.0 < wy <= WORLD_Y_MAX_CM):
            return (wx, wy)
        if wy <= 0.0 or wy > WORLD_Y_MAX_CM:
            use_far_limit = True

    if mesh_status == MESH_TOO_NEAR or not use_far_limit:
        return None
    far_x = ground_far_limit_x(h, px)
    if far_x is None:
        return None
    if boundary_world is not None and boundary_u is not None:
        boundary_far_x = ground_far_limit_x(h, boundary_u)
        if mesh_status == MESH_TOO_FAR and boundary_far_x is not None:
            far_x += boundary_world[0] - boundary_far_x
        else:
            far_x += correction_x
    if far_x < -WORLD_X_LIMIT_CM or far_x > WORLD_X_LIMIT_CM:
        return None
    return (far_x, WORLD_Y_MAX_CM)

def ground_pixel_to_world(px, py):
    px = float(px)
    py = float(py)
    if (px < 0.0 or px >= GROUND_IMAGE_W or
            py < 0.0 or py >= GROUND_IMAGE_H):
        return None
    result = mesh_ground_pixel_to_world(px, py)
    if result is not None:
        return result
    if ground_mesh_boundaries:
        mesh_status, boundary_u, boundary_v = classify_ground_outside(px, py)
    else:
        mesh_status = MESH_INVALID
        boundary_u = None
        boundary_v = None
    return fallback_ground_pixel_to_world(
        px, py, mesh_status, boundary_u, boundary_v)

def center_line_world_x_for_row(row):
    # Lazy per-row cache of the u=160 centre-line projection used by
    # GROUND_CENTER_X_ON_IMAGE; False marks a row not yet computed.
    cached = ground_center_x_cache[row]
    if cached is False:
        world = ground_pixel_to_world(GROUND_IMAGE_W * 0.5, row + 0.5)
        cached = None if world is None else world[0]
        ground_center_x_cache[row] = cached
    return cached

_loaded_ground_projection = load_ground_projection()
if _loaded_ground_projection is not None:
    ground_mesh_triangles = _loaded_ground_projection[0]
    ground_mesh_boundaries = _loaded_ground_projection[1]
    ground_mesh_near_v_max = _loaded_ground_projection[2]
    ground_fallback_h = _loaded_ground_projection[3]
    print('[GROUND] loaded %d triangles from %s' %
          (len(ground_mesh_triangles), CAMERA_GROUND_MESH_PATH))
else:
    print('[GROUND] mesh unavailable; using embedded 28-point homography fallback')
del _loaded_ground_projection
gc.collect()

def clamp_int(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v
def world_cm_to_mm(value_cm):
    scaled = float(value_cm) * 10.0
    return int(scaled + 0.5) if scaled >= 0.0 else int(scaled - 0.5)
def pick_top_y_from_strip(blobs):
    if not blobs:
        return None
    # Hoist the invariants and read each blob accessor once per blob.
    min_h = CUT_BLOB_MIN_H
    bottom_limit = CUT_SCAN_Y_MAX - CUT_BLOB_BOTTOM_MARGIN
    top_y = None
    for b in blobs:
        bh = b.h()
        if bh < min_h:
            continue
        by = b.y()
        if by + bh < bottom_limit:
            continue
        if top_y is None or by < top_y:
            top_y = by
    if top_y is None:
        return None
    gap_bridge = CUT_GAP_BRIDGE
    bridged_top = None
    for b in blobs:
        bh = b.h()
        if bh < min_h:
            continue
        by = b.y()
        by2 = by + bh
        if by2 <= top_y and top_y - by2 <= gap_bridge and by < top_y:
            if bridged_top is None or by < bridged_top:
                bridged_top = by
    if bridged_top is not None:
        top_y = bridged_top
    return top_y
def update_dynamic_cut(img, frame_count):
    global dynamic_cut_left_y
    global dynamic_cut_valid, dynamic_cut_miss_count, dynamic_detect_roi
    if (not ENABLE_DYNAMIC_CUT) or (frame_count % CUT_UPDATE_INTERVAL != 0):
        return
    top_ys = []
    strip_xs = []
    for i in range(len(CUT_STRIP_ROIS)):
        roi = CUT_STRIP_ROIS[i]
        blobs = img.find_blobs(BLUE_GROUND_THRESHOLD, roi=roi,
                               pixels_threshold=CUT_MIN_PIXELS, area_threshold=CUT_MIN_AREA, merge=True)
        ty = pick_top_y_from_strip(blobs)
        if ty is not None:
            top_ys.append(ty)
            strip_xs.append(CUT_STRIP_XS[i])
    valid_strips = len(top_ys)
    if valid_strips >= CUT_MIN_VALID_STRIPS:
        if max(strip_xs) - min(strip_xs) < CUT_MIN_VALID_X_SPAN:
            valid_strips = 0
    top_y_pick = None
    if valid_strips >= CUT_MIN_VALID_STRIPS:
        top_ys.sort()
        pick_i = (valid_strips - 1) * 2 // 3
        top_y_pick = top_ys[pick_i]
        baffle_spread = (valid_strips >= 3 and
                         top_ys[pick_i] - top_ys[0] > CUT_BAFFLE_SPREAD_PX)
        if (not baffle_spread and valid_strips >= 2 and
                top_ys[1] - top_ys[0] > CUT_SINGLE_STRIP_MAX_LEAD):
            top_y_pick = top_ys[1]
    if valid_strips >= CUT_MIN_VALID_STRIPS:
        dynamic_cut_miss_count = 0
        if not dynamic_cut_valid:
            dynamic_cut_left_y = top_y_pick
            dynamic_cut_valid = True
        else:
            a = CUT_EMA_ALPHA
            delta = top_y_pick - dynamic_cut_left_y
            if delta < -CUT_MAX_STEP_UP:
                top_y_pick = dynamic_cut_left_y - CUT_MAX_STEP_UP
            elif delta > CUT_MAX_STEP_DOWN:
                top_y_pick = dynamic_cut_left_y + CUT_MAX_STEP_DOWN
            dynamic_cut_left_y = int(
                a * top_y_pick + (1.0 - a) * dynamic_cut_left_y)
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
# def red_bag_aspect_valid(width, height):
#     return (width > 0 and height > 0 and
#             width * 100 <= height * RED_BAG_MAX_WIDTH_HEIGHT_X100)
def valid_color_blob(blob, color_id, pixels_threshold_override=0):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return False
    box_area = w * h
    if box_area < NEAR_NOISE_BOX_AREA and blob.y() > NEAR_NOISE_Y_MIN:
        return False
    if color_id == 3:
        pixels_threshold = TENNIS_TRACK_MIN_PIXELS
        area_threshold = TENNIS_TRACK_MIN_AREA
    else:
        pixels_threshold, area_threshold = _color_blob_limits[color_id - 1]
    if pixels_threshold_override > 0:
        pixels_threshold = pixels_threshold_override
    if blob.pixels() < pixels_threshold or box_area < area_threshold:
        return False
    if color_id == 3:
        if w * 100 < h * 35 or w * 100 > h * 230:
            return False
        if blob.density() < 0.25:
            return False
    elif color_id == 4 or color_id == 5:
        if w * 100 < h * 30 or w * 100 > h * 250:
            return False
        if blob.density() < 0.25:
            return False
    # elif color_id == 2:  # 红沙包专用长宽比过滤已停用
    #     if w * 100 < h * 60 or not red_bag_aspect_valid(w, h):
    #         return False
    #     if blob.density() < 0.40:
    #         return False
    else:
        if w * 100 < h * 60 or w * 100 > h * 180:
            return False
        if blob.density() < 0.40:
            return False
    return True
def color_id_to_model_label(color_id):
    if color_id == 1 or color_id == 2:
        return 2
    if color_id == 3:
        return 1
    if color_id == 4 or color_id == 5:
        return 0
    return -1
def locally_trusted_model_color_id(label):
    if (color_id_available_for_search(model_color[0]) and
            color_id_to_model_label(model_color[0]) == label):
        if (color_adapt_pending_count > 0 and
                color_adapt_pending_id != model_color[0]):
            return 0
        return model_color[0]
    if 0 <= label < len(MODEL_COLOR_IDS):
        candidates = MODEL_COLOR_IDS[label]
        if (len(candidates) == 1 and
                color_id_available_for_search(candidates[0])):
            return candidates[0]
    return 0
def trusted_model_color_id(label):
    if (host_forced_target_active() and
            color_id_to_model_label(target_color_id) == label):
        return target_color_id
    return locally_trusted_model_color_id(label)
def reset_model_track():
    model_track[:] = [False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0, 0]
def reset_color_adaptation_pending():
    global color_adapt_pending_id, color_adapt_pending_threshold
    global color_adapt_pending_count
    color_adapt_pending_id = 0
    color_adapt_pending_threshold = None
    color_adapt_pending_count = 0
def reset_color_coordinate_anchor():
    global color_coordinate_anchor_blob_x
    global color_coordinate_anchor_blob_y
    global color_coordinate_anchor_output_box, color_coordinate_anchor_box
    color_coordinate_anchor_blob_x = None
    color_coordinate_anchor_blob_y = None
    color_coordinate_anchor_output_box = None
    color_coordinate_anchor_box = None
def reset_color_blob_search():
    global color_blob_box
    color_blob_box = None
    reset_color_coordinate_anchor()
def reset_color_blob_tracking():
    global color_track_active, color_track_box, color_track_coordinate_box
    global color_track_color_id
    global color_lost_count
    reset_color_blob_search()
    color_track_active = False
    color_track_box = None
    color_track_coordinate_box = None
    color_track_color_id = 0
    color_lost_count = 0
def reset_first_lock_pending():
    global first_lock_pending_label, first_lock_pending_box
    global first_lock_pending_hits, first_lock_pending_samples
    first_lock_pending_label = -1
    first_lock_pending_box = None
    first_lock_pending_hits = 0
    first_lock_pending_samples = 0
    first_lock_pending_boxes[:] = []
    first_lock_pending_scores[:] = []
    first_lock_pending_color_ids[:] = []
    first_lock_pending_color_thresholds[:] = []
def reset_hybrid_tracking():
    global model_last_frame, model_last_score
    model_lock[:] = [-1, None, -1, None, 0, 0]
    model_color[:] = [0, 0, 0, False]
    model_last_frame = -MODEL_REFRESH_INTERVAL
    model_last_score = 0.0
    reset_color_adaptation_pending()
    reset_color_blob_tracking()
    reset_model_track()
    reset_first_lock_pending()
def restore_host_hybrid_lock():
    color_id = target_color_id if host_color_id_received else 0
    reset_hybrid_tracking()
    if color_id > 0:
        model_color[3] = True
def apply_host_hybrid_color(color_id):
    label = color_id_to_model_label(color_id)
    locked_color_id = (locally_trusted_model_color_id(model_lock[0])
                       if model_lock[1] is not None else 0)
    if (model_lock[1] is None or
            model_lock[0] != label or
            locked_color_id != color_id):
        reset_hybrid_tracking()
    else:
        model_lock[2:6] = [-1, None, 0, 0]
        reset_color_adaptation_pending()
        reset_color_blob_tracking()
    model_color[:] = [0, 0, 0, True]
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
        feed_watchdog()
        model_net = tf.load(MODEL_PATH)
        model_fb = sensor.alloc_extra_fb(240, 240, sensor.RGB565)
        feed_watchdog()
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
def interpolate_score_by_distance(distance, near_score, mid_score, far_score):
    if distance <= MODEL_DISTANCE_NEAR_CM:
        return near_score
    if distance >= MODEL_DISTANCE_FAR_CM:
        return far_score
    if distance <= MODEL_DISTANCE_MID_CM:
        ratio = ((distance - MODEL_DISTANCE_NEAR_CM) /
                 (MODEL_DISTANCE_MID_CM - MODEL_DISTANCE_NEAR_CM))
        return near_score + (mid_score - near_score) * ratio
    ratio = ((distance - MODEL_DISTANCE_MID_CM) /
             (MODEL_DISTANCE_FAR_CM - MODEL_DISTANCE_MID_CM))
    return mid_score + (far_score - mid_score) * ratio
def model_box_distance(box):
    world_point = box_to_world(box[0], box[1], box[2], box[3])
    return (MODEL_DISTANCE_FAR_CM if world_point is None
            else world_point[1])
def model_score_minimum(label, box, locked):
    if locked:
        return MODEL_SCORE_LOCKED
    return interpolate_score_by_distance(
        model_box_distance(box), MODEL_SCORE_NEAR,
        MODEL_SCORE_MID, MODEL_SCORE_FAR)
def model_acquire_rank(box, score):
    distance_mm = int(model_box_distance(box) * 10.0 + 0.5)
    center_x = box[0] + box[2] // 2
    return (-distance_mm, box[1] + box[3], box[2] * box[3],
            -abs(center_x - 160))
def model_candidate_matches_requested_color(img, label, box):
    # Returns (matches, sampled); sampled is the (color_id, threshold)
    # pair when a color sample was taken, so callers can reuse it.
    candidates = MODEL_COLOR_IDS[label]
    if host_color_id_received and target_color_id > 0:
        if (not color_id_available_for_search(target_color_id) or
                target_color_id not in candidates):
            return False, None
        sampled = sample_model_color(img, label, box)
        return sampled[0] == target_color_id, sampled
    available_count = 0
    for color_id in candidates:
        if color_id_available_for_search(color_id):
            available_count += 1
    if available_count <= 0:
        return False, None
    if available_count == len(candidates):
        return True, None
    sampled = sample_model_color(img, label, box)
    return color_id_available_for_search(sampled[0]), sampled

def tennis_candidate_is_yellow_line(img, box):
    x, y, w, h = box
    if (w * 100 < h * TENNIS_MODEL_ASPECT_MIN_X100 or
            w * 100 > h * TENNIS_MODEL_ASPECT_MAX_X100):
        return True
    pad_x = max(TENNIS_LINE_CONTEXT_MIN_PAD,
                w * TENNIS_LINE_CONTEXT_PAD_PERCENT // 100)
    pad_y = max(TENNIS_LINE_CONTEXT_MIN_PAD,
                h * TENNIS_LINE_CONTEXT_PAD_PERCENT // 100)
    x1 = max(0, x - pad_x)
    y1 = max(0, y - pad_y)
    x2 = min(320, x + w + pad_x)
    y2 = min(COLOR_DETECT_Y_MAX, y + h + pad_y)
    if x2 <= x1 or y2 <= y1:
        return False
    try:
        try:
            blobs = img.find_blobs(
                TENNIS_LINE_THRESHOLD, roi=(x1, y1, x2 - x1, y2 - y1),
                pixels_threshold=TENNIS_LINE_MIN_PIXELS,
                area_threshold=TENNIS_LINE_MIN_AREA, merge=True,
                margin=TENNIS_LINE_MERGE_MARGIN)
        except TypeError:
            blobs = img.find_blobs(
                TENNIS_LINE_THRESHOLD, roi=(x1, y1, x2 - x1, y2 - y1),
                pixels_threshold=TENNIS_LINE_MIN_PIXELS,
                area_threshold=TENNIS_LINE_MIN_AREA, merge=True)
    except Exception:
        return False
    if not blobs:
        return False
    model_area = w * h
    for blob in blobs:
        blob_box = (blob.x(), blob.y(), blob.w(), blob.h())
        overlap = rect_intersection_area(blob_box, box)
        if (model_area <= 0 or overlap * 100 <
                model_area * TENNIS_LINE_MODEL_OVERLAP_PERCENT):
            continue
        bw = blob_box[2]
        bh = blob_box[3]
        elongated = (bw * 100 >= bh * TENNIS_LINE_ASPECT_X100 or
                     bh * 100 >= bw * TENNIS_LINE_ASPECT_X100)
        try:
            elongated = (elongated or
                         blob.elongation() >= TENNIS_LINE_ELONGATION_MIN)
        except Exception:
            pass
        try:
            elongated = (elongated or
                         blob.density() <= TENNIS_LINE_DIAGONAL_MAX_DENSITY)
        except Exception:
            pass
        extends = (bw * 100 >= w * TENNIS_LINE_EXTEND_X100 or
                   bh * 100 >= h * TENNIS_LINE_EXTEND_X100)
        if elongated and extends:
            return True
    return False
def model_copy_frame(img):
    global model_copy_to_fb_supported
    if not model_copy_to_fb_supported or model_fb is None:
        raise RuntimeError('preallocated model framebuffer unavailable')
    try:
        return img.copy(0.75, 1, copy_to_fb=model_fb).replace(hmirror=True)
    except TypeError:
        model_copy_to_fb_supported = False
        raise RuntimeError('copy_to_fb unsupported')
def run_model_best(img):
    global model_infer_error_count, model_last_frame
    desired_label = color_id_to_model_label(target_color_id)
    if desired_label < 0 and model_lock[1] is not None:
        desired_label = model_lock[0]
    try:
        model_last_frame = frame_count
        objects = tf.detect(model_net, model_copy_frame(img))
        best = None
        best_rank = None
        best_sampled = None
        locked = model_lock[1] is not None
        anchor = model_lock[1]
        if locked and color_track_active and color_track_box is not None:
            anchor = color_track_box
        # 首锁分数门限与确认次数整帧恒定，提前算好，
        # 避免每个候选框重复走 host_forced_target_active() 三层调用链。
        if locked:
            acquire_minimum_score = 0.0
            acquire_confirm = 1
        else:
            host_forced = host_forced_target_active()
            acquire_minimum_score = (HOST_FORCED_FIRST_LOCK_SCORE_MIN
                                     if host_forced
                                     else FIRST_LOCK_SCORE_MIN)
            acquire_confirm = (HOST_FORCED_FIRST_LOCK_REQUIRED_HITS
                               if host_forced
                               else FIRST_LOCK_REQUIRED_HITS)
        frame_width = img.width()
        frame_height = img.height()
        label_count = len(MODEL_COLOR_IDS)
        for obj in objects:
            x1, y1, x2, y2, label_value, score_value = obj
            label = int(label_value)
            score = float(score_value)
            if (label < 0 or label >= label_count or
                    (desired_label >= 0 and label != desired_label)):
                continue
            left_ratio = float(x1)
            top_ratio = float(y1)
            model_x = int(left_ratio * frame_width)
            y = int(top_ratio * frame_height)
            w = int((float(x2) - left_ratio) * frame_width)
            h = int((float(y2) - top_ratio) * frame_height)
            # Model input is sensor-native; tracking uses the mirrored control frame.
            x = frame_width - model_x - w
            if (w < MODEL_MIN_BOX_SIDE or h < MODEL_MIN_BOX_SIDE or
                    w * h < MODEL_MIN_BOX_AREA):
                continue
            box = raw_model_box(x, y, w, h)
            minimum_score = (model_score_minimum(label, box, True)
                             if locked else acquire_minimum_score)
            if score < minimum_score:
                continue
            if (ENABLE_TENNIS_LINE_FILTER and label == 1 and
                    tennis_candidate_is_yellow_line(img, box)):
                continue
            if locked and not model_box_matches(
                    box, anchor, MODEL_MATCH_CENTER2):
                continue
            matches, sampled = model_candidate_matches_requested_color(
                img, label, box)
            if not matches:
                continue
            if locked:
                rank = (int(score * 100000) +
                        int(box_iou(box, anchor) * 50000) -
                        center_dist2(box, anchor))
                confirm = 1
            else:
                confirm = acquire_confirm
                rank = model_acquire_rank(box, score)
                rank = target_anchor_model_rank(label, box, rank)
                if rank is None:
                    continue
            if best is None or rank > best_rank:
                best = (label, box, score, confirm)
                best_rank = rank
                best_sampled = sampled
        model_infer_error_count = 0
        if best is not None:
            if locked:
                return best + (0, None)
            if best_sampled is None:
                best_sampled = sample_model_color(img, best[0], best[1])
            return best + best_sampled
        return None
    except Exception as error:
        model_infer_error_count += 1
        if model_infer_error_count >= 3:
            disable_model_runtime('three inference failures: ' + str(error))
        return None
def rect_intersection_area(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[0] + a[2], b[0] + b[2])
    y2 = min(a[1] + a[3], b[1] + b[3])
    if x2 <= x1 or y2 <= y1:
        return 0
    return (x2 - x1) * (y2 - y1)
def model_sample_roi(label, box):
    x, y, w, h = box
    inset_x_percent = COLOR_SAMPLE_INSET_X_PERCENT
    inset_top_percent = COLOR_SAMPLE_INSET_TOP_PERCENT
    inset_bottom_percent = COLOR_SAMPLE_INSET_BOTTOM_PERCENT
    if label == 1:
        inset_x_percent += 8
        inset_top_percent += 8
        inset_bottom_percent += 5
    inset_x = max(1, w * inset_x_percent // 100)
    inset_top = max(1, h * inset_top_percent // 100)
    inset_bottom = max(1, h * inset_bottom_percent // 100)
    x += inset_x
    y += inset_top
    w -= inset_x * 2
    h -= inset_top + inset_bottom
    y2 = min(y + h, COLOR_DETECT_Y_MAX)
    if w < 4 or y2 - y < 4:
        return None
    return (x, y, w, y2 - y)
def threshold_center_distance(lab, threshold):
    # 展开三通道：省掉 range 对象与逐次下标乘法。
    total = 0
    low = threshold[0]
    high = threshold[1]
    half_span = max(6, (high - low) // 2)
    total += abs(lab[0] * 2 - (low + high)) * 50 // half_span
    low = threshold[2]
    high = threshold[3]
    half_span = max(6, (high - low) // 2)
    total += abs(lab[1] * 2 - (low + high)) * 50 // half_span
    low = threshold[4]
    high = threshold[5]
    half_span = max(6, (high - low) // 2)
    total += abs(lab[2] * 2 - (low + high)) * 50 // half_span
    return total
def sample_color_id(label, lab, forced_color_id=0):
    if label < 0 or label >= len(MODEL_COLOR_IDS):
        return 0
    candidates = MODEL_COLOR_IDS[label]
    best_id = 0
    best_distance = None
    second_distance = None
    for color_id in candidates:
        distance = threshold_center_distance(
            lab, all_color_thresholds[color_id - 1])
        if best_distance is None or distance < best_distance:
            second_distance = best_distance
            best_distance = distance
            best_id = color_id
        elif second_distance is None or distance < second_distance:
            second_distance = distance
    if forced_color_id > 0:
        return forced_color_id if best_id == forced_color_id else 0
    if second_distance is not None:
        if second_distance - best_distance < COLOR_CLASS_DISTANCE_MARGIN:
            return 0
    return best_id
def sample_color_id_from_stats(label, sample, forced_color_id=0):
    median_id = sample_color_id(label, (sample[6], sample[7], sample[8]),
                                forced_color_id)
    if label != 0 or median_id <= 0:
        return median_id
    # A bear is accepted only when its median and darker quartile agree.
    lower_l_id = sample_color_id(
        label, (sample[0], sample[7], sample[8]), forced_color_id)
    return median_id if lower_l_id == median_id else 0
def build_dynamic_channel(q_low, q_high, median_value, min_span, max_span,
                          margin, base_low, base_high, channel):
    iqr = max(1, q_high - q_low)
    expand = max(margin, iqr * COLOR_DYNAMIC_IQR_EXPAND_X100 // 100)
    low = q_low - expand
    high = q_high + expand
    span = high - low
    if span < min_span:
        low = median_value - min_span // 2
        high = low + min_span
    elif span > max_span:
        low = median_value - max_span // 2
        high = low + max_span
    allowed_expand = COLOR_DYNAMIC_BASE_EXPAND[channel]
    low = max(low, base_low - allowed_expand)
    high = min(high, base_high + allowed_expand)
    # L 通道 0..100，a/b 通道 -128..127；直接比较，避免每次调用建两个元组。
    if channel == 0:
        low = max(low, 0)
        high = min(high, 100)
    else:
        low = max(low, -128)
        high = min(high, 127)
    if low >= high or median_value < low or median_value > high:
        return None
    return (int(low), int(high))
def build_dynamic_threshold(color_id, sample):
    base = all_color_thresholds[color_id - 1]
    min_spans = COLOR_DYNAMIC_MIN_SPANS[color_id - 1]
    max_spans = COLOR_DYNAMIC_MAX_SPANS[color_id - 1]
    # 展开三个通道：省掉 range 对象、中间 list 及 tuple() 转换，
    # 每次采样少约 4 次临时分配；失败短路顺序与原来完全一致。
    l_pair = build_dynamic_channel(
        sample[0], sample[1], sample[6],
        min_spans[0], max_spans[0], COLOR_DYNAMIC_MARGIN[0],
        base[0], base[1], 0)
    if l_pair is None:
        return None
    a_pair = build_dynamic_channel(
        sample[2], sample[3], sample[7],
        min_spans[1], max_spans[1], COLOR_DYNAMIC_MARGIN[1],
        base[2], base[3], 1)
    if a_pair is None:
        return None
    b_pair = build_dynamic_channel(
        sample[4], sample[5], sample[8],
        min_spans[2], max_spans[2], COLOR_DYNAMIC_MARGIN[2],
        base[4], base[5], 2)
    if b_pair is None:
        return None
    return (l_pair[0], l_pair[1], a_pair[0], a_pair[1],
            b_pair[0], b_pair[1])
def blend_threshold(old, new, alpha_x100):
    mixed = []
    for index in range(6):
        value = (old[index] * (100 - alpha_x100) +
                 new[index] * alpha_x100)
        mixed.append((value + 50) // 100)
    return tuple(mixed)
def threshold_centers_close(first, second):
    if first is None or second is None:
        return False
    for channel in range(3):
        first_center_x2 = first[channel * 2] + first[channel * 2 + 1]
        second_center_x2 = second[channel * 2] + second[channel * 2 + 1]
        if (abs(first_center_x2 - second_center_x2) >
                COLOR_DYNAMIC_PENDING_CENTER_MAX[channel] * 2):
            return False
    return True
def sample_box_lab_stats(img, label, box, max_iqr):
    roi = model_sample_roi(label, box)
    if roi is None:
        return None
    try:
        stats = img.get_statistics(roi=roi)
        sample = (
            stats.l_lq(), stats.l_uq(),
            stats.a_lq(), stats.a_uq(),
            stats.b_lq(), stats.b_uq(),
            stats.l_median(), stats.a_median(), stats.b_median(),
        )
    except Exception:
        return None
    # 展开三通道 IQR 宽度检查：省掉 range 对象与逐次下标乘法。
    if (sample[1] - sample[0] > max_iqr[0] or
            sample[3] - sample[2] > max_iqr[1] or
            sample[5] - sample[4] > max_iqr[2]):
        return None
    return sample
def sample_model_color(img, label, box):
    forced_color_id = (target_color_id
                       if host_forced_target_active() else 0)
    max_iqr = (HOST_FORCED_COLOR_SAMPLE_MAX_IQR
               if forced_color_id > 0 else COLOR_SAMPLE_MAX_IQR)
    sample = sample_box_lab_stats(img, label, box, max_iqr)
    if sample is None:
        return 0, None
    color_id = sample_color_id_from_stats(
        label, sample, forced_color_id)
    if color_id <= 0:
        return 0, None
    if not color_id_available_for_search(color_id):
        return 0, None
    return color_id, build_dynamic_threshold(color_id, sample)
def confirm_model_color(observed_id, observed_threshold):
    global color_adapt_pending_id, color_adapt_pending_threshold
    global color_adapt_pending_count
    if (observed_id <= 0 or observed_threshold is None or
            not color_id_available_for_search(observed_id)):
        reset_color_adaptation_pending()
        return False
    if model_color[0] > 0 and model_color[0] != observed_id:
        model_color[0] = 0
        reset_color_blob_tracking()
    if (observed_id == color_adapt_pending_id and
            threshold_centers_close(
                observed_threshold, color_adapt_pending_threshold)):
        color_adapt_pending_threshold = blend_threshold(
            color_adapt_pending_threshold, observed_threshold, 50)
        color_adapt_pending_count += 1
    else:
        color_adapt_pending_id = observed_id
        color_adapt_pending_threshold = observed_threshold
        color_adapt_pending_count = 1
    if color_adapt_pending_count < COLOR_CONFIRM_FRAMES:
        return False
    if model_color[0] > 0 and model_color[0] != observed_id:
        reset_color_blob_tracking()
    model_color[0] = observed_id
    existing = adaptive_color_thresholds[observed_id - 1]
    if existing is None:
        adaptive_color_thresholds[observed_id - 1] = color_adapt_pending_threshold
    else:
        adaptive_color_thresholds[observed_id - 1] = blend_threshold(
            existing, color_adapt_pending_threshold,
            COLOR_DYNAMIC_UPDATE_ALPHA_X100)
    reset_color_adaptation_pending()
    return True
def update_model_guided_color(img, score):
    if model_lock[1] is None or model_lock[0] < 0:
        reset_color_adaptation_pending()
        return False
    if score < model_score_minimum(
            model_lock[0], model_lock[1], False):
        reset_color_adaptation_pending()
        return False
    color_id, threshold = sample_model_color(
        img, model_lock[0], model_lock[1])
    return confirm_model_color(color_id, threshold)
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
def expand_tracking_box(box, percent, minimum_pad):
    x, y, w, h = box
    pad_x = max(minimum_pad, w * percent // 100)
    pad_y = max(minimum_pad, h * percent // 100)
    x1 = max(0, x - pad_x)
    y1 = max(0, y - pad_y)
    x2 = min(320, x + w + pad_x)
    y2 = min(COLOR_DETECT_Y_MAX, y + h + pad_y)
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2 - x1, y2 - y1)
def intersect_rois(first, second):
    if first is None or second is None:
        return None
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[0] + first[2], second[0] + second[2])
    y2 = min(first[1] + first[3], second[1] + second[3])
    if x2 <= x1 or y2 <= y1:
        return None
    return (x1, y1, x2 - x1, y2 - y1)
def union_rois(first, second):
    if first is None:
        return second
    if second is None:
        return first
    x1 = min(first[0], second[0])
    y1 = min(first[1], second[1])
    x2 = max(first[0] + first[2], second[0] + second[2])
    y2 = max(first[1] + first[3], second[1] + second[3])
    return (x1, y1, x2 - x1, y2 - y1)
def color_tracking_gate(model_observed=False):
    field_roi = intersect_rois(
        dynamic_detect_roi, (0, 0, 320, COLOR_DETECT_Y_MAX))
    if field_roi is None or model_lock[1] is None:
        return None
    model_gate = expand_tracking_box(
        model_lock[1], COLOR_TRACK_MODEL_PAD_PERCENT, COLOR_TRACK_MIN_PAD)
    if color_blob_box is not None and not model_observed:
        blob_gate = expand_tracking_box(
            color_blob_box, COLOR_TRACK_LOCAL_PAD_PERCENT,
            COLOR_TRACK_MIN_PAD)
        model_gate = union_rois(model_gate, blob_gate)
    return intersect_rois(model_gate, field_roi)
def color_tracking_search_roi(model_observed=False):
    tracking_gate = color_tracking_gate(model_observed)
    if tracking_gate is None:
        return None
    if model_observed or color_blob_box is None:
        return intersect_rois(model_color_roi(model_lock[1]), tracking_gate)
    local_percent = (COLOR_TRACK_LOCAL_PAD_PERCENT +
                     min(color_lost_count, COLOR_TRACK_MAX_MISSES) * 10)
    local_roi = expand_tracking_box(
        color_blob_box, local_percent, COLOR_TRACK_MIN_PAD)
    return intersect_rois(local_roi, tracking_gate)
def find_adaptive_color_blobs(img, roi, color_id, keep_base_minimum=False):
    threshold = adaptive_color_thresholds[color_id - 1]
    if threshold is None or roi is None:
        return None, 0
    roi_area = roi[2] * roi[3]
    base_pixels, base_area = _color_blob_limits[color_id - 1]
    if color_id == 3:
        base_pixels = TENNIS_TRACK_MIN_PIXELS
        base_area = TENNIS_TRACK_MIN_AREA
    minimum = base_pixels
    if not keep_base_minimum:
        minimum = max(
            minimum, roi_area * COLOR_TRACK_MIN_COVER_X1000 // 1000)
    area_minimum = max(base_area, minimum)
    try:
        if color_id == 4 or color_id == 5:
            margin = (BRN_BEAR_MERGE_MARGIN if color_id == 4
                      else WHT_BEAR_MERGE_MARGIN)
            try:
                blobs = img.find_blobs(
                    [threshold], roi=roi, pixels_threshold=minimum,
                    area_threshold=area_minimum, merge=True, margin=margin)
                return blobs, minimum
            except TypeError:
                pass
        blobs = img.find_blobs(
            [threshold], roi=roi, pixels_threshold=minimum,
            area_threshold=area_minimum, merge=True)
        return blobs, minimum
    except Exception:
        return None, minimum
def strict_blob_candidate(blob, color_id, reference, model_gate,
                          pixels_threshold):
    if not valid_color_blob(blob, color_id, pixels_threshold):
        return False
    box = (blob.x(), blob.y(), blob.w(), blob.h())
    box_area = box[2] * box[3]
    overlap = rect_intersection_area(box, model_gate)
    if (box_area <= 0 or overlap * 100 <
            box_area * COLOR_TRACK_GATE_OVERLAP_PERCENT):
        return False
    if reference is None:
        max_distance = max(
            12, max(model_lock[1][2], model_lock[1][3],
                    box[2], box[3]) *
            COLOR_TRACK_CENTER_SCALE_X100 // 100)
        if center_dist2(box, model_lock[1]) > max_distance * max_distance:
            return False
        return True
    reference_area = reference[2] * reference[3]
    if (box_area * 100 < reference_area * COLOR_TRACK_AREA_MIN_PERCENT or
            box_area * 100 >
            reference_area * COLOR_TRACK_AREA_MAX_PERCENT):
        return False
    max_distance = max(
        12, max(reference[2], reference[3]) *
        COLOR_TRACK_CENTER_SCALE_X100 // 100 +
        color_lost_count * COLOR_TRACK_MIN_PAD)
    return center_dist2(box, reference) <= max_distance * max_distance
def pick_tracking_blob(blobs, color_id, reference, model_gate,
                       pixels_threshold):
    best = None
    best_rank = None
    if not blobs:
        return None
    anchor = reference if reference is not None else model_lock[1]
    for blob in blobs:
        if not strict_blob_candidate(
                blob, color_id, reference, model_gate, pixels_threshold):
            continue
        box = (blob.x(), blob.y(), blob.w(), blob.h())
        rank = (blob.pixels() * 100 +
                rect_intersection_area(box, anchor) * 10 -
                center_dist2(box, anchor))
        if best is None or rank > best_rank:
            best = box
            best_rank = rank
    return best
def track_color_in_model_roi(img, color_id, model_observed=False):
    global color_blob_box, color_lost_count
    roi = color_tracking_search_roi(model_observed)
    tracking_gate = color_tracking_gate(model_observed)
    blobs, minimum = find_adaptive_color_blobs(img, roi, color_id)
    reference = None if model_observed else color_blob_box
    picked = pick_tracking_blob(
        blobs, color_id, reference, tracking_gate, minimum)
    if picked is None:
        color_lost_count += 1
        if color_lost_count > COLOR_TRACK_MAX_MISSES:
            reset_color_blob_search()
        return None
    color_blob_box = picked
    color_lost_count = 0
    return picked
def translate_tracking_box(box, dx, dy):
    center_x = box[0] + box[2] * 0.5 + dx
    center_y = box[1] + box[3] * 0.5 + dy
    return box_from_center(center_x, center_y, box[2], box[3])
def anchor_color_coordinate_geometry(blob_box):
    global color_coordinate_anchor_blob_x
    global color_coordinate_anchor_blob_y
    global color_coordinate_anchor_output_box, color_coordinate_anchor_box
    if not model_track[0]:
        reset_color_coordinate_anchor()
        return False
    color_coordinate_anchor_blob_x = blob_box[0] + blob_box[2] * 0.5
    color_coordinate_anchor_blob_y = blob_box[1] + blob_box[3] * 0.5
    color_coordinate_anchor_output_box = output_box_from_track()
    color_coordinate_anchor_box = coordinate_box_from_track()
    return True
def color_blob_geometry(blob_box, model_observed=False):
    if blob_box is None:
        return None
    box = raw_model_box(blob_box[0], blob_box[1],
                        blob_box[2], blob_box[3])
    if (model_observed or color_coordinate_anchor_box is None) and not (
            anchor_color_coordinate_geometry(box)):
        return None
    dx = ((box[0] + box[2] * 0.5) -
          color_coordinate_anchor_blob_x)
    dy = ((box[1] + box[3] * 0.5) -
          color_coordinate_anchor_blob_y)
    return (translate_tracking_box(
        color_coordinate_anchor_output_box, dx, dy),
        translate_tracking_box(color_coordinate_anchor_box, dx, dy))
def should_run_model(frame_index):
    if model_lock[1] is None:
        return True
    if color_blob_box is None or color_lost_count > 0 or model_lock[5] > 0:
        return True
    return frame_index - model_last_frame >= MODEL_REFRESH_INTERVAL

def coordinate_box_from_track():
    return box_from_center(model_track[5],
                           model_track[6] - model_track[10] * 0.5,
                           model_track[9], model_track[10])
def output_box_from_track():
    return box_from_center(model_track[1], model_track[2],
                           model_track[3], model_track[4])
def observe_model_box(label, box):
    raw_x = box[0] + box[2] * 0.5 + MODEL_CONTACT_OFF_X[label]
    raw_y = box[1] + box[3] + MODEL_CONTACT_OFF_Y[label]
    display = corrected_output_box(label, box)
    center_x = display[0] + display[2] * 0.5
    center_y = display[1] + display[3] * 0.5
    if not model_track[0]:
        model_track[:] = [True, center_x, center_y, float(display[2]),
                          float(display[3]), raw_x, raw_y, raw_x, raw_y,
                          box[2], box[3]]
        return True
    raw_dx = raw_x - model_track[7]
    raw_dy = raw_y - model_track[8]
    raw_distance2 = raw_dx * raw_dx + raw_dy * raw_dy
    if (raw_distance2 > CONTACT_REJECT_JUMP2 and model_lock[1] is not None and
            box_iou(box, model_lock[1]) < TRACK_MIN_IOU):
        return False
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
    model_track[9] = box[2]
    model_track[10] = box[3]
    return True
def first_lock_boxes_match(first, second):
    if first is None or second is None:
        return False
    host_forced = host_forced_target_active()
    center_limit2 = (HOST_FORCED_FIRST_LOCK_MATCH_CENTER2
                     if host_forced
                     else FIRST_LOCK_MATCH_CENTER2)
    size_delta_percent = (HOST_FORCED_FIRST_LOCK_SIZE_DELTA_PERCENT
                          if host_forced
                          else FIRST_LOCK_SIZE_DELTA_PERCENT)
    if center_dist2(first, second) > center_limit2:
        return False
    for index in (2, 3):
        largest = max(first[index], second[index])
        if (largest <= 0 or
                abs(first[index] - second[index]) * 100 >
                largest * size_delta_percent):
            return False
    return True
def first_lock_median_box():
    values = []
    middle = len(first_lock_pending_boxes) // 2
    for index in range(4):
        channel = sorted([box[index] for box in first_lock_pending_boxes])
        values.append(channel[middle])
    return tuple(values)
def first_lock_color_consensus():
    best_id = 0
    best_count = 0
    for color_id in range(1, len(all_color_thresholds) + 1):
        count = 0
        for observed_id in first_lock_pending_color_ids:
            if observed_id == color_id:
                count += 1
        if count > best_count:
            best_id = color_id
            best_count = count
    if (best_count < COLOR_CONFIRM_FRAMES or
            not color_id_available_for_search(best_id)):
        return 0, None
    selected = []
    for index in range(len(first_lock_pending_color_ids)):
        if (first_lock_pending_color_ids[index] == best_id and
                first_lock_pending_color_thresholds[index] is not None):
            selected.append(first_lock_pending_color_thresholds[index])
    if len(selected) < COLOR_CONFIRM_FRAMES:
        return 0, None
    values = []
    middle = len(selected) // 2
    for channel in range(6):
        ordered = sorted([threshold[channel] for threshold in selected])
        values.append(ordered[middle])
    return best_id, tuple(values)
def begin_first_lock_pending(candidate):
    global first_lock_pending_label, first_lock_pending_box
    global first_lock_pending_hits, first_lock_pending_samples
    label, box, score, _, color_id, color_threshold = candidate
    if not host_forced_target_active():
        model_color[:] = [0, 0, 0, False]
        reset_color_adaptation_pending()
    first_lock_pending_label = label
    first_lock_pending_box = box
    first_lock_pending_hits = 1
    first_lock_pending_samples = 1
    first_lock_pending_boxes[:] = [box]
    first_lock_pending_scores[:] = [score]
    first_lock_pending_color_ids[:] = [color_id]
    first_lock_pending_color_thresholds[:] = [color_threshold]
def commit_first_lock():
    global model_last_score
    label = first_lock_pending_label
    box = first_lock_median_box()
    color_id, color_threshold = first_lock_color_consensus()
    scores = sorted(first_lock_pending_scores)
    score = scores[len(scores) // 2]
    reset_first_lock_pending()
    model_lock[0] = label
    model_lock[1] = box
    model_lock[2:6] = [-1, None, 0, 0]
    reset_model_track()
    if not observe_model_box(label, box):
        model_lock[:] = [-1, None, -1, None, 0, 0]
        return False
    if color_id > 0 and color_threshold is not None:
        model_color[0] = color_id
        existing = adaptive_color_thresholds[color_id - 1]
        if existing is None:
            adaptive_color_thresholds[color_id - 1] = color_threshold
        else:
            adaptive_color_thresholds[color_id - 1] = blend_threshold(
                existing, color_threshold,
                COLOR_DYNAMIC_UPDATE_ALPHA_X100)
    model_last_score = score
    return True
def accept_first_lock_candidate(candidate):
    global first_lock_pending_box, first_lock_pending_hits
    global first_lock_pending_samples
    if first_lock_pending_box is None:
        if candidate is not None:
            begin_first_lock_pending(candidate)
        return False
    first_lock_pending_samples += 1
    if candidate is not None:
        label, box, score, _, color_id, color_threshold = candidate
        if (label == first_lock_pending_label and
                first_lock_boxes_match(box, first_lock_pending_box)):
            first_lock_pending_box = box
            first_lock_pending_hits += 1
            first_lock_pending_boxes.append(box)
            first_lock_pending_scores.append(score)
            first_lock_pending_color_ids.append(color_id)
            first_lock_pending_color_thresholds.append(color_threshold)
        else:
            if target_anchor_enabled_for(target_color_id):
                candidate_distance2 = target_anchor_candidate_distance2(
                    label, box)
                pending_distance2 = target_anchor_candidate_distance2(
                    first_lock_pending_label, first_lock_pending_box)
                candidate_preferred = (
                    candidate_distance2 is not None and
                    (pending_distance2 is None or
                     candidate_distance2 < pending_distance2))
            else:
                candidate_preferred = (
                    model_box_distance(box) + FIRST_LOCK_NEARER_MARGIN_CM <
                    model_box_distance(first_lock_pending_box))
            if candidate_preferred:
                begin_first_lock_pending(candidate)
                return False
    # Commit as soon as the active automatic or host-forced evidence threshold
    # is reached; do not wait for unused slots at the end of the window.
    # host_forced_target_active() 为纯查询，两处判定复用一次结果
    forced = host_forced_target_active()
    required_hits = (HOST_FORCED_FIRST_LOCK_REQUIRED_HITS
                     if forced
                     else FIRST_LOCK_REQUIRED_HITS)
    window_frames = (HOST_FORCED_FIRST_LOCK_WINDOW_FRAMES
                     if forced
                     else FIRST_LOCK_WINDOW_FRAMES)
    if first_lock_pending_hits >= required_hits:
        return commit_first_lock()
    if first_lock_pending_samples >= window_frames:
        reset_first_lock_pending()
    return False
def accept_model_candidate(candidate):
    global model_last_score
    if model_lock[1] is None:
        return accept_first_lock_candidate(candidate)
    if candidate is None:
        return False
    label, box, score, _, _, _ = candidate
    if label != model_lock[0]:
        return False
    if not observe_model_box(label, box):
        return False
    model_lock[0] = label
    model_lock[1] = box
    model_lock[5] = 0
    model_last_score = score
    return True
def smooth_tracking_box(previous, current, alpha_x100=None):
    if (previous is None or current is None or
            center_dist2(previous, current) > OUTPUT_SMOOTH_RESET_CENTER2):
        return current
    alpha = (OUTPUT_SMOOTH_ALPHA_X100 if alpha_x100 is None
             else alpha_x100)
    keep = 100 - alpha
    center_x2 = ((previous[0] * 2 + previous[2]) * keep +
                 (current[0] * 2 + current[2]) * alpha + 50) // 100
    center_y2 = ((previous[1] * 2 + previous[3]) * keep +
                 (current[1] * 2 + current[3]) * alpha + 50) // 100
    width = (previous[2] * keep + current[2] * alpha + 50) // 100
    height = (previous[3] * keep + current[3] * alpha + 50) // 100
    return box_from_center(center_x2 * 0.5, center_y2 * 0.5,
                           width, height)
def tracking_box_contact(box):
    return (box[0] + box[2] * 0.5,
            box[1] + box[3] - 0.5)
def set_color_tracking(color_id, box, coordinate_box):
    global color_track_active, color_track_box
    global color_track_coordinate_box
    global color_track_color_id, color_lost_count
    if color_track_active and color_track_color_id == color_id:
        box = smooth_tracking_box(color_track_box, box)
        coordinate_box = smooth_tracking_box(
            color_track_coordinate_box, coordinate_box)
    color_track_active = True
    color_track_box = box
    color_track_coordinate_box = coordinate_box
    color_track_color_id = color_id
    color_lost_count = 0
    return box, coordinate_box
def current_tracking_hold_result(color_id):
    if (not color_track_active or color_track_color_id != color_id or
            color_track_box is None or color_track_coordinate_box is None):
        return None
    return (color_id, color_track_box, color_track_coordinate_box, 1)
def held_color_tracking_result(color_id):
    if (color_lost_count <= 0 or
            color_lost_count > COLOR_OUTPUT_HOLD_FRAMES):
        return None
    return current_tracking_hold_result(color_id)
def model_geometry_tracking_result(color_id):
    global color_blob_box
    global color_lost_count
    if not model_track[0]:
        return None
    color_blob_box = None
    color_lost_count = 0
    reset_color_coordinate_anchor()
    output_box = output_box_from_track()
    coordinate_box = coordinate_box_from_track()
    output_box, coordinate_box = set_color_tracking(
        color_id, output_box, coordinate_box)
    return (color_id, output_box, coordinate_box, 3)
def held_model_tracking_result(color_id):
    if model_lock[5] <= 0 or model_lock[5] > MODEL_HOLD_FRAMES:
        return None
    return current_tracking_hold_result(color_id)
def maybe_collect(frame_index):
    # 每 GC_FORCE_INTERVAL 帧无条件回收一次以预防碎片，不能只在快耗尽时被动回收。
    if (frame_index % GC_CHECK_INTERVAL == 0 and
            (frame_index % GC_FORCE_INTERVAL == 0 or
             gc.mem_free() < GC_MIN_FREE)):
        gc.collect()
def process_model_only_target(img, frame_index, run_model):
    if (not model_runtime_enabled or
            (openart_mode != MODE_SEARCH and openart_mode != MODE_CARRY)):
        reset_color_adaptation_pending()
        reset_color_blob_tracking()
        return None
    observed = False
    if run_model:
        candidate = run_model_best(img)
        acquiring = model_lock[1] is None
        observed = accept_model_candidate(candidate)
        color_confirmed = False
        if (observed and not acquiring and
                not host_forced_target_active()):
            color_confirmed = update_model_guided_color(
                img, model_last_score)
        if (color_confirmed and host_color_id_received and
                model_color[0] != target_color_id):
            restore_host_hybrid_lock()
            return None
        if model_lock[1] is not None and not observed:
            model_lock[5] += 1
            if model_lock[5] > MODEL_LOST_FRAMES:
                restore_host_hybrid_lock()
                return None
        elif candidate is None and model_lock[1] is None:
            # 逐元素赋值：省掉列表字面量与切片对象两次分配
            model_lock[2] = -1
            model_lock[3] = None
            model_lock[4] = 0
            reset_color_adaptation_pending()
    if model_lock[1] is None:
        return None
    color_id = trusted_model_color_id(model_lock[0])
    if (color_id <= 0 or
            color_id_to_model_label(color_id) != model_lock[0] or
            not model_track[0]):
        return None
    if adaptive_color_thresholds[color_id - 1] is None:
        if observed:
            return model_geometry_tracking_result(color_id)
        return held_model_tracking_result(color_id)
    blob_box = track_color_in_model_roi(img, color_id, observed)
    if blob_box is None:
        if observed:
            model_result = model_geometry_tracking_result(color_id)
            if model_result is not None:
                return model_result
        held = held_color_tracking_result(color_id)
        if held is not None:
            return held
        return held_model_tracking_result(color_id)
    geometry = color_blob_geometry(blob_box, observed)
    if geometry is None:
        return None
    output_box, coordinate_box = geometry
    output_box, coordinate_box = set_color_tracking(
        color_id, output_box, coordinate_box)
    return (color_id, output_box, coordinate_box, 3 if observed else 2)
def front_scan_current_target():
    if color_track_active and color_track_box:
        return color_track_box, color_track_color_id
    return None, target_color_id
def front_scan_box_is_current(box, current_box):
    if not current_box:
        return False
    if box_iou(box, current_box) >= FRONT_SCAN_EXCLUDE_IOU:
        return True
    return center_dist2(box, current_box) <= FRONT_SCAN_EXCLUDE_CENTER2
def front_scan_roi():
    x, y, w, h = dynamic_detect_roi
    y2 = min(y + h, FRONT_SCAN_Y_MAX)
    if y2 <= y:
        return None
    return (x, y, w, y2 - y)
def front_scan_color_id(img, label, box):
    if label < 0 or label >= len(MODEL_COLOR_IDS):
        return 0
    candidates = MODEL_COLOR_IDS[label]
    if len(candidates) == 1:
        return candidates[0]
    sample = sample_box_lab_stats(img, label, box, COLOR_SAMPLE_MAX_IQR)
    if sample is None:
        return 0
    return sample_color_id_from_stats(label, sample)
def scan_front_other_color_ids(img):
    global model_infer_error_count, model_last_frame
    current_box, current_id = front_scan_current_target()
    mask = 0
    count = 0
    roi = front_scan_roi()
    if (roi is None or not model_runtime_enabled or model_net is None):
        return current_id, mask, count
    try:
        model_last_frame = frame_count
        objects = tf.detect(model_net, model_copy_frame(img))
        model_infer_error_count = 0
    except Exception as error:
        model_infer_error_count += 1
        if model_infer_error_count >= 3:
            disable_model_runtime(
                'three front-scan inference failures: ' + str(error))
        return current_id, mask, count
    if not objects:
        return current_id, mask, count
    frame_width = img.width()
    frame_height = img.height()
    for obj in objects:
        x1, y1, x2, y2, label_value, score_value = obj
        label = int(label_value)
        score = float(score_value)
        if (label < 0 or label >= len(MODEL_COLOR_IDS) or
                score < FRONT_SCAN_SCORE_MIN):
            continue
        model_x = int(float(x1) * frame_width)
        y = int(float(y1) * frame_height)
        w = int((float(x2) - float(x1)) * frame_width)
        h = int((float(y2) - float(y1)) * frame_height)
        x = frame_width - model_x - w
        if (w < MODEL_MIN_BOX_SIDE or h < MODEL_MIN_BOX_SIDE or
                w * h < MODEL_MIN_BOX_AREA):
            continue
        box = raw_model_box(x, y, w, h)
        if rect_intersection_area(box, roi) <= 0:
            continue
        if ENABLE_DYNAMIC_CUT and dynamic_cut_valid:
            if box[1] + box[3] < dynamic_cut_left_y + CUT_BLOB_DELTA:
                continue
        if (ENABLE_TENNIS_LINE_FILTER and label == 1 and
                tennis_candidate_is_yellow_line(img, box)):
            continue
        if front_scan_box_is_current(box, current_box):
            continue
        color_id = front_scan_color_id(img, label, box)
        if color_id <= 0:
            continue
        color_bit = 1 << (color_id - 1)
        if mask & color_bit:
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
def detect_return_yellow_y(img):
    try:
        blobs = img.find_blobs(
            RETURN_YELLOW_THRESHOLD, roi=RETURN_YELLOW_ROI,
            pixels_threshold=RETURN_YELLOW_MIN_PIXELS,
            area_threshold=RETURN_YELLOW_MIN_AREA, merge=True)
    except Exception:
        return None
    best_y = None
    best_bottom = -1
    best_pixels = -1
    if blobs:
        for blob in blobs:
            bottom = blob.y() + blob.h()
            if (best_y is None or bottom > best_bottom or
                    (bottom == best_bottom and blob.pixels() > best_pixels)):
                best_y = blob.cy()
                best_bottom = bottom
                best_pixels = blob.pixels()
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
def box_to_world(x, y, w, h):
    # The mesh was measured at the visible ground-contact point.
    contact_x = x + w * 0.5
    contact_y = y + h - 0.5
    world = ground_pixel_to_world(contact_x, contact_y)
    if world is None or not GROUND_CENTER_X_ON_IMAGE:
        return world
    row = int(contact_y)
    if 0 <= row < GROUND_IMAGE_H and contact_y == row + 0.5:
        center_x = center_line_world_x_for_row(row)
    else:
        center_world = ground_pixel_to_world(GROUND_IMAGE_W * 0.5, contact_y)
        center_x = None if center_world is None else center_world[0]
    if center_x is None:
        return world
    return (world[0] - center_x, world[1])
def tracking_world_point(color_id, coordinate_box):
    return box_to_world(
        coordinate_box[0], coordinate_box[1],
        coordinate_box[2], coordinate_box[3])

_tx_world_buf = bytearray(16)
_tx_world_no_target_buf = bytearray(16)
_tx_world_buf[0] = _tx_world_no_target_buf[0] = 0xAA
_tx_world_buf[1] = _tx_world_no_target_buf[1] = 0x55
def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00):
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
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[15] = (data[2] + data[3] + data[4] + data[5] + data[6] +
                data[7] + data[8] + data[9] + data[10]) & 0xFF
    uart.write(data)
def send_world_no_target(yellow_flag=False, pos_flag=0x00):
    data = _tx_world_no_target_buf
    data[9] = 0x01 if yellow_flag else 0x00
    data[10] = pos_flag & 0xFF
    data[15] = (data[9] + data[10]) & 0xFF
    uart.write(data)
def send_front_scan_target_hold(img, yellow_flag=False, pos_flag=0x00):
    if (not color_track_active or color_track_color_id <= 0 or
            color_track_box is None or color_track_coordinate_box is None):
        return False
    world_point = tracking_world_point(
        color_track_color_id, color_track_coordinate_box)
    if world_point is None:
        return False
    world_x, world_y = world_point
    send_world_data(color_track_color_id,
                    world_cm_to_mm(world_x), world_cm_to_mm(world_y),
                    color_track_box[2], yellow_flag, pos_flag)
    img.draw_rectangle(
        color_track_box, color=TARGET_BOX_COLORS[color_track_color_id - 1],
        thickness=2)
    return True
def receive_command_from_host():
    global lost_frame_count, openart_mode, carry_start_frame
    global target_color_id, host_color_id_received
    global _cmd_rx_buf, front_scan_requested
    global first_lock_reset_cycle_active
    available = uart.any()
    if available:
        data = uart.read(available)
        if data:
            _cmd_rx_buf.extend(data)
    if len(_cmd_rx_buf) > 64:
        _cmd_rx_buf = _cmd_rx_buf[-32:]
    while len(_cmd_rx_buf) >= 4:
        header = -1
        for index in range(len(_cmd_rx_buf) - 1):
            if _cmd_rx_buf[index] == 0xAA and _cmd_rx_buf[index + 1] == 0x55:
                header = index
                break
        if header < 0:
            _cmd_rx_buf = bytearray()
            return
        if header:
            _cmd_rx_buf = _cmd_rx_buf[header:]
        if len(_cmd_rx_buf) < 4:
            return
        command = _cmd_rx_buf[2]
        if command == TARGET_ANCHOR_COMMAND:
            frame_size = TARGET_ANCHOR_FRAME_SIZE
        elif command == 0x03 or command == 0x04:
            frame_size = 5
        else:
            frame_size = 4
        if len(_cmd_rx_buf) < frame_size:
            return
        if command == TARGET_ANCHOR_COMMAND:
            param = _cmd_rx_buf[3]
            x_mm = _cmd_rx_buf[4] | (_cmd_rx_buf[5] << 8)
            y_mm = _cmd_rx_buf[6] | (_cmd_rx_buf[7] << 8)
            if x_mm >= 0x8000:
                x_mm -= 0x10000
            if y_mm >= 0x8000:
                y_mm -= 0x10000
            anchor_radius_cm = _cmd_rx_buf[8]
            anchor_sequence = _cmd_rx_buf[9]
            checksum_calc = 0
            for index in range(2, frame_size - 1):
                checksum_calc = (checksum_calc + _cmd_rx_buf[index]) & 0xFF
        else:
            param = _cmd_rx_buf[3] if frame_size == 5 else 0
            checksum_calc = (command + param) & 0xFF
        checksum = _cmd_rx_buf[frame_size - 1]
        if checksum != checksum_calc:
            _cmd_rx_buf = _cmd_rx_buf[2:]
            continue
        _cmd_rx_buf = _cmd_rx_buf[frame_size:]
        if command == TARGET_ANCHOR_COMMAND:
            apply_target_anchor_command(
                param, x_mm, y_mm, anchor_radius_cm, anchor_sequence)
        elif command == 0x03 and 1 <= param <= len(all_color_thresholds):
            if not color_id_available_for_search(param):
                reset_target_tracking_state()
            else:
                same_target = host_color_id_received and target_color_id == param
                if (target_anchor_active and
                        (not ENABLE_TARGET_ANCHOR_LOCK or
                         target_anchor_color_id != param)):
                    clear_target_anchor()
                target_color_id = param
                host_color_id_received = True
                lost_frame_count = 0
                if not same_target:
                    apply_host_hybrid_color(param)
        elif command == 0x01:
            first_lock_reset_cycle_active = False
            reset_first_lock_pending()
            front_scan_requested = False
            reset_front_scan_state()
            reset_return_yellow_state()
            openart_mode = MODE_CARRY
            carry_start_frame = frame_count
            reset_yellow_state()
        elif command == CMD_CLEAR_COMPLETED:
            clear_completed_carry_state()
            openart_mode = MODE_SEARCH
            front_scan_requested = False
            reset_front_scan_state()
            reset_target_tracking_state()
            reset_yellow_state()
            reset_return_yellow_state()
        elif command == 0x06:
            openart_mode = MODE_SEARCH
            reset_yellow_state()
            reset_return_yellow_state()
            reset_front_scan_state()
            front_scan_requested = True
        elif command == 0x07:
            first_lock_reset_cycle_active = False
            reset_first_lock_pending()
            clear_target_anchor()
            openart_mode = MODE_RETURN
            front_scan_requested = False
            reset_front_scan_state()
            reset_yellow_state()
            reset_return_yellow_state()
        elif command == 0x02:
            if not first_lock_reset_cycle_active:
                first_lock_reset_cycle_active = True
                openart_mode = MODE_SEARCH
                reset_target_tracking_state()
                reset_yellow_state()
                reset_return_yellow_state()
        elif command == 0x00:
            first_lock_reset_cycle_active = False
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
            reset_return_yellow_state()
        return
def pick_yellow_blob(blobs):
    best = None
    best_pixels = -1
    # 循环不变量：遍历过程中模式与目标框不会变化，提前取出
    track_box = (color_track_box
                 if openart_mode == MODE_CARRY and color_track_active
                 else None)
    if track_box:
        tx, ty, tw, th = track_box
    for blob in blobs:
        if track_box:
            bx = blob.x()
            by = blob.y()
            bw = blob.w()
            bh = blob.h()
            overlap_w = min(tx + tw, bx + bw) - max(tx, bx)
            overlap_h = min(ty + th, by + bh) - max(ty, by)
            if (overlap_w > 0 and overlap_h > 0 and
                    overlap_w * overlap_h * 100 >=
                    bw * bh * YELLOW_TARGET_OVERLAP_PERCENT):
                continue
        pixels = blob.pixels()
        if best is None or pixels > best_pixels:
            best = blob
            best_pixels = pixels
    return best
def draw_carry_yellow_line(img):
    if (not ENABLE_YELLOW_DRAW or
            (openart_mode != MODE_SEARCH and openart_mode != MODE_CARRY) or
            not yellow_detected):
        return
    # 用四个标量代替 points 列表：每帧少一个列表和最多四个坐标元组。
    # px<0 表示该点尚未取到（合法 x 恒在 0..319）。
    px0 = -1
    py0 = 0
    px1 = -1
    py1 = 0
    for x in (0, 319):
        y = yellow_line_k * x + yellow_line_b
        if 0 <= y <= 239:
            if px0 < 0:
                px0 = x
                py0 = int(y)
            elif px1 < 0:
                px1 = x
                py1 = int(y)
    # 左右边界已凑满两点时，上下边界交点原本也只会落在不被使用的下标上
    if px1 < 0 and abs(yellow_line_k) > 0.0001:
        for y in (0, 239):
            x = (y - yellow_line_b) / yellow_line_k
            if 0 <= x <= 319:
                xi = int(x)
                if px0 < 0:
                    px0 = xi
                    py0 = y
                elif px1 < 0 and (xi != px0 or y != py0):
                    px1 = xi
                    py1 = y
    if px1 >= 0:
        img.draw_line(px0, py0, px1, py1,
                      color=(255, 255, 0), thickness=2)
def current_pos_flag(frame_index):
    global yellow_lost_count, yellow_seen_in_carry
    global yellow_bottom_reached_in_carry, yellow_carry_confirm_count
    global openart_mode
    if openart_mode == MODE_CARRY:
        if carry_start_frame >= 0 and frame_index - carry_start_frame < YELLOW_CARRY_IGNORE_FRAMES:
            yellow_lost_count = 0
            return POS_NO_BOUNDARY
        if yellow_detected:
            if not yellow_seen_in_carry:
                yellow_carry_confirm_count += 1
                yellow_seen_in_carry = yellow_carry_confirm_count >= YELLOW_CARRY_CONFIRM_FRAMES
            yellow_lost_count = 0
            if yellow_seen_in_carry and yellow_bottom_visible:
                yellow_bottom_reached_in_carry = True
            return POS_NO_BOUNDARY
        if not yellow_seen_in_carry:
            yellow_carry_confirm_count = 0
        if yellow_bottom_reached_in_carry:
            yellow_lost_count += 1
            if yellow_lost_count >= YELLOW_LOST_THRESHOLD:
                mark_color_completed(active_selected_color_id())
                openart_mode = MODE_WAIT_TURN
                return POS_CROSSED
        return POS_NO_BOUNDARY
    if openart_mode == MODE_WAIT_TURN:
        return POS_CROSSED
    if openart_mode == MODE_SEARCH:
        yellow_lost_count = 0
        return POS_RIGHT_SIDE if yellow_detected else POS_NO_BOUNDARY
    return POS_NO_BOUNDARY
def update_yellow_detection(img):
    global yellow_tracking, yellow_detected, yellow_bottom_visible
    global yellow_line_k, yellow_line_b
    minimum = YELLOW_KEEP_PIXELS if yellow_tracking else YELLOW_ENTER_PIXELS
    top = pick_yellow_blob(img.find_blobs(
        yellow_threshold, roi=YELLOW_ROI_TOP, pixels_threshold=minimum,
        area_threshold=20, merge=True))
    bottom = pick_yellow_blob(img.find_blobs(
        yellow_threshold, roi=YELLOW_ROI_BOTTOM, pixels_threshold=minimum,
        area_threshold=20, merge=True))
    valid = top is not None and bottom is not None
    if valid:
        dx = bottom.cx() - top.cx()
        dy = bottom.cy() - top.cy()
        valid = (abs(dx) >= YELLOW_MIN_FIT_DX and
                 abs(dy) * 100 <= abs(dx) * YELLOW_MAX_FIT_SLOPE_X100)
    yellow_detected = valid
    yellow_bottom_visible = False
    if valid:
        yellow_tracking = True
        slope = dy / dx
        intercept = top.cy() - slope * top.cx()
        yellow_line_k = slope
        yellow_line_b = intercept
        yellow_bottom_visible = (intercept >= YELLOW_BOTTOM_Y or
                                 slope * 319 + intercept >= YELLOW_BOTTOM_Y)
    elif openart_mode == MODE_SEARCH or not yellow_seen_in_carry:
        yellow_tracking = False
        yellow_line_k = 0.0
        yellow_line_b = 0.0
frame_count = 0
init_model_runtime()
while True:
    frame_count += 1
    receive_command_from_host()
    img = snapshot_frame()
    if openart_mode == MODE_RETURN:
        process_return_yellow(img)
        maybe_collect(frame_count)
        feed_watchdog()
        continue
    update_dynamic_cut(img, frame_count)
    if openart_mode == MODE_CARRY:
        update_yellow_detection(img)
    pos_flag = current_pos_flag(frame_count)
    draw_carry_yellow_line(img)
    if process_front_scan_request(img):
        send_front_scan_target_hold(img, yellow_detected, pos_flag)
        maybe_collect(frame_count)
        feed_watchdog()
        continue
    result = process_model_only_target(
        img, frame_count, should_run_model(frame_count))
    has_target = result is not None
    if has_target:
        send_color_id, output_box, coordinate_box, source = result
        w = output_box[2]
    if has_target:
        world_point = tracking_world_point(send_color_id, coordinate_box)
        if world_point is None:
            has_target = False
    if has_target:
        lost_frame_count = 0
        world_x, world_y = world_point
        wx_mm = world_cm_to_mm(world_x)
        wy_mm = world_cm_to_mm(world_y)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_detected, pos_flag)
        img.draw_rectangle(output_box,
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
        send_world_no_target(yellow_detected, pos_flag)
    maybe_collect(frame_count)
    feed_watchdog()
