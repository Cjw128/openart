# ======================================================================
# OpenART shared front-camera vision runtime.
# ======================================================================


import sensor, image, time, math
from machine import UART
from openart_calibration import run_calibration_mode
from openart_config import ACTIVE_CONFIG
from openart_detectors import OpenArtDetectors
from openart_math import calc_homography, pixel_to_world, world_to_pixel
from openart_trackers import ReturnBeaconTracker, TargetTracker, YellowLineTracker
from openart_uart import UartProtocol, calculate_checksum
from yellow_crossline_ipm import create_crossline_ipm

# ======================================================================
# Mode selection
# ======================================================================
CALIBRATION_MODE = ACTIVE_CONFIG["calibration_mode"]
BIRDVIEW_DEBUG = ACTIVE_CONFIG["birdview_debug"]
IS_SLAVE_CAR = ACTIVE_CONFIG["is_slave_car"]
SLAVE_MODE = IS_SLAVE_CAR  # True: color is controlled by host 0x03 command

# ======================================================================
# Hardware initialization
# ======================================================================

# Camera initialization
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
sensor.set_framerate(60)

# White balance configuration
# True = fixed gains for competition, False = auto-converge then lock for tuning
WB_FIXED = ACTIVE_CONFIG["wb_fixed"]
# Fixed white balance gains (R_db, G_db, B_db)
# Fill these after running wb_calibrate.py under competition lighting.
WB_GAINS = ACTIVE_CONFIG["wb_gains"]

if WB_FIXED:
    sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
    sensor.skip_frames(time=500)
else:
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)

# Fixed exposure
sensor.set_auto_exposure(False, exposure_us=ACTIVE_CONFIG["fixed_exposure_us"])
sensor.set_auto_gain(False, gain_db=0)

# UART initialization (UART12, 115200 baud)
if IS_SLAVE_CAR:
    uart = UART(2, baudrate=115200)
else:
    uart = UART(12, baudrate=115200)

# FPS timer
clock = time.clock()

# ======================================================================
# LAB color thresholds - dynamic multi-color detection
# ======================================================================
# LAB format: (L_min, L_max, A_min, A_max, B_min, B_max)
# L: brightness (0-100)
# A: red-green axis (positive=red, negative=green)
# B: yellow-blue axis (positive=yellow, negative=blue)

# Supported color thresholds
all_color_thresholds = ACTIVE_CONFIG["color_thresholds"]

COLOR_LOST_FRAMES = 5
COLOR_TRACK_MARGIN = 45
COLOR_MIN_PIXELS = 100
COLOR_MIN_AREA = 100

# White bear is detected only by the TFLite model, not by LAB color threshold.
WHITE_BEAR_COLOR_ID = 5
MODEL_ENABLED = ACTIVE_CONFIG["model_enabled"]
MODEL_PATH = ACTIVE_CONFIG["model_path"]
MODEL_INPUT_SCALE = 0.75
MODEL_SCORE_THRESHOLD = 0.40
MODEL_FALLBACK_SCORE_THRESHOLD = 0.25
MODEL_LABEL_BEAR = 0
# Blue floor threshold, example values that must be tuned on field.
# Focus on the B channel; blue is usually below -20.
# BLUE_GROUND_THRESHOLD = (10, 50, -20, 50, -77, -25)

# Target-lost counters
MAX_LOST_FRAMES = 30                    # Maximum lost frames, about 0.5s at 60 FPS
STABLE_FRAMES_REQUIRED = 5              # Frames required before locking a color

# ======================================================================
# Recognition parameters
# ======================================================================
MIN_PIXELS = 30        # Blob pixel threshold
MIN_AREA = 80          # Minimum bounding-box area
MERGE_DISTANCE = False      # Merge threshold matches at the same position
DETECT_Y_MIN = 8           # Ignore image rows above this Y value
DETECT_ROI = (0, DETECT_Y_MIN, 320, 240 - DETECT_Y_MIN)

# ======================================================================
# Dynamic cut line (based on blue-ground strips on left/right)
# ======================================================================
ENABLE_DYNAMIC_CUT = True
BLUE_GROUND_THRESHOLD = [(32, 57, -52, 76, -108, -28)]
CUT_LEFT_X = 10
CUT_RIGHT_X = 310
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

# Local ROI tracking (lock-target fast path)
ENABLE_LOCAL_TRACK_ROI = True
TRACK_MARGIN_X_RATIO = 0.50
TRACK_MARGIN_Y_RATIO = 0.50
TRACK_MARGIN_X_MIN = 12
TRACK_MARGIN_Y_MIN = 12
TRACK_MARGIN_X_MAX = 90
TRACK_MARGIN_Y_MAX = 70
TRACK_MIN_ROI_W = 16
TRACK_MIN_ROI_H = 16
TRACK_MAX_JUMP_PX = 90
TRACK_AREA_CHANGE_MAX_PERCENT = 60

# Aspect-ratio filter to reduce false positives
ENABLE_ASPECT_RATIO_FILTER = True   # Enable aspect-ratio filtering
MIN_ASPECT_RATIO = 0.3            # Minimum width/height ratio
MAX_ASPECT_RATIO = 2.3             # Maximum width/height ratio

MIN_ROUNDNESS = 0.4                 # Roundness threshold; ball high, line very low
MIN_DENSITY = 0.6

last_target_cx = -1
last_target_cy = -1
_cmd_rx_buf = bytearray()
crossline_angle_enabled = False
crossline_angle_result = None
# ======================================================================
# Yellow line detection parameters
# ======================================================================
yellow_threshold = ACTIVE_CONFIG["yellow_threshold"]
YELLOW_ROI_LEFT = ACTIVE_CONFIG["yellow_roi_left"]
YELLOW_ROI_RIGHT = ACTIVE_CONFIG["yellow_roi_right"]
YELLOW_DETECT_INTERVAL = 5              # Detect yellow line every N frames
YELLOW_ENTER_PIXELS = 10                # Pixel threshold for first yellow-line hit
YELLOW_KEEP_PIXELS = 3                  # Lower hold threshold after line is seen
# ======================================================================
# Orange obstacle detection
# ======================================================================
obstacle_threshold = [(56, 96, 16, 127, 9, 127)]
OBSTACLE_ROI = (0, 80, 320, 160)
OBSTACLE_PATH_X_MIN = 110
OBSTACLE_PATH_X_MAX = 210
OBSTACLE_MIN_PIXELS = 80
OBSTACLE_MIN_AREA = 80

OBSTACLE_NONE = 0x00
OBSTACLE_MOVE_RIGHT = 0x01
OBSTACLE_MOVE_LEFT = 0x02
OBSTACLE_BLOCKED = 0x03
OBSTACLE_TARGET_OVERLAP_PIXELS = 200

YELLOW_LOST_THRESHOLD = 5   # Lost frames required before crossed-line decision
YELLOW_RECENT_DETECTIONS = 5 # Recent yellow-line latch window for carry occlusion

# State machine
MODE_SEARCH = 0
MODE_CARRY = 1
MODE_WAIT_TURN = 2
MODE_RETURN = 3

openart_mode = MODE_SEARCH   # 0=search, 1=carry, 2=wait for turn, 3=return

# Position flags
POS_NO_BOUNDARY = 0x00
POS_RIGHT_SIDE  = 0x01
POS_CROSSED     = 0x02

# Return-to-depot beacon detection. Uses the same IPM and UART packet as normal targets.
RETURN_BEACON_ID = 0x06
BEACON_THRESHOLD = [(79, 95, 5, 65, -54, 73)]
BEACON_DETECT_Y_MIN = 20
BEACON_DETECT_ROI = (0, BEACON_DETECT_Y_MIN, 320, 240 - BEACON_DETECT_Y_MIN)
BEACON_MIN_PIXELS = 100
BEACON_MIN_AREA = 100
BEACON_MERGE_BLOBS = True
BEACON_MIN_ASPECT_RATIO = 0.10
BEACON_MAX_ASPECT_RATIO = 3.50
BEACON_MIN_DENSITY = 0.20
BEACON_TRACK_MAX_LOST = 10

yellow_tracker_config = {
    "threshold": yellow_threshold,
    "roi_left": YELLOW_ROI_LEFT,
    "roi_right": YELLOW_ROI_RIGHT,
    "detect_interval": YELLOW_DETECT_INTERVAL,
    "enter_pixels": YELLOW_ENTER_PIXELS,
    "keep_pixels": YELLOW_KEEP_PIXELS,
    "recent_detections": YELLOW_RECENT_DETECTIONS,
    "lost_threshold": YELLOW_LOST_THRESHOLD,
    "mode_search": MODE_SEARCH,
    "mode_carry": MODE_CARRY,
    "mode_wait_turn": MODE_WAIT_TURN,
    "pos_no_boundary": POS_NO_BOUNDARY,
    "pos_right_side": POS_RIGHT_SIDE,
    "pos_crossed": POS_CROSSED,
}

yellow_tracker = YellowLineTracker(yellow_tracker_config, pixel_to_world)
beacon_tracker = ReturnBeaconTracker()
target_tracker = TargetTracker(all_color_thresholds, WHITE_BEAR_COLOR_ID,
                               COLOR_LOST_FRAMES, MAX_LOST_FRAMES)

def code_to_color_id(code):
    """将find_blobs返回的位掩码code映射为颜色ID(1/2/3...)。"""
    if code <= 0:
        return 0
    for i in range(len(all_color_thresholds)):
        if code & (1 << i):
            return i + 1
    return 0

def reset_target_tracking_state():
    """清空上一轮搬运留下的目标锁定状态，下一帧从全局重新找场地中央目标。"""
    global last_target_cx, last_target_cy

    last_target_cx = -1
    last_target_cy = -1
    target_tracker.reset()

def reset_yellow_state():
    """清空黄线状态，避免新一轮任务继承上一轮的边界/滞回。"""
    yellow_tracker.reset()

def reset_beacon_state():
    beacon_tracker.reset()

# ======================================================================
# Homography transform for inverse perspective mapping
# ======================================================================

def update_dynamic_cut(img, frame_count):
    detectors.update_dynamic_cut(img, frame_count)

def find_color_target(img, last_box):
    return detectors.find_color_target(img, last_box)

def find_white_bear_model_target(img, last_box):
    return detectors.find_white_bear_model_target(img, last_box)

def find_beacon_blob(img, last_box):
    return detectors.find_beacon_blob(img, last_box)

def box_to_world(x, y, w, h):
    return detectors.box_to_world(x, y, w, h)

# ======================================================================
# IPM calibration data; tune for the actual camera mount.
# ======================================================================
CALIB_PIXEL = ACTIVE_CONFIG["calib_pixel"]
CALIB_WORLD = ACTIVE_CONFIG["calib_world"]

# Compute homography matrix
H_pix2world = calc_homography(CALIB_PIXEL, CALIB_WORLD)
H_world2pix = calc_homography(CALIB_WORLD, CALIB_PIXEL)

if H_pix2world:
    print("[OK] 前视逆透视矩阵计算成功")
else:
    print("[ERROR] 前视逆透视矩阵计算失败!")

crossline_ipm = create_crossline_ipm(uart_enabled=False)
crossline_ipm.set_debug_draw(False)
crossline_ipm.H_pix2world = H_pix2world
crossline_ipm.yellow_threshold = yellow_threshold

# ======================================================================
# Bird-view configuration for debug mode
# ======================================================================
BIRD_W = 80
BIRD_H = 80
X_MIN = -15.0    # Left 15 cm
X_MAX = 15.0     # Right 15 cm
Y_MIN = 5.0      # Near 5 cm
Y_MAX = 35.0     # Far 35 cm
SX = (X_MAX - X_MIN) / BIRD_W
SY = (Y_MAX - Y_MIN) / BIRD_H

# ======================================================================
# Distance estimation parameters based on target width
# ======================================================================
# Distance formula: distance = (real_width * focal_length) / pixel_width

# Real target widths in mm
TARGET_REAL_WIDTH = [
    70.0,   # Color 1: bag, 7 cm
    70.0,   # Color 2: bag, 7 cm
    67.0,   # Color 3: tennis ball, about 6.7 cm diameter
    120.0,  # Color 4: brown teddy bear, about 12 cm body width
    120.0   # Color 5: white teddy bear, about 12 cm body width
]

# Camera calibration parameter from field testing
FOCAL_LENGTH = 167.5  # Focal length parameter; tune after calibration

# Valid distance range in mm
MIN_DETECT_DISTANCE = 50    # Minimum detection distance
MAX_DETECT_DISTANCE = 2000  # Maximum detection distance

detector_config = {
    "color_thresholds": all_color_thresholds,
    "color_track_margin": COLOR_TRACK_MARGIN,
    "color_min_pixels": COLOR_MIN_PIXELS,
    "color_min_area": COLOR_MIN_AREA,
    "white_bear_color_id": WHITE_BEAR_COLOR_ID,
    "model_enabled": MODEL_ENABLED,
    "model_path": MODEL_PATH,
    "model_input_scale": MODEL_INPUT_SCALE,
    "model_fallback_score_threshold": MODEL_FALLBACK_SCORE_THRESHOLD,
    "model_label_bear": MODEL_LABEL_BEAR,
    "detect_y_min": DETECT_Y_MIN,
    "detect_roi": DETECT_ROI,
    "enable_dynamic_cut": ENABLE_DYNAMIC_CUT,
    "blue_ground_threshold": BLUE_GROUND_THRESHOLD,
    "cut_left_x": CUT_LEFT_X,
    "cut_right_x": CUT_RIGHT_X,
    "cut_strip_half_w": CUT_STRIP_HALF_W,
    "cut_scan_y_min": CUT_SCAN_Y_MIN,
    "cut_scan_y_max": CUT_SCAN_Y_MAX,
    "cut_update_interval": CUT_UPDATE_INTERVAL,
    "cut_min_pixels": CUT_MIN_PIXELS,
    "cut_min_area": CUT_MIN_AREA,
    "cut_y_margin": CUT_Y_MARGIN,
    "cut_ema_alpha": CUT_EMA_ALPHA,
    "cut_max_miss": CUT_MAX_MISS,
    "cut_blob_delta": CUT_BLOB_DELTA,
    "enable_local_track_roi": ENABLE_LOCAL_TRACK_ROI,
    "track_margin_x_ratio": TRACK_MARGIN_X_RATIO,
    "track_margin_y_ratio": TRACK_MARGIN_Y_RATIO,
    "track_margin_x_min": TRACK_MARGIN_X_MIN,
    "track_margin_y_min": TRACK_MARGIN_Y_MIN,
    "track_margin_x_max": TRACK_MARGIN_X_MAX,
    "track_margin_y_max": TRACK_MARGIN_Y_MAX,
    "track_min_roi_w": TRACK_MIN_ROI_W,
    "track_min_roi_h": TRACK_MIN_ROI_H,
    "obstacle_threshold": obstacle_threshold,
    "obstacle_roi": OBSTACLE_ROI,
    "obstacle_path_x_min": OBSTACLE_PATH_X_MIN,
    "obstacle_path_x_max": OBSTACLE_PATH_X_MAX,
    "obstacle_min_pixels": OBSTACLE_MIN_PIXELS,
    "obstacle_min_area": OBSTACLE_MIN_AREA,
    "obstacle_none": OBSTACLE_NONE,
    "obstacle_move_right": OBSTACLE_MOVE_RIGHT,
    "obstacle_move_left": OBSTACLE_MOVE_LEFT,
    "obstacle_blocked": OBSTACLE_BLOCKED,
    "obstacle_target_overlap_pixels": OBSTACLE_TARGET_OVERLAP_PIXELS,
    "beacon_threshold": BEACON_THRESHOLD,
    "beacon_detect_y_min": BEACON_DETECT_Y_MIN,
    "beacon_detect_roi": BEACON_DETECT_ROI,
    "beacon_min_pixels": BEACON_MIN_PIXELS,
    "beacon_min_area": BEACON_MIN_AREA,
    "beacon_merge_blobs": BEACON_MERGE_BLOBS,
    "beacon_min_aspect_ratio": BEACON_MIN_ASPECT_RATIO,
    "beacon_max_aspect_ratio": BEACON_MAX_ASPECT_RATIO,
    "beacon_min_density": BEACON_MIN_DENSITY,
    "target_real_width": TARGET_REAL_WIDTH,
    "focal_length": FOCAL_LENGTH,
    "min_detect_distance": MIN_DETECT_DISTANCE,
    "max_detect_distance": MAX_DETECT_DISTANCE,
}
detectors = OpenArtDetectors(detector_config, target_tracker, H_pix2world)
detectors.load_white_bear_model()

def calculate_distance(pixel_width, color_id=1):
    """根据目标像素宽度和颜色ID计算距离"""
    return detectors.calculate_distance(pixel_width, color_id)

def detect_obstacle(img):
    return detectors.detect_obstacle(img)

def box_hits_obstacle(box, obstacle_blobs):
    return detectors.box_hits_obstacle(box, obstacle_blobs)

# ======================================================================
# UART protocol
# ======================================================================
# Packet format, 14 bytes:
# [0-1]   Header: 0xAA 0x55
# [2]     Color ID, 0=no target, 1=light blue, 2=red, 3+=reserved
# [3-4]   Center X, low byte then high byte
# [5-6]   Center Y, low byte then high byte
# [7-8]   Width, low byte then high byte
# [9-10]  Height, low byte then high byte
# [11-12] Distance in mm, low byte then high byte
# [13]    Checksum, sum of data bytes masked to 0xFF

protocol = UartProtocol(uart)

def send_target_data(color_id, cx, cy, w, h, distance):
    """
    发送沙包坐标数据到RT1021主控

    Args:
        color_id: 颜色ID (1=颜色1, 2=颜色2, ...)
        cx: 中心X坐标 (0-320)
        cy: 中心Y坐标 (0-240)
        w: 宽度
        h: 高度
        distance: 距离 (mm)
    """
    data = protocol.build_target_packet(color_id, cx, cy, w, h, distance)

    # Debug print of transmitted packet, throttled to once per second
    global last_print_time
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_time) >= 1000:
        print("TX: [", end="")
        for i, b in enumerate(data):
            if i > 0:
                print(", ", end="")
            print("0x%02x" % b, end="")
        print("]")
        last_print_time = now

    uart.write(data)


def send_no_target():
    """发送无目标数据"""
    protocol.send_no_target()

# ======================================================================
# UART protocol, world-coordinate packet
# ======================================================================
# [0-1]  Header: 0xAA 0x55
# [2]    Color ID, 0=no target, 1=light blue, 2=red, 3=ball, 4=brown bear, 5=white bear
# [3-4]  World X, int16 little-endian, mm*10, left positive
# [5-6]  World Y, int16 little-endian, mm*10, forward positive
# [7-8]  Pixel width, uint16 little-endian
# [9]    Checksum, sum(data[2:9]) & 0xFF

def send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                    angle_flag=0x00, angle_cdeg=0):
    # World packet v2, 16 bytes:
    # [12] angle_flag: bit0=angle enabled, bit1=angle valid
    # [13-14] crossline angle, int16 little-endian, degree * 100
    # [15] checksum = sum(data[2:15]) & 0xFF
    """发送世界坐标数据包 (12字节, 含黄线信息)
    [0-1]  帧头 0xAA 0x55
    [2]    颜色ID
    [3-4]  世界X (mm, int16, 小端序)
    [5-6]  世界Y (mm, int16, 小端序)
    [7-8]  像素宽度 (uint16)
    [9]    黄线标志 0x00/0x01
    [10]   位置关系 0x00/0x01/0x02
    [11]   校验和 (data[2:11])
    """
    protocol.send_world_data(color_id, wx_mm, wy_mm, pw, yellow_flag, pos_flag,
                             obstacle_flag, angle_flag, angle_cdeg)

def send_world_no_target(yellow_flag=False, pos_flag=0x00, obstacle_flag=0x00,
                         angle_flag=0x00, angle_cdeg=0):
    # Same 16-byte packet layout as send_world_data(), with color_id/position fields zero.
    """发送无目标数据包 (12字节)"""
    protocol.send_world_no_target(yellow_flag, pos_flag, obstacle_flag, angle_flag, angle_cdeg)

def process_return_beacon_frame(img):
    global last_print_time

    obstacle_flag, obstacle_blobs = detect_obstacle(img)
    angle_flag, angle_cdeg = get_crossline_angle_fields()

    if H_pix2world is None:
        send_world_no_target(False, POS_NO_BOUNDARY, obstacle_flag, angle_flag, angle_cdeg)
        return

    blob = find_beacon_blob(img, beacon_tracker.last_box)
    if blob is not None:
        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()

        beacon_tracker.last_box = (x, y, w, h)
        beacon_tracker.lost_frames = 0

        world_x, world_y = box_to_world(x, y, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        send_world_data(RETURN_BEACON_ID, wx_mm, wy_mm, w, False, POS_NO_BOUNDARY,
                        obstacle_flag, angle_flag, angle_cdeg)

        img.draw_rectangle(blob.rect(), color=(0, 255, 255), thickness=2)
        img.draw_cross(cx, cy, color=(255, 0, 0), size=8, thickness=2)
        img.draw_string(x, max(0, y - 16),
                        "return ({:.1f},{:.1f})cm".format(world_x, world_y),
                        color=(0, 255, 255), scale=1)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 300:
            print('[{}] return beacon w=({:.1f},{:.1f})cm box=({},{},{},{}) obs={} fps={:.1f}'.format(
                frame_count, world_x, world_y, x, y, w, h, obstacle_flag, clock.fps()))
            last_print_time = now
    else:
        beacon_tracker.lost_frames += 1
        if beacon_tracker.lost_frames > BEACON_TRACK_MAX_LOST:
            beacon_tracker.last_box = None
        send_world_no_target(False, POS_NO_BOUNDARY, obstacle_flag, angle_flag, angle_cdeg)

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 500:
            print('[{}] return beacon none obs={} fps={:.1f}'.format(
                frame_count, obstacle_flag, clock.fps()))
            last_print_time = now

def receive_command_from_host():
    """接收RT1021主机命令"""
    global openart_mode
    global _cmd_rx_buf, crossline_angle_enabled, crossline_angle_result

    if uart.any():
        chunk = uart.read(uart.any())
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
            return (0, 0)
        if idx > 0:
            _cmd_rx_buf = _cmd_rx_buf[idx:]
        if len(_cmd_rx_buf) < 4:
            return (0, 0)

        command = _cmd_rx_buf[2]
        if command == 0x03 or command == 0x04:
            if len(_cmd_rx_buf) < 5:
                return (0, 0)
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
                target_tracker.set_target_color(param)
                print(">>> Color lock command: ID={} <<<".format(param))
        elif command == 0x01:  # Enter carry mode
            openart_mode = MODE_CARRY
            yellow_tracker.enter_carry_mode()
            reset_beacon_state()
            print(">>> Enter carry mode <<<")
        elif command == 0x04:  # SET_CROSSLINE_ANGLE_ENABLE
            crossline_angle_enabled = (param == 1)
            if not crossline_angle_enabled:
                crossline_angle_result = None
            print(">>> Crossline angle {} <<<".format("ON" if crossline_angle_enabled else "OFF"))
        elif command == 0x05:  # ENTER_RETURN_MODE
            openart_mode = MODE_RETURN
            reset_target_tracking_state()
            reset_yellow_state()
            reset_beacon_state()
            crossline_angle_enabled = False
            crossline_angle_result = None
            print(">>> Enter return mode <<<")
        elif command == 0x00 or command == 0x02:  # Reset to search mode or turn completed
            openart_mode = MODE_SEARCH
            reset_target_tracking_state()
            reset_yellow_state()
            reset_beacon_state()
            crossline_angle_enabled = False
            crossline_angle_result = None
            print(">>> Reset to search mode <<<")
        return (command, param)
    return (0, 0)

def current_pos_flag():
    global openart_mode
    pos_flag, openart_mode = yellow_tracker.position_flag(openart_mode)
    return pos_flag

def update_yellow_detection(img, frame_count):
    yellow_tracker.update(img, frame_count, openart_mode, H_pix2world)

def get_crossline_angle_fields():
    if not crossline_angle_enabled or crossline_angle_result is None:
        return (0x00, 0)

    flag = 0x01
    if crossline_angle_result["valid"]:
        flag |= 0x02
    return (flag, crossline_angle_result["angle_cdeg"])

def find_best_target(img, obstacle_blobs):
    best = None
    source = 'color'
    found = None
    last_box = target_tracker.track_box if target_tracker.track_active else None

    if MODEL_ENABLED and target_tracker.target_color_id == WHITE_BEAR_COLOR_ID:
        model_found = find_white_bear_model_target(img, last_box)
        if model_found:
            send_color_id, x1, y1, w, h, model_score = model_found
            if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
                best = (send_color_id, x1, y1, w, h)
                target_tracker.mark_found(send_color_id, (x1, y1, w, h))
                source = 'model'
                found = True
    else:
        found = find_color_target(img, last_box)

    if found and (not MODEL_ENABLED or target_tracker.target_color_id != WHITE_BEAR_COLOR_ID):
        send_color_id, blob = found
        x1 = blob.x()
        y1 = blob.y()
        w = blob.w()
        h = blob.h()
        if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
            best = (send_color_id, x1, y1, w, h)
            target_tracker.mark_found(send_color_id, (x1, y1, w, h))
        else:
            found = None

    if MODEL_ENABLED and not found and target_tracker.target_color_id == 0:
        model_found = find_white_bear_model_target(img, last_box)
        if model_found:
            send_color_id, x1, y1, w, h, model_score = model_found
            if not box_hits_obstacle((x1, y1, w, h), obstacle_blobs):
                best = (send_color_id, x1, y1, w, h)
                target_tracker.mark_found(send_color_id, (x1, y1, w, h))
                source = 'model'
                found = True

    if not found:
        held = target_tracker.hold_last_box()
        if held:
            send_color_id, x1, y1, w, h = held
            best = (send_color_id, x1, y1, w, h)
            source = 'color_hold'

    return (best, source)

# ======================================================================
# Main loop
# ======================================================================
frame_count = 0
detect_count = 0
last_print_time = time.ticks_ms()

print("=" * 50)
print(ACTIVE_CONFIG["program_title"])
print("=" * 50)
print("分辨率: 320x240 (QVGA)")
print("帧率: 60 FPS")
print("串口: UART{}, 115200bps".format(2 if IS_SLAVE_CAR else 12))
print("颜色模式: 初始多颜色检测 -> 锁定单颜色跟踪")
print("支持颜色: {} 种".format(len(all_color_thresholds)))
print("颜色阈值:", all_color_thresholds)
if ENABLE_ASPECT_RATIO_FILTER:
    print("长宽比过滤: 启用 ({:.1f} ~ {:.1f})".format(MIN_ASPECT_RATIO, MAX_ASPECT_RATIO))
else:
    print("长宽比过滤: 关闭")
print("-" * 50)
print("黄线检测参数:")
print("  阈值(LAB)  : {}".format(yellow_threshold))
print("  左侧ROI    : {}".format(YELLOW_ROI_LEFT))
print("  右侧ROI    : {}".format(YELLOW_ROI_RIGHT))
print("  检测间隔   : 每{}帧".format(YELLOW_DETECT_INTERVAL))
print("  进入像素   : {}".format(YELLOW_ENTER_PIXELS))
print("  保持像素   : {}".format(YELLOW_KEEP_PIXELS))
print("  丢失阈值   : 连续{}帧判定过线".format(YELLOW_LOST_THRESHOLD))
print("主机命令: 0x00=重置/寻找, 0x01=搬运, 0x02=右转完成, 0x03=锁色, 0x04=黄线角度开关(param 1/0), 0x05=回库")
print("回传协议: 16字节, [12]=角度标志, [13-14]=黄线偏移角度*100(int16 LE), [15]=checksum")
print("=" * 50)
print("开始识别...")
print()

# ======================================================================
# Calibration mode, enabled when CALIBRATION_MODE is True
# ======================================================================
if CALIBRATION_MODE:
    run_calibration_mode(sensor, clock, all_color_thresholds, DETECT_ROI, CALIB_WORLD)

# ======================================================================
# Bird-view frame buffer for debug mode
# ======================================================================
if BIRDVIEW_DEBUG and not CALIBRATION_MODE:
    bird = sensor.alloc_extra_fb(BIRD_W, BIRD_H, sensor.RGB565)

while True:
    # FPS accounting
    clock.tick()
    frame_count += 1

    # Receive host command
    cmd, param = receive_command_from_host()

    # Capture image and apply lens correction
    img = sensor.snapshot().lens_corr(2)
    world_x = 0.0
    world_y = 0.0

    if openart_mode == MODE_RETURN:
        process_return_beacon_frame(img)
        continue

    if crossline_angle_enabled:
        crossline_angle_result = crossline_ipm.process_frame(img)

    # ===== Dynamic cut update =====
    update_dynamic_cut(img, frame_count)
    obstacle_flag, obstacle_blobs = detect_obstacle(img)
    update_yellow_detection(img, frame_count)

    # ===== Color blob detection / tracking =====
    best, source = find_best_target(img, obstacle_blobs)

    pos_flag = current_pos_flag()
    angle_flag, angle_cdeg = get_crossline_angle_fields()

    if best:
        target_tracker.lost_frames = 0
        target_tracker.stable_frames += 1
        send_color_id, x1, y1, w, h = best

        if (target_tracker.target_color_id == 0 and send_color_id > 0 and
                target_tracker.stable_frames >= STABLE_FRAMES_REQUIRED and not SLAVE_MODE):
            target_tracker.lock_auto_color(send_color_id)

        cx = x1 + w // 2
        cy = y1 + h // 2
        x2 = x1 + w
        y2 = y1 + h

        world_x, world_y = box_to_world(x1, y1, w, h)
        wx_mm = int(world_x * 10)
        wy_mm = int(world_y * 10)
        distance = calculate_distance(w, send_color_id)
        send_world_data(send_color_id, wx_mm, wy_mm, w, yellow_tracker.detected, pos_flag, obstacle_flag,
                        angle_flag, angle_cdeg)

        color = (255, 0, 0)
        if send_color_id == 1:
            color = (0, 170, 255)
        elif send_color_id == 2:
            color = (255, 0, 0)
        elif send_color_id == 3:
            color = (0, 255, 0)
        elif send_color_id == 4:
            color = (160, 96, 32)
        elif send_color_id == 5:
            color = (255, 255, 255)
        text = 'cid={} {}'.format(send_color_id, source)
        img.draw_rectangle((x1, y1, w, h), color=color, thickness=2)
        img.draw_cross(cx, cy, color=color, size=8, thickness=2)
        img.draw_string(x1, max(0, y1 - 15), text, color=color, scale=1)
        detect_count += 1

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 500:
            print('[{}] src={} cid={} w=({:.1f},{:.1f})cm dist={}mm box=({},{},{},{}) yflag={} pos={} obs={} fps={:.1f}'.format(
                frame_count, source, send_color_id, world_x, world_y, distance,
                x1, y1, x2, y2, yellow_tracker.detected, pos_flag, obstacle_flag, clock.fps()))
            last_print_time = now
    else:
        target_tracker.lost_frames += 1
        target_tracker.stable_frames = 0
        if target_tracker.should_reset_after_loss():
            reset_target_tracking_state()
        send_world_no_target(yellow_tracker.detected, pos_flag, obstacle_flag, angle_flag, angle_cdeg)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_time) >= 1000:
            print('[{}] src=none cid=0 yflag={} pos={} obs={} fps={:.1f}'.format(
                frame_count, yellow_tracker.detected, pos_flag, obstacle_flag, clock.fps()))
            last_print_time = now
    # Yellow detection is updated before pos_flag is calculated.

    # Draw dynamic cut line (debug)
    if ENABLE_DYNAMIC_CUT and detectors.dynamic_cut_valid:
        img.draw_line(CUT_LEFT_X, detectors.dynamic_cut_left_y, CUT_RIGHT_X, detectors.dynamic_cut_right_y,
                      color=(0, 180, 255), thickness=2)

    # Draw yellow boundary line
    yellow_tracker.draw(img)

    # Render bird-view debug image
    if BIRDVIEW_DEBUG:
        bird.clear()
        for _by in range(BIRD_H):
            Y = Y_MIN + _by * SY
            for _bx in range(BIRD_W):
                X = X_MIN + _bx * SX
                u, v = world_to_pixel(X, Y, H_world2pix)
                if 0 <= u < 320 and 0 <= v < 240:
                    bird.set_pixel(_bx, _by, img.get_pixel(u, v))

        if world_x != 0.0 or world_y != 0.0:
            bx_mark = int((world_x - X_MIN) / SX)
            by_mark = int((world_y - Y_MIN) / SY)
            if 0 <= bx_mark < BIRD_W and 0 <= by_mark < BIRD_H:
                bird.draw_cross(bx_mark, by_mark, color=(255, 0, 0), size=5, thickness=2)

        img.clear()
        img.draw_image(bird, 0, 0)
        img.draw_string(85, 5, "w=({:.1f},{:.1f})".format(world_x, world_y),
                        color=(0,255,255), scale=1)
        img.draw_string(85, 20, "FPS:{:.1f}".format(clock.fps()),
                        color=(255,255,0), scale=1)
