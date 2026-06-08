# Shared configuration for OpenART Plus and OpenART Mini.

PLUS_CONFIG = {
    "device_name": "OpenART Plus",
    "program_title": "OpenART Plus 多颜色沙包识别程序",
    "calibration_mode": False,
    "birdview_debug": False,
    "is_slave_car": False,
    "wb_fixed": True,
    "wb_gains": (101.00, 64.00, 97.00),
    "exposure_init": 1000,
    "fixed_exposure_us": 1200,
    "color_thresholds": [
        (34, 100, -41, 5, -72, -17),
        (10, 80, 22, 122, -17, 93),
        (50, 100, -128, -27, 20, 127),
        (20, 55, 30, -1, 50, 0),
        (53, 100, -10, 11, -11, 8),
    ],
    "model_enabled": True,
    "model_path": "/sd/dataset_25000_blur_0.30.tflite",
    "yellow_threshold": [(56, 100, -56, -2, 41, 127)],
    "yellow_roi_left": (0, 80, 70, 160),
    "yellow_roi_right": (250, 80, 70, 160),
    "calib_pixel": [
        [85, 240],
        [267, 240],
        [125, 129],
        [219, 129],
    ],
    "calib_world": [
        [-7.5, 7.5],
        [7.5, 7.5],
        [-7.5, 22.5],
        [7.5, 22.5],
    ],
}

MINI_CONFIG = {
    "device_name": "OpenART Mini",
    "program_title": "OpenART Mini 多颜色目标识别程序",
    "calibration_mode": False,
    "birdview_debug": False,
    "is_slave_car": True,
    "wb_fixed": True,
    "wb_gains": (101.00, 64.00, 97.00),
    "exposure_init": 1200,
    "fixed_exposure_us": 1200,
    "color_thresholds": [
        (23, 96, -49, 4, -53, -30),
        (10, 80, 22, 122, -17, 93),
        (50, 100, -128, -27, 20, 127),
        (20, 55, 30, -1, 50, 0),
        (53, 100, -10, 11, -11, 8),
    ],
    "model_enabled": True,
    "model_path": "/sd/dataset_25000_blur_0.30.tflite",
    "yellow_threshold": [(56, 100, -56, -2, 41, 127)],
    "yellow_roi_left": (0, 80, 70, 160),
    "yellow_roi_right": (250, 80, 70, 160),
    "calib_pixel": [
        [85, 240],
        [267, 240],
        [125, 129],
        [219, 129],
    ],
    "calib_world": [
        [-7.5, 7.5],
        [7.5, 7.5],
        [-7.5, 22.5],
        [7.5, 22.5],
    ],
}

ACTIVE_CONFIG = PLUS_CONFIG

def set_active_config(config):
    global ACTIVE_CONFIG
    ACTIVE_CONFIG = config
