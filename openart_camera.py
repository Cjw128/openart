TARGET_BRIGHTNESS = 45.0
EXPOSURE_MIN = 100
EXPOSURE_MAX = 4500
GAIN_INIT = 0
CALIBRATION_DELAY = 50
CALIBRATION_SETTLE_MS = 200
MAX_EXPOSURE_STEP_UP = 1.6
MAX_EXPOSURE_STEP_DOWN = 0.6


def set_exposure(sensor, exposure_us):
    sensor.set_auto_exposure(False, exposure_us=exposure_us)


def measure_brightness(sensor, roi=None):
    img = sensor.snapshot()
    if roi:
        stats = img.get_statistics(roi=roi)
    else:
        stats = img.get_statistics()
    return stats.l_mean()


def measure_brightness_stable(sensor, time, samples=5, roi=None):
    total = 0.0
    for _ in range(samples):
        total += measure_brightness(sensor, roi)
        time.sleep_ms(CALIBRATION_DELAY)
    return total / samples


def calculate_exposure_adjustment(current_brightness, target_brightness, current_exposure):
    if current_brightness <= 0:
        return current_exposure
    ratio = target_brightness / current_brightness
    if ratio > MAX_EXPOSURE_STEP_UP:
        ratio = MAX_EXPOSURE_STEP_UP
    elif ratio < MAX_EXPOSURE_STEP_DOWN:
        ratio = MAX_EXPOSURE_STEP_DOWN
    new_exposure = int(current_exposure * ratio)
    return max(EXPOSURE_MIN, min(EXPOSURE_MAX, new_exposure))


def calibrate_brightness_startup(sensor, time, wb_fixed, wb_gains, exposure_init,
                                 target=TARGET_BRIGHTNESS, samples=5, roi=None,
                                 max_iterations=3):
    print("=" * 40)
    print(">>> 启动亮度校准 <<<")

    sensor.set_auto_exposure(False, exposure_us=exposure_init)
    sensor.set_auto_gain(False, gain_db=GAIN_INIT)

    exposure = exposure_init
    set_exposure(sensor, exposure)
    time.sleep_ms(CALIBRATION_SETTLE_MS)

    for _ in range(max_iterations):
        current_brightness = measure_brightness_stable(sensor, time, samples=samples, roi=roi)
        error = abs(target - current_brightness)
        if error < 3.0:
            break
        exposure = calculate_exposure_adjustment(current_brightness, target, exposure)
        set_exposure(sensor, exposure)
        time.sleep_ms(CALIBRATION_SETTLE_MS)

    if wb_fixed:
        print("白平衡: 使用固定增益 {}".format(wb_gains))
    else:
        print("等待白平衡收敛...")
        sensor.skip_frames(time=1500)
        sensor.set_auto_whitebal(False)
        print("白平衡已锁定")
    print("=" * 40)
    return exposure
