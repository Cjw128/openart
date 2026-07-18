# OpenART IDE field image capture.
# The camera is configured like calib_ide_autocalib_competition.py, then one
# clean QVGA JPEG is saved to the SD card every second until the script stops.

import os
import sensor
import time


SD_OUTPUT_DIR = "/sd/field_capture"
FLASH_OUTPUT_DIRS = ("/flash/field_capture", "/field_capture")
FILE_PREFIX = "field_"
CAPTURE_INTERVAL_MS = 1000
JPEG_QUALITY = 95

WB_GAINS = (101.00, 64.00, 97.00)
L_TARGET = 38
L_TOL = 1.0
STEP_MIN = 0.80
STEP_MAX = 1.10
EXPOSURE_INIT = 1400
EXPOSURE_MIN = 100
EXPOSURE_MAX = 4500
METER_AVG = 4
HIGHLIGHT_LUQ = 92


def clamp_int(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def calibrate_exposure():
    exposure = EXPOSURE_INIT
    sensor.set_auto_exposure(False, exposure_us=exposure)

    for iteration in range(10):
        time.sleep_ms(150)
        for _ in range(3):
            sensor.snapshot()

        l_sum = 0.0
        l_uq = 0
        for _ in range(METER_AVG):
            img = sensor.snapshot()
            stats = img.get_statistics()
            l_sum += stats.l_mean()
            l_uq = stats.l_uq()

        lightness = l_sum / METER_AVG
        print("[exposure] iter=%d exposure_us=%d L=%.1f Luq=%d" %
              (iteration, exposure, lightness, l_uq))
        if abs(lightness - L_TARGET) <= L_TOL:
            break

        ratio = L_TARGET / max(lightness, 1.0)
        ratio = min(max(ratio, STEP_MIN), STEP_MAX)
        exposure = clamp_int(int(exposure * ratio),
                             EXPOSURE_MIN, EXPOSURE_MAX)
        sensor.set_auto_exposure(False, exposure_us=exposure)

    for _ in range(5):
        for _ in range(3):
            img = sensor.snapshot()
        l_uq = img.get_statistics().l_uq()
        if l_uq < HIGHLIGHT_LUQ:
            break
        exposure = max(int(exposure * 0.8), EXPOSURE_MIN)
        sensor.set_auto_exposure(False, exposure_us=exposure)
        time.sleep_ms(150)

    print("[exposure] final exposure_us=%d" % exposure)
    return exposure


def prepare_output_dir(path):
    try:
        os.stat(path)
    except OSError:
        try:
            os.mkdir(path)
            print("[capture] created " + path)
        except OSError as error:
            print("[capture] unavailable %s: %s" % (path, error))
            return False

    # Verify that this is a writable directory, not merely an existing path.
    probe_path = path + "/.capture_probe"
    try:
        probe = open(probe_path, "w")
        probe.write("ok")
        probe.close()
        os.remove(probe_path)
        return True
    except OSError as error:
        print("[capture] not writable %s: %s" % (path, error))
        return False


def choose_output_dir():
    if prepare_output_dir(SD_OUTPUT_DIR):
        return SD_OUTPUT_DIR

    print("[capture] WARNING: /sd is not mounted; using internal flash")
    print("[capture] WARNING: flash space is limited; stop capture promptly")
    for path in FLASH_OUTPUT_DIRS:
        if prepare_output_dir(path):
            return path
    raise Exception("No writable storage. Insert and mount a FAT32 SD card.")


def next_file_index(output_dir):
    # Avoid os.listdir(path): some OpenART firmware returns ENODEV for it.
    for value in range(1, 100000):
        path = output_dir + "/" + FILE_PREFIX + ("%05d" % value) + ".jpg"
        try:
            os.stat(path)
        except OSError:
            return value
    raise Exception("Capture file index exceeded 99999")


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_framerate(60)
sensor.set_auto_whitebal(False, rgb_gain_db=WB_GAINS)
sensor.set_auto_gain(False, gain_db=0)
sensor.set_auto_exposure(False, exposure_us=EXPOSURE_INIT)
sensor.skip_frames(time=800)
sensor.set_vflip(True)

exposure_us = calibrate_exposure()
output_dir = choose_output_dir()
file_index = next_file_index(output_dir)

print("[capture] one image per second; stop the script to finish")
print("[capture] output=" + output_dir)

while True:
    cycle_start = time.ticks_ms()
    img = sensor.snapshot()
    stats = img.get_statistics()
    filename = FILE_PREFIX + ("%05d" % file_index) + ".jpg"
    path = output_dir + "/" + filename
    img.save(path, quality=JPEG_QUALITY)
    print("[capture] %s exposure_us=%d LAB=(%d,%d,%d)" %
          (path, exposure_us, stats.l_mean(), stats.a_mean(), stats.b_mean()))
    file_index += 1

    elapsed = time.ticks_diff(time.ticks_ms(), cycle_start)
    remaining = CAPTURE_INTERVAL_MS - elapsed
    if remaining > 0:
        time.sleep_ms(remaining)
