import time

from openart_math import calc_homography, pixel_to_world


def run_calibration_mode(sensor, clock, color_thresholds, detect_roi, calib_world):
    calib_pts = []
    calib_stable = 0
    calib_last = [0, 0]
    calib_wait_remove = False
    stable_need = 20
    move_thr = 10
    calib_t = time.ticks_ms()

    print("=" * 50)
    print(">>> 前视逆透视标定模式 <<<")
    print("  [2]----[3]  远处")
    print("   |      |")
    print("  [0]----[1]  近处")
    print("=" * 50)
    print("请将标记物放在位置 0 ...")

    calib_thresholds = color_thresholds[:2]

    while len(calib_pts) < 4:
        clock.tick()
        img = sensor.snapshot()

        for gx in range(0, 321, 40):
            img.draw_line(gx, 0, gx, 240, color=(64, 64, 64))
        for gy in range(0, 241, 40):
            img.draw_line(0, gy, 320, gy, color=(64, 64, 64))
        img.draw_line(160, 0, 160, 240, color=(0, 128, 0))
        img.draw_line(0, 120, 320, 120, color=(0, 128, 0))

        for ci in range(len(calib_pts)):
            cp = calib_pts[ci]
            img.draw_circle(cp[0], cp[1], 6, color=(0, 255, 0), thickness=2)
            img.draw_string(cp[0] + 8, cp[1] - 4, str(ci), color=(0, 255, 0), scale=2)

        img.draw_string(2, 2, "Calib {}/4".format(len(calib_pts)), color=(255, 255, 0), scale=2)

        blobs = img.find_blobs(calib_thresholds, roi=detect_roi,
                               pixels_threshold=50, area_threshold=50, merge=True)

        if blobs:
            blob = max(blobs, key=lambda b: b.pixels())
            bx, by = blob.cx(), blob.cy()

            if calib_wait_remove:
                img.draw_string(2, 220, "Remove marker...", color=(255, 128, 0), scale=2)
            else:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(bx, by, color=(255, 0, 0), size=10, thickness=2)
                img.draw_string(bx + 12, by - 8, "({},{})".format(bx, by),
                                color=(255, 255, 0), scale=2)

                dx = abs(bx - calib_last[0])
                dy = abs(by - calib_last[1])
                if dx < move_thr and dy < move_thr:
                    calib_stable += 1
                    bar = min(calib_stable * 100 // stable_need, 100)
                    img.draw_rectangle((10, 225, bar, 10), color=(0, 255, 0), fill=True)
                    img.draw_rectangle((10, 225, 100, 10), color=(255, 255, 255))

                    if calib_stable >= stable_need:
                        calib_pts.append([bx, by])
                        n = len(calib_pts) - 1
                        print(">>> Point {}: [{}, {}]".format(n, bx, by))
                        calib_stable = 0
                        calib_wait_remove = True
                        if len(calib_pts) < 4:
                            print("请拿走标记物，放到位置 {} ...".format(len(calib_pts)))
                else:
                    calib_stable = 0
                calib_last = [bx, by]
        else:
            calib_stable = 0
            calib_wait_remove = False

    print("")
    print("=" * 50)
    print("标定完成! 复制以下内容到 CALIB_PIXEL:")
    print("")
    print("CALIB_PIXEL = [")
    for ci in range(4):
        cp = calib_pts[ci]
        print("    [{}, {}],     # Point {}".format(cp[0], cp[1], ci))
    print("]")
    print("")

    H_test = calc_homography(calib_pts, calib_world)
    if H_test:
        print("进入验证模式: 移动标记物查看世界坐标")

    while True:
        clock.tick()
        img = sensor.snapshot()
        for ci in range(4):
            cp = calib_pts[ci]
            img.draw_circle(cp[0], cp[1], 6, color=(0, 255, 0), thickness=2)
            img.draw_string(cp[0] + 8, cp[1] - 4, str(ci), color=(0, 255, 0), scale=2)
        img.draw_string(2, 2, "Verify", color=(0, 255, 0), scale=2)

        blobs = img.find_blobs(calib_thresholds, roi=detect_roi,
                               pixels_threshold=50, area_threshold=50, merge=True)
        if blobs and H_test:
            blob = max(blobs, key=lambda b: b.pixels())
            bx, by = blob.cx(), blob.cy()
            wx, wy = pixel_to_world(bx, by, H_test)
            img.draw_cross(bx, by, color=(255, 0, 0), size=10, thickness=2)
            img.draw_string(bx + 12, by - 8, "({},{})".format(bx, by),
                            color=(255, 255, 0), scale=1)
            img.draw_string(bx + 12, by + 8, "w({:.1f},{:.1f})cm".format(wx, wy),
                            color=(0, 255, 255), scale=1)
            now = time.ticks_ms()
            if time.ticks_diff(now, calib_t) >= 500:
                print("pixel=({},{}) -> world=({:.1f},{:.1f})cm".format(bx, by, wx, wy))
                calib_t = now
