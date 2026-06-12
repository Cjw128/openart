import sensor
import time
import math
from machine import UART


# ======================================================================
# OpenART Plus - horizontal yellow line angle correction with IPM
#
# 场景假设:
# 1. 黄色线在图像里大致横向出现。
# 2. 摄像头固定在车上，CALIB_PIXEL/CALIB_WORLD 已按实际安装标定。
# 3. 输出 angle_deg: 地面坐标中黄线相对“正横向”的角度。
#    angle_deg = 0 表示线与车体 X 轴平行；正值表示右侧更远。
# ======================================================================


FLAG_VALID = 0x01
FLAG_WEAK = 0x02
FLAG_FILTERED = 0x04


def clamp(value, lo, hi):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


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


class YellowCrosslineIPM:
    def __init__(self, uart_enabled=True, uart_id=2):
        self.width = 320
        self.height = 240
        self.center_x = self.width // 2

        # 黄色阈值需要按赛场光照重新标定。
        self.yellow_threshold = [(66, 95, 5, -27, 40, 95)]

        # 从 main.py 复制来的逆透视标定点。
        self.calib_pixel = [
            [85, 240],
            [267, 240],
            [125, 129],
            [219, 129],
        ]
        self.calib_world = [
            [-7.5, 7.5],
            [7.5, 7.5],
            [-7.5, 22.5],
            [7.5, 22.5],
        ]
        self.H_pix2world = calc_homography(self.calib_pixel, self.calib_world)

        # 竖向小窗口采样横线。每个窗口取最大黄色 blob 的中心点。
        self.scan_xs = [35, 65, 95, 125, 155, 185, 215, 245, 275]
        self.roi_half_w = 9
        self.scan_y_min = 70
        self.scan_y_max = 238
        self.pixels_threshold = 12
        self.area_threshold = 12
        self.merge = True

        self.min_points = 4
        self.max_line_residual_cm = 2.8
        self.min_world_span_cm = 7.0

        self.filtered_angle = 0.0
        self.filtered_center_y = 0.0
        self.filter_alpha = 0.45
        self.has_filter = False
        self.lost_frames = 0

        self.debug_draw = True
        self.uart_enabled = uart_enabled
        self.uart = UART(uart_id, baudrate=115200) if uart_enabled else None
        self.clock = time.clock()
        self.frame_id = 0
        self.last_print_ms = time.ticks_ms()

    def configure_sensor(self):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_framerate(60)

        sensor.set_auto_whitebal(False, rgb_gain_db=(101.00, 64.00, 97.00))
        sensor.skip_frames(time=500)
        sensor.set_auto_exposure(False, exposure_us=1200)
        sensor.set_auto_gain(False, gain_db=0)
        sensor.skip_frames(time=300)

    def set_threshold(self, threshold):
        if isinstance(threshold, tuple):
            self.yellow_threshold = [threshold]
        else:
            self.yellow_threshold = threshold

    def set_debug_draw(self, enabled):
        self.debug_draw = bool(enabled)

    def set_uart_enabled(self, enabled):
        self.uart_enabled = bool(enabled)

    def _pick_blob(self, blobs):
        if not blobs:
            return None
        best_blob = None
        best_score = -100000
        for blob in blobs:
            # Prefer the yellow segment whose bottom edge is closest to the car.
            bottom_y = blob.y() + blob.h() - 1
            score = bottom_y * 1000 + blob.pixels()
            if score > best_score:
                best_score = score
                best_blob = blob
        return best_blob

    def _sample_points(self, img):
        points = []
        roi_h = self.scan_y_max - self.scan_y_min + 1
        for sx in self.scan_xs:
            x0 = clamp(sx - self.roi_half_w, 0, self.width - 1)
            x1 = clamp(sx + self.roi_half_w, 0, self.width - 1)
            roi = (x0, self.scan_y_min, x1 - x0 + 1, roi_h)

            blobs = img.find_blobs(
                self.yellow_threshold,
                roi=roi,
                pixels_threshold=self.pixels_threshold,
                area_threshold=self.area_threshold,
                merge=self.merge,
            )
            blob = self._pick_blob(blobs)
            if blob is None:
                continue

            # Use the nearest edge of the yellow band as the stable angle baseline.
            px = blob.cx()
            py = blob.y() + blob.h() - 1
            wx, wy = pixel_to_world(px, py, self.H_pix2world)
            points.append({
                "px": px,
                "py": py,
                "wx": wx,
                "wy": wy,
                "pixels": blob.pixels(),
                "rect": blob.rect(),
            })
        return points

    def _fit_world_line(self, points):
        # 拟合 wy = k * wx + b。k 的角度就是横线相对车体横向的偏角。
        n = len(points)
        if n < self.min_points:
            return None

        sx = 0.0
        sy = 0.0
        sxx = 0.0
        sxy = 0.0
        min_x = points[0]["wx"]
        max_x = points[0]["wx"]
        for p in points:
            x = p["wx"]
            y = p["wy"]
            sx += x
            sy += y
            sxx += x * x
            sxy += x * y
            if x < min_x:
                min_x = x
            if x > max_x:
                max_x = x

        span_x = max_x - min_x
        if span_x < self.min_world_span_cm:
            return None

        denom = n * sxx - sx * sx
        if abs(denom) < 1e-6:
            return None

        k = (n * sxy - sx * sy) / denom
        b = (sy - k * sx) / n

        residual_sum = 0.0
        for p in points:
            err = p["wy"] - (k * p["wx"] + b)
            residual_sum += err * err
        residual = math.sqrt(residual_sum / n)

        angle_deg = math.atan(k) * 180.0 / math.pi
        center_y = k * 0.0 + b
        return {
            "k": k,
            "b": b,
            "angle_deg": angle_deg,
            "center_y_cm": center_y,
            "residual_cm": residual,
            "span_x_cm": span_x,
        }

    def _build_result(self, points, fit):
        flags = 0
        if fit is None:
            self.lost_frames += 1
            return {
                "frame_id": self.frame_id,
                "valid": False,
                "flags": flags,
                "angle_deg": self.filtered_angle,
                "angle_cdeg": int(self.filtered_angle * 100),
                "center_y_cm": self.filtered_center_y,
                "center_y_mm": int(self.filtered_center_y * 10),
                "confidence": 0,
                "point_count": len(points),
                "points": points,
                "lost_frames": self.lost_frames,
                "fit": fit,
            }

        self.lost_frames = 0
        confidence = len(points) * 12
        confidence += int(fit["span_x_cm"] * 2)
        confidence -= int(fit["residual_cm"] * 15)
        confidence = clamp(confidence, 1, 100)

        if fit["residual_cm"] > self.max_line_residual_cm:
            flags |= FLAG_WEAK
            confidence = min(confidence, 45)

        raw_angle = fit["angle_deg"]
        raw_center_y = fit["center_y_cm"]
        if self.has_filter:
            self.filtered_angle = (
                self.filtered_angle * (1.0 - self.filter_alpha) +
                raw_angle * self.filter_alpha
            )
            self.filtered_center_y = (
                self.filtered_center_y * (1.0 - self.filter_alpha) +
                raw_center_y * self.filter_alpha
            )
            flags |= FLAG_FILTERED
        else:
            self.filtered_angle = raw_angle
            self.filtered_center_y = raw_center_y
            self.has_filter = True

        flags |= FLAG_VALID
        angle_cdeg = int(self.filtered_angle * 100)
        center_y_mm = int(self.filtered_center_y * 10)

        return {
            "frame_id": self.frame_id,
            "valid": True,
            "flags": flags,
            "angle_deg": self.filtered_angle,
            "angle_cdeg": angle_cdeg,
            "center_y_cm": self.filtered_center_y,
            "center_y_mm": center_y_mm,
            "confidence": confidence,
            "point_count": len(points),
            "points": points,
            "lost_frames": self.lost_frames,
            "fit": fit,
        }

    def process_frame(self, img):
        if self.H_pix2world is None:
            return self._build_result([], None)

        points = self._sample_points(img)
        fit = self._fit_world_line(points)
        result = self._build_result(points, fit)

        if self.debug_draw:
            self.draw_debug(img, result)

        return result

    def send_result(self, result):
        if not self.uart_enabled or self.uart is None:
            return

        # 12字节协议:
        # [0:1]  AA 55
        # [2]    valid
        # [3:4]  angle_cdeg int16, 角度 * 100
        # [5:6]  center_y_mm int16, 黄线中心前向距离
        # [7]    confidence
        # [8]    point_count
        # [9]    lost_frames
        # [10]   flags
        # [11]   checksum = sum(data[2:11]) & 0xFF
        data = bytearray(12)
        angle = result["angle_cdeg"] & 0xFFFF
        center_y = result["center_y_mm"] & 0xFFFF

        data[0] = 0xAA
        data[1] = 0x55
        data[2] = 0x01 if result["valid"] else 0x00
        data[3] = angle & 0xFF
        data[4] = (angle >> 8) & 0xFF
        data[5] = center_y & 0xFF
        data[6] = (center_y >> 8) & 0xFF
        data[7] = result["confidence"] & 0xFF
        data[8] = result["point_count"] & 0xFF
        data[9] = result["lost_frames"] & 0xFF
        data[10] = result["flags"] & 0xFF
        data[11] = sum(data[2:11]) & 0xFF
        self.uart.write(data)

    def draw_debug(self, img, result):
        for sx in self.scan_xs:
            x0 = clamp(sx - self.roi_half_w, 0, self.width - 1)
            x1 = clamp(sx + self.roi_half_w, 0, self.width - 1)
            img.draw_rectangle(
                x0,
                self.scan_y_min,
                x1 - x0 + 1,
                self.scan_y_max - self.scan_y_min + 1,
                color=(60, 60, 60),
            )

        points = sorted(result["points"], key=lambda p: p["px"])
        for p in points:
            img.draw_rectangle(p["rect"], color=(255, 255, 0), thickness=1)
            img.draw_cross(p["px"], p["py"], color=(255, 0, 0), size=7, thickness=2)

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            img.draw_line(p0["px"], p0["py"], p1["px"], p1["py"],
                          color=(0, 180, 255), thickness=2)

        img.draw_string(
            2,
            2,
            "A:{:.2f} Y:{:.1f} C:{} P:{}".format(
                result["angle_deg"],
                result["center_y_cm"],
                result["confidence"],
                result["point_count"],
            ),
            color=(255, 255, 255),
            scale=1,
        )

    def run_forever(self):
        self.configure_sensor()

        print("=" * 56)
        print("OpenART Yellow Crossline IPM Angle")
        print("angle_deg: 0=horizontal, + means right side farther")
        print("uart packet: [AA 55 valid angL angH yL yH conf pts lost flags sum]")
        print("=" * 56)

        while True:
            self.clock.tick()
            self.frame_id += 1
            img = sensor.snapshot().lens_corr(2)
            result = self.process_frame(img)
            self.send_result(result)

            now = time.ticks_ms()
            if time.ticks_diff(now, self.last_print_ms) >= 300:
                print(
                    "[{}] valid={} angle={:.2f} y={:.1f} pts={} conf={} fps={:.1f}".format(
                        self.frame_id,
                        result["valid"],
                        result["angle_deg"],
                        result["center_y_cm"],
                        result["point_count"],
                        result["confidence"],
                        self.clock.fps(),
                    )
                )
                self.last_print_ms = now


def create_crossline_ipm(uart_enabled=True, uart_id=2):
    return YellowCrosslineIPM(uart_enabled=uart_enabled, uart_id=uart_id)


def main():
    detector = create_crossline_ipm()
    detector.run_forever()


if __name__ == "__main__":
    main()
