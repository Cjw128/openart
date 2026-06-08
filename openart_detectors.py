import gc
import time

from openart_math import box_iou, center_dist2, clamp_int, pixel_to_world


class OpenArtDetectors:
    """Vision detectors and per-detector runtime state."""

    def __init__(self, cfg, target_tracker, H_pix2world):
        self.cfg = cfg
        self.target_tracker = target_tracker
        self.H_pix2world = H_pix2world
        self.dynamic_cut_left_y = cfg["detect_y_min"]
        self.dynamic_cut_right_y = cfg["detect_y_min"]
        self.dynamic_cut_valid = False
        self.dynamic_cut_miss_count = 0
        self.dynamic_detect_roi = cfg["detect_roi"]
        self.model_tf = None
        self.model_net = None
        self.model_ready = False
        self.model_status_last_print = 0

    def cut_line_y_at_x(self, x):
        dx = self.cfg["cut_right_x"] - self.cfg["cut_left_x"]
        if dx == 0:
            return self.dynamic_cut_left_y
        return int(self.dynamic_cut_left_y + (self.dynamic_cut_right_y - self.dynamic_cut_left_y) *
                   (x - self.cfg["cut_left_x"]) / dx)

    def pick_top_y_from_strip(self, blobs):
        if not blobs:
            return None
        top_y = 240
        for b in blobs:
            by = b.y()
            if by < top_y:
                top_y = by
        return top_y

    def update_dynamic_cut(self, img, frame_count):
        if (not self.cfg["enable_dynamic_cut"]) or (frame_count % self.cfg["cut_update_interval"] != 0):
            return

        strip_h = self.cfg["cut_scan_y_max"] - self.cfg["cut_scan_y_min"]
        half_w = self.cfg["cut_strip_half_w"]
        left_roi = (self.cfg["cut_left_x"] - half_w, self.cfg["cut_scan_y_min"], half_w * 2 + 1, strip_h)
        right_roi = (self.cfg["cut_right_x"] - half_w, self.cfg["cut_scan_y_min"], half_w * 2 + 1, strip_h)

        left_blobs = img.find_blobs(self.cfg["blue_ground_threshold"], roi=left_roi,
                                    pixels_threshold=self.cfg["cut_min_pixels"],
                                    area_threshold=self.cfg["cut_min_area"], merge=True)
        right_blobs = img.find_blobs(self.cfg["blue_ground_threshold"], roi=right_roi,
                                     pixels_threshold=self.cfg["cut_min_pixels"],
                                     area_threshold=self.cfg["cut_min_area"], merge=True)

        left_y_new = self.pick_top_y_from_strip(left_blobs)
        right_y_new = self.pick_top_y_from_strip(right_blobs)

        if left_y_new is not None and right_y_new is not None:
            self.dynamic_cut_miss_count = 0
            if not self.dynamic_cut_valid:
                self.dynamic_cut_left_y = left_y_new
                self.dynamic_cut_right_y = right_y_new
                self.dynamic_cut_valid = True
            else:
                a = self.cfg["cut_ema_alpha"]
                self.dynamic_cut_left_y = int(a * left_y_new + (1.0 - a) * self.dynamic_cut_left_y)
                self.dynamic_cut_right_y = int(a * right_y_new + (1.0 - a) * self.dynamic_cut_right_y)

            self.dynamic_cut_left_y = clamp_int(self.dynamic_cut_left_y, self.cfg["detect_y_min"],
                                                self.cfg["cut_scan_y_max"])
            self.dynamic_cut_right_y = clamp_int(self.dynamic_cut_right_y, self.cfg["detect_y_min"],
                                                 self.cfg["cut_scan_y_max"])
        else:
            self.dynamic_cut_miss_count += 1
            if self.dynamic_cut_miss_count > self.cfg["cut_max_miss"]:
                self.dynamic_cut_valid = False
                self.dynamic_cut_left_y = self.cfg["detect_y_min"]
                self.dynamic_cut_right_y = self.cfg["detect_y_min"]

        if self.dynamic_cut_valid:
            y_base = min(self.dynamic_cut_left_y, self.dynamic_cut_right_y) - self.cfg["cut_y_margin"]
            y_base = clamp_int(y_base, self.cfg["detect_y_min"], 239)
        else:
            y_base = self.cfg["detect_y_min"]
        self.dynamic_detect_roi = (0, y_base, 320, 240 - y_base)

    def clamp_roi_to_frame(self, x, y, w, h, y_min_limit):
        y_min_limit = clamp_int(y_min_limit, 0, 239)
        if y_min_limit > 240 - self.cfg["track_min_roi_h"]:
            y_min_limit = 240 - self.cfg["track_min_roi_h"]

        x0 = clamp_int(x, 0, 319)
        y0 = clamp_int(y, y_min_limit, 239)
        x1 = clamp_int(x + w, x0 + 1, 320)
        y1 = clamp_int(y + h, y0 + 1, 240)

        if (x1 - x0) < self.cfg["track_min_roi_w"]:
            if x0 + self.cfg["track_min_roi_w"] <= 320:
                x1 = x0 + self.cfg["track_min_roi_w"]
            else:
                x0 = 320 - self.cfg["track_min_roi_w"]
                x1 = 320

        if (y1 - y0) < self.cfg["track_min_roi_h"]:
            if y0 + self.cfg["track_min_roi_h"] <= 240:
                y1 = y0 + self.cfg["track_min_roi_h"]
            else:
                y0 = 240 - self.cfg["track_min_roi_h"]
                y1 = 240
            if y0 < y_min_limit:
                y0 = y_min_limit
                y1 = min(240, y0 + self.cfg["track_min_roi_h"])

        return (x0, y0, x1 - x0, y1 - y0)

    def make_roi_from_box(self, box):
        if not box:
            return self.dynamic_detect_roi
        x, y, w, h = box
        margin = self.cfg["color_track_margin"]
        x0 = clamp_int(x - margin, 0, 319)
        y0 = clamp_int(y - margin, self.cfg["detect_y_min"], 239)
        x1 = clamp_int(x + w + margin, x0 + 1, 320)
        y1 = clamp_int(y + h + margin, y0 + 1, 240)
        return (x0, y0, x1 - x0, y1 - y0)

    def threshold_items_for_color(self):
        items = []
        for i in range(len(self.cfg["color_thresholds"])):
            color_id = i + 1
            if self.cfg["model_enabled"] and color_id == self.cfg["white_bear_color_id"]:
                continue
            if self.target_tracker.target_color_id > 0 and color_id != self.target_tracker.target_color_id:
                continue
            items.append((color_id, self.cfg["color_thresholds"][i]))
        return items

    def valid_color_blob(self, blob, color_id):
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            return False
        aspect = w / h
        if color_id == 3:
            if aspect < 0.45 or aspect > 1.85:
                return False
            if blob.density() < 0.35:
                return False
        elif color_id == 4 or color_id == 5:
            if aspect < 0.30 or aspect > 2.50:
                return False
            if blob.pixels() < 120:
                return False
        else:
            if aspect < 0.60 or aspect > 1.80:
                return False
            if blob.density() < 0.50:
                return False
        return True

    def find_color_target(self, img, last_box):
        items = self.threshold_items_for_color()
        if not items:
            return None
        roi = self.make_roi_from_box(last_box)
        candidates = []
        for color_id, threshold in items:
            try:
                blobs = img.find_blobs([threshold], roi=roi,
                                       pixels_threshold=self.cfg["color_min_pixels"],
                                       area_threshold=self.cfg["color_min_area"],
                                       merge=True)
            except TypeError:
                blobs = None
            if not blobs:
                continue
            for blob in blobs:
                if self.cfg["enable_dynamic_cut"] and self.dynamic_cut_valid:
                    if blob.cy() < self.cut_line_y_at_x(blob.cx()) + self.cfg["cut_blob_delta"]:
                        continue
                if self.valid_color_blob(blob, color_id):
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

    def load_white_bear_model(self):
        if not self.cfg["model_enabled"]:
            self.model_tf = None
            self.model_net = None
            self.model_ready = False
            return
        try:
            import tf
            gc.collect()
            self.model_tf = tf
            self.model_net = tf.load(self.cfg["model_path"])
            self.model_ready = True
            print(">>> White bear model loaded: {} <<<".format(self.cfg["model_path"]))
        except Exception as e:
            self.model_tf = None
            self.model_net = None
            self.model_ready = False
            print(">>> White bear model disabled: {} <<<".format(e))

    def find_white_bear_model_target(self, img, last_box):
        if not self.model_ready or self.model_tf is None or self.model_net is None:
            now = time.ticks_ms()
            if time.ticks_diff(now, self.model_status_last_print) >= 1000:
                print(">>> White bear model not ready; target_id={} <<<".format(
                    self.target_tracker.target_color_id))
                self.model_status_last_print = now
            return None

        detect_img = img
        if self.cfg["model_input_scale"] != 1.0:
            try:
                detect_img = img.copy(self.cfg["model_input_scale"], 1)
            except Exception:
                return None

        best = None
        best_score = -1.0
        try:
            for obj in self.model_tf.detect(self.model_net, detect_img):
                x1, y1, x2, y2, label, score = obj
                label = int(label)
                score = float(score)
                if label != self.cfg["model_label_bear"] or score < self.cfg["model_fallback_score_threshold"]:
                    continue

                rx = int(x1 * img.width())
                ry = int(y1 * img.height())
                rw = int((x2 - x1) * img.width())
                rh = int((y2 - y1) * img.height())
                if rw <= 0 or rh <= 0:
                    continue

                if self.cfg["enable_dynamic_cut"] and self.dynamic_cut_valid:
                    if (ry + rh // 2) < self.cut_line_y_at_x(rx + rw // 2) + self.cfg["cut_blob_delta"]:
                        continue

                cand_box = (rx, ry, rw, rh)
                rank = score
                if last_box:
                    rank += box_iou(cand_box, last_box) * 0.4
                    rank -= center_dist2(cand_box, last_box) / 200000.0
                if rank > best_score:
                    best_score = rank
                    best = (self.cfg["white_bear_color_id"], rx, ry, rw, rh, score)
        except Exception as e:
            print(">>> White bear model detect failed: {} <<<".format(e))
            best = None

        if detect_img is not img:
            detect_img = None
        return best

    def beacon_roi_from_box(self, box):
        if (not self.cfg["enable_local_track_roi"]) or box is None:
            return self.cfg["beacon_detect_roi"]

        x, y, w, h = box
        mx = clamp_int(int(w * self.cfg["track_margin_x_ratio"]),
                       self.cfg["track_margin_x_min"], self.cfg["track_margin_x_max"])
        my = clamp_int(int(h * self.cfg["track_margin_y_ratio"]),
                       self.cfg["track_margin_y_min"], self.cfg["track_margin_y_max"])
        return self.clamp_roi_to_frame(x - mx, y - my, w + mx * 2, h + my * 2,
                                       self.cfg["beacon_detect_y_min"])

    def valid_beacon_blob(self, blob):
        w = blob.w()
        h = blob.h()
        if w <= 0 or h <= 0:
            return False

        aspect = w / h
        if aspect < self.cfg["beacon_min_aspect_ratio"] or aspect > self.cfg["beacon_max_aspect_ratio"]:
            return False
        if blob.density() < self.cfg["beacon_min_density"]:
            return False
        return True

    def beacon_center_dist2(self, blob, box):
        bx = box[0] + box[2] // 2
        by = box[1] + box[3] // 2
        dx = blob.cx() - bx
        dy = blob.cy() - by
        return dx * dx + dy * dy

    def find_beacon_blob(self, img, last_box):
        roi = self.beacon_roi_from_box(last_box)
        blobs = img.find_blobs(self.cfg["beacon_threshold"], roi=roi,
                               pixels_threshold=self.cfg["beacon_min_pixels"],
                               area_threshold=self.cfg["beacon_min_area"],
                               merge=self.cfg["beacon_merge_blobs"])
        if not blobs:
            return None

        candidates = []
        for blob in blobs:
            if self.valid_beacon_blob(blob):
                candidates.append(blob)
        if not candidates:
            return None

        if last_box is not None:
            return max(candidates,
                       key=lambda b: b.pixels() - self.beacon_center_dist2(b, last_box) // 20)
        return max(candidates, key=lambda b: b.pixels())

    def box_to_world(self, x, y, w, h):
        if self.H_pix2world is None:
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
            wx, wy = pixel_to_world(px, py, self.H_pix2world)
            wx_sum += wx
            wy_sum += wy
        return (wx_sum / 4.0, wy_sum / 4.0)

    def calculate_distance(self, pixel_width, color_id=1):
        if pixel_width <= 0:
            return -1
        widths = self.cfg["target_real_width"]
        real_width = widths[color_id - 1] if 1 <= color_id <= len(widths) else 70.0
        distance = (real_width * self.cfg["focal_length"]) / pixel_width
        if distance < self.cfg["min_detect_distance"] or distance > self.cfg["max_detect_distance"]:
            return -1
        return int(distance)

    def detect_obstacle(self, img):
        blobs = img.find_blobs(self.cfg["obstacle_threshold"], roi=self.cfg["obstacle_roi"],
                               pixels_threshold=self.cfg["obstacle_min_pixels"],
                               area_threshold=self.cfg["obstacle_min_area"],
                               merge=True)
        if not blobs:
            return (self.cfg["obstacle_none"], None)

        in_path = []
        for blob in blobs:
            left = blob.x()
            right = blob.x() + blob.w()
            img.draw_rectangle(blob.rect(), color=(255, 128, 0), thickness=2)
            if not (right < self.cfg["obstacle_path_x_min"] or left > self.cfg["obstacle_path_x_max"]):
                in_path.append(blob)

        if not in_path:
            return (self.cfg["obstacle_none"], blobs)

        if len(in_path) > 1:
            return (self.cfg["obstacle_blocked"], blobs)

        blob = in_path[0]
        left = blob.x()
        right = blob.x() + blob.w()
        overlap_left = max(left, self.cfg["obstacle_path_x_min"])
        overlap_right = min(right, self.cfg["obstacle_path_x_max"])
        overlap_center = (overlap_left + overlap_right) // 2

        if overlap_center < 160:
            return (self.cfg["obstacle_move_right"], blobs)
        return (self.cfg["obstacle_move_left"], blobs)

    def box_hits_obstacle(self, box, obstacle_blobs):
        if not obstacle_blobs:
            return False
        x, y, w, h = box
        x2 = x + w
        y2 = y + h
        for blob in obstacle_blobs:
            bx = blob.x()
            by = blob.y()
            bx2 = bx + blob.w()
            by2 = by + blob.h()
            inter_w = min(x2, bx2) - max(x, bx)
            inter_h = min(y2, by2) - max(y, by)
            if inter_w > 0 and inter_h > 0 and inter_w * inter_h >= self.cfg["obstacle_target_overlap_pixels"]:
                return True
        return False
