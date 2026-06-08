class YellowLineTracker:
    """Owns yellow-line hysteresis, carry-mode crossing state, and boundary geometry."""

    def __init__(self, cfg, pixel_to_world_fn):
        self.cfg = cfg
        self.pixel_to_world = pixel_to_world_fn
        self.reset()

    def reset(self):
        self.lost_count = 0
        self.seen_in_carry = False
        self.tracking = False
        self.detected = False
        self.recent_count = 0
        self.boundary_y = 0
        self.boundary_left_y = 0
        self.boundary_right_y = 0
        self.boundary_wy = 0.0

    def enter_carry_mode(self):
        had_seen_yellow = self.detected or self.recent_count > 0
        self.reset()
        if had_seen_yellow:
            self.seen_in_carry = True
            self.tracking = True

    def update(self, img, frame_count, mode, H_pix2world):
        if frame_count % self.cfg["detect_interval"] != 0:
            return

        pixels_threshold = self.cfg["keep_pixels"] if self.tracking else self.cfg["enter_pixels"]
        left_blobs = img.find_blobs(self.cfg["threshold"], roi=self.cfg["roi_left"],
                                    pixels_threshold=pixels_threshold,
                                    area_threshold=20, merge=True)
        right_blobs = img.find_blobs(self.cfg["threshold"], roi=self.cfg["roi_right"],
                                     pixels_threshold=pixels_threshold,
                                     area_threshold=20, merge=True)

        raw_seen = (left_blobs and right_blobs)
        if raw_seen:
            self.tracking = True
            self.detected = True
            self.recent_count = self.cfg["recent_detections"]
        else:
            self.detected = False
            if self.recent_count > 0:
                self.recent_count -= 1
            if mode == self.cfg["mode_search"]:
                self.tracking = False

        if self.detected:
            left_blob = max(left_blobs, key=lambda b: b.pixels())
            right_blob = max(right_blobs, key=lambda b: b.pixels())
            self.boundary_left_y = left_blob.cy()
            self.boundary_right_y = right_blob.cy()
            self.boundary_y = (self.boundary_left_y + self.boundary_right_y) // 2
            if H_pix2world:
                _, self.boundary_wy = self.pixel_to_world(160, self.boundary_y, H_pix2world)
        elif mode == self.cfg["mode_search"]:
            self.boundary_y = 0
            self.boundary_left_y = 0
            self.boundary_right_y = 0
            self.boundary_wy = 0.0

    def position_flag(self, mode):
        if mode == self.cfg["mode_carry"]:
            if self.detected:
                self.seen_in_carry = True
                self.lost_count = 0
                return (self.cfg["pos_no_boundary"], mode)
            if self.seen_in_carry:
                self.lost_count += 1
                if self.lost_count >= self.cfg["lost_threshold"]:
                    return (self.cfg["pos_crossed"], self.cfg["mode_wait_turn"])
            return (self.cfg["pos_no_boundary"], mode)
        if mode == self.cfg["mode_wait_turn"]:
            return (self.cfg["pos_crossed"], mode)
        if mode == self.cfg["mode_search"]:
            self.lost_count = 0
            if self.detected:
                return (self.cfg["pos_right_side"], mode)
        return (self.cfg["pos_no_boundary"], mode)

    def draw(self, img):
        if self.boundary_y > 0:
            img.draw_line(0, self.boundary_y, 320, self.boundary_y,
                          color=(255, 255, 0), thickness=2)


class ReturnBeaconTracker:
    """Keeps the return-beacon local tracking box and loss counter together."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_box = None
        self.lost_frames = 0


class TargetTracker:
    """Owns color/model target lock state across frames and host commands."""

    def __init__(self, color_thresholds, white_bear_color_id, color_lost_frames, max_lost_frames):
        self.color_thresholds = color_thresholds
        self.white_bear_color_id = white_bear_color_id
        self.color_lost_frames = color_lost_frames
        self.max_lost_frames = max_lost_frames
        self.reset()

    def reset(self):
        self.active_threshold = None
        self.active_color_id = 0
        self.thresholds = self.color_thresholds
        self.lost_frames = 0
        self.stable_frames = 0
        self.local_rect = None
        self.last_pixels = -1
        self.force_global_next = False
        self.local_miss_count = 0
        self.target_color_id = 0
        self.track_active = False
        self.track_box = None
        self.track_color_id = 0
        self.color_lost_count = 0

    def set_target_color(self, color_id):
        self.target_color_id = color_id
        self.active_color_id = color_id
        if color_id == self.white_bear_color_id:
            self.active_threshold = None
            self.thresholds = self.color_thresholds
        else:
            self.active_threshold = [self.color_thresholds[color_id - 1]]
            self.thresholds = self.active_threshold
        self.lost_frames = 0
        self.stable_frames = 0
        self.clear_frame_track()
        self.local_rect = None
        self.last_pixels = -1
        self.force_global_next = False
        self.local_miss_count = 0

    def clear_frame_track(self):
        self.track_active = False
        self.track_box = None
        self.track_color_id = 0
        self.color_lost_count = 0

    def mark_found(self, color_id, box):
        self.track_active = True
        self.track_box = box
        self.track_color_id = color_id
        self.color_lost_count = 0

    def hold_last_box(self):
        if not self.track_active or not self.track_box:
            return None
        self.color_lost_count += 1
        if self.color_lost_count <= self.color_lost_frames:
            return (self.track_color_id,) + self.track_box
        self.clear_frame_track()
        return None

    def lock_auto_color(self, color_id):
        self.target_color_id = color_id
        self.active_color_id = color_id
        self.active_threshold = [self.color_thresholds[color_id - 1]]
        self.thresholds = self.active_threshold

    def should_reset_after_loss(self):
        return (self.lost_frames > self.max_lost_frames and
                (self.target_color_id > 0 or self.active_threshold is not None or self.track_active))
