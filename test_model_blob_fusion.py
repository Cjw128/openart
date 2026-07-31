import ast
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
MAIN_FILE = ROOT / "main.py"
OBSERVER_FILE = ROOT / "world_coordinate_test.py"
RUNTIME_FILES = (MAIN_FILE, ROOT / "minimain.py", OBSERVER_FILE)

FUSION_ASSIGNMENTS = {
    "ENABLE_COMPLETED_COLOR_EXCLUSION",
    "ENABLE_TARGET_ANCHOR_LOCK",
    "ID2_ABSOLUTE_PRIORITY",
    "MODEL_COLOR_IDS",
    "MODEL_HOLD_FRAMES",
    "FIRST_LOCK_SCORE_MIN",
    "FIRST_LOCK_WINDOW_FRAMES",
    "FIRST_LOCK_REQUIRED_HITS",
    "FIRST_LOCK_MATCH_CENTER_PX",
    "FIRST_LOCK_MATCH_CENTER2",
    "FIRST_LOCK_SIZE_DELTA_PERCENT",
    "COLOR_ROI_INSET_X_PERCENT",
    "COLOR_ROI_INSET_TOP_PERCENT",
    "COLOR_ROI_INSET_BOTTOM_PERCENT",
    "COLOR_DETECT_Y_MAX",
    "COLOR_TRACK_MODEL_PAD_PERCENT",
    "COLOR_TRACK_LOCAL_PAD_PERCENT",
    "COLOR_TRACK_MIN_PAD",
    "COLOR_TRACK_MAX_MISSES",
    "COLOR_OUTPUT_HOLD_FRAMES",
    "COLOR_TRACK_MIN_COVER_X1000",
    "COLOR_TRACK_GATE_OVERLAP_PERCENT",
    "COLOR_TRACK_AREA_MIN_PERCENT",
    "COLOR_TRACK_AREA_MAX_PERCENT",
    "COLOR_TRACK_CENTER_SCALE_X100",
    "OUTPUT_SMOOTH_ALPHA_X100",
    "OUTPUT_SMOOTH_RESET_CENTER2",
    "model_track",
    "color_coordinate_anchor_blob_x",
    "color_coordinate_anchor_blob_y",
    "color_coordinate_anchor_output_box",
    "color_coordinate_anchor_box",
    "target_color_id",
    "host_color_id_received",
    "target_anchor_active",
    "target_anchor_color_id",
    "target_anchor_x_cm",
    "target_anchor_y_cm",
    "target_anchor_radius_cm",
    "target_anchor_sequence",
    "TARGET_ANCHOR_COMMAND",
    "TARGET_ANCHOR_FRAME_SIZE",
}

FUSION_FUNCTIONS = {
    "color_id_completed",
    "color_id_available_for_search",
    "clear_target_anchor",
    "update_target_anchor",
    "target_anchor_enabled_for",
    "target_anchor_world_distance2",
    "target_anchor_candidate_distance2",
    "target_anchor_model_rank",
    "apply_target_anchor_command",
    "clamp_int",
    "raw_model_box",
    "model_color_roi",
    "center_dist2",
    "box_from_center",
    "expand_tracking_box",
    "intersect_rois",
    "union_rois",
    "color_tracking_gate",
    "color_tracking_search_roi",
    "find_adaptive_color_blobs",
    "strict_blob_candidate",
    "pick_tracking_blob",
    "track_color_in_model_roi",
    "reset_color_coordinate_anchor",
    "translate_tracking_box",
    "anchor_color_coordinate_geometry",
    "color_blob_geometry",
    "should_run_model",
    "coordinate_box_from_track",
    "output_box_from_track",
    "smooth_tracking_box",
    "tracking_box_contact",
    "set_color_tracking",
    "tracking_world_point",
    "current_tracking_hold_result",
    "held_color_tracking_result",
    "model_geometry_tracking_result",
    "held_model_tracking_result",
}


def selected_fusion_nodes(source_path):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {
                target.id for target in node.targets
                if isinstance(target, ast.Name)
            }
            if names & FUSION_ASSIGNMENTS:
                selected.append(node)
        elif (isinstance(node, ast.FunctionDef) and
              node.name in FUSION_FUNCTIONS):
            selected.append(node)
    return selected


class ObserverInstrumentationStripper(ast.NodeTransformer):
    @staticmethod
    def _observer_name(name):
        return name.startswith("WORLD_COORD_") or name.startswith("_world_coord")

    def visit_Assign(self, node):
        names = {
            target.id for target in node.targets
            if isinstance(target, ast.Name)
        }
        if any(self._observer_name(name) for name in names):
            return None
        if (isinstance(node.value, ast.Name) and
                self._observer_name(node.value.id)):
            return None
        return self.generic_visit(node)

    def visit_FunctionDef(self, node):
        if self._observer_name(node.name):
            return None
        return self.generic_visit(node)

    def visit_Expr(self, node):
        call = node.value
        if (isinstance(call, ast.Call) and
                isinstance(call.func, ast.Name) and
                self._observer_name(call.func.id)):
            return None
        return self.generic_visit(node)


def normalized_runtime_ast(source_path, strip_observer=False):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    if strip_observer:
        tree = ObserverInstrumentationStripper().visit(tree)
        tree = ast.fix_missing_locations(tree)
    return ast.dump(tree, include_attributes=False)


def load_named_function(source_path, function_name):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == function_name
    )
    namespace = {}
    module = ast.fix_missing_locations(ast.Module(
        body=[function], type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    return namespace


def target_anchor_frame(color_id, x_mm, y_mm, radius_cm, sequence):
    x_raw = x_mm & 0xFFFF
    y_raw = y_mm & 0xFFFF
    payload = bytearray((
        0x09, color_id,
        x_raw & 0xFF, (x_raw >> 8) & 0xFF,
        y_raw & 0xFF, (y_raw >> 8) & 0xFF,
        radius_cm, sequence,
    ))
    return bytes((0xAA, 0x55)) + bytes(payload) + bytes((sum(payload) & 0xFF,))


class FakeUART:
    def __init__(self, data):
        self.data = bytearray(data)

    def any(self):
        return len(self.data)

    def read(self, count):
        chunk = bytes(self.data[:count])
        del self.data[:count]
        return chunk


def load_fusion_runtime(source_path):
    namespace = {"math": math}
    module = ast.fix_missing_locations(ast.Module(
        body=selected_fusion_nodes(source_path), type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    namespace["dynamic_detect_roi"] = (0, 8, 320, 232)
    namespace["model_lock"] = [0, (100, 80, 20, 30), -1, None, 0, 0]
    namespace["model_track"] = [
        True, 100.0, 105.0, 50.0, 60.0,
        100.0, 120.0, 100.0, 120.0, 40, 30,
    ]
    namespace["color_blob_box"] = None
    namespace["color_lost_count"] = 0
    namespace["ground_pixel_to_world"] = lambda x, y: (0.0, 100.0)
    return namespace


def contains(outer, inner):
    return (
        outer[0] <= inner[0] and
        outer[1] <= inner[1] and
        outer[0] + outer[2] >= inner[0] + inner[2] and
        outer[1] + outer[3] >= inner[1] + inner[3]
    )


class ModelBlobFusionTests(unittest.TestCase):
    def test_master_and_slave_fusion_helpers_are_identical(self):
        dumps = []
        for source_path in RUNTIME_FILES:
            module = ast.Module(
                body=selected_fusion_nodes(source_path), type_ignores=[])
            dumps.append(ast.dump(module, include_attributes=False))
        for dump in dumps[1:]:
            self.assertEqual(dumps[0], dump)

    def test_world_observer_matches_complete_main_runtime(self):
        self.assertEqual(
            normalized_runtime_ast(MAIN_FILE, strip_observer=True),
            normalized_runtime_ast(OBSERVER_FILE, strip_observer=True),
        )

    def test_motion_first_lock_policy_matches_all_entries(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            self.assertEqual(runtime["FIRST_LOCK_SCORE_MIN"], 0.30)
            self.assertEqual(runtime["FIRST_LOCK_WINDOW_FRAMES"], 5)
            self.assertEqual(runtime["FIRST_LOCK_REQUIRED_HITS"], 3)
            self.assertEqual(runtime["FIRST_LOCK_MATCH_CENTER_PX"], 36)
            self.assertEqual(runtime["FIRST_LOCK_MATCH_CENTER2"], 36 * 36)
            self.assertEqual(runtime["FIRST_LOCK_SIZE_DELTA_PERCENT"], 50)

    def test_id2_absolute_priority_matches_provincial_gate(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["all_color_thresholds"] = [None] * 5
            runtime["completed_color_mask"] = 0
            available = runtime["color_id_available_for_search"]

            self.assertTrue(runtime["ID2_ABSOLUTE_PRIORITY"])
            self.assertEqual(
                [available(color_id) for color_id in range(1, 6)],
                [False, True, False, False, False],
            )

            runtime["completed_color_mask"] = 1 << (2 - 1)
            self.assertEqual(
                [available(color_id) for color_id in range(1, 6)],
                [True, True, True, True, True],
            )

            runtime["ENABLE_COMPLETED_COLOR_EXCLUSION"] = True
            self.assertEqual(
                [available(color_id) for color_id in range(1, 6)],
                [True, False, True, True, True],
            )

            runtime["completed_color_mask"] = 0
            runtime["ID2_ABSOLUTE_PRIORITY"] = False
            self.assertEqual(
                [available(color_id) for color_id in range(1, 6)],
                [True, True, True, True, True],
            )

    def test_completed_color_exclusion_switch_preserves_records(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["all_color_thresholds"] = [None] * 5
            runtime["ID2_ABSOLUTE_PRIORITY"] = False
            runtime["completed_color_mask"] = 1 << (4 - 1)
            available = runtime["color_id_available_for_search"]

            self.assertFalse(runtime["ENABLE_COMPLETED_COLOR_EXCLUSION"])
            self.assertTrue(available(4))

            runtime["ENABLE_COMPLETED_COLOR_EXCLUSION"] = True
            self.assertFalse(available(4))
            self.assertEqual(runtime["completed_color_mask"], 1 << (4 - 1))

            runtime["ENABLE_COMPLETED_COLOR_EXCLUSION"] = False
            self.assertTrue(available(4))

    def test_target_anchor_gate_and_switch_preserve_legacy_rank(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            rank = runtime["target_anchor_model_rank"]
            self.assertEqual(rank(2, (13, 24, 1, 1), 1234), 1234)

            runtime["target_color_id"] = 2
            runtime["box_to_world"] = lambda x, y, w, h: (float(x), float(y))
            self.assertFalse(runtime["update_target_anchor"](
                2, 100, 200, 5, 7))
            self.assertEqual(
                runtime["target_anchor_candidate_distance2"](
                    2, (13, 24, 1, 1)),
                25.0,
            )
            self.assertEqual(rank(2, (13, 24, 1, 1), 1234), (-25.0, 1234))
            self.assertIsNone(rank(2, (16, 20, 1, 1), 999999))
            self.assertIsNone(rank(1, (10, 20, 1, 1), 999999))

            runtime["ENABLE_TARGET_ANCHOR_LOCK"] = False
            self.assertEqual(rank(2, (16, 20, 1, 1), 999999), 999999)

    def test_target_anchor_sequence_controls_identity_reset(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["all_color_thresholds"] = [None] * 5
            runtime["completed_color_mask"] = 0
            runtime["lost_frame_count"] = 99
            runtime["model_color"] = [0, 0, 0, False]
            resets = []
            legacy_applies = []
            runtime["reset_hybrid_tracking"] = lambda: resets.append(True)
            runtime["apply_host_hybrid_color"] = (
                lambda color_id: legacy_applies.append(color_id))

            self.assertFalse(runtime["apply_target_anchor_command"](
                2, 100, 200, 20, 40))
            self.assertEqual(len(resets), 1)
            self.assertTrue(runtime["model_color"][3])

            self.assertTrue(runtime["apply_target_anchor_command"](
                2, 130, 240, 25, 40))
            self.assertEqual(len(resets), 1)
            self.assertEqual(runtime["target_anchor_x_cm"], 13.0)
            self.assertEqual(runtime["target_anchor_y_cm"], 24.0)
            self.assertEqual(runtime["target_anchor_radius_cm"], 25.0)

            self.assertFalse(runtime["apply_target_anchor_command"](
                2, 130, 240, 25, 41))
            self.assertEqual(len(resets), 2)

            self.assertFalse(runtime["apply_target_anchor_command"](
                9, 0, 0, 25, 1))
            self.assertEqual(len(resets), 2)
            self.assertEqual(runtime["target_color_id"], 2)

            runtime["ENABLE_TARGET_ANCHOR_LOCK"] = False
            self.assertTrue(runtime["apply_target_anchor_command"](
                2, 0, 0, 25, 42))
            self.assertEqual(len(resets), 2)
            self.assertFalse(runtime["target_anchor_active"])
            self.assertEqual(legacy_applies, [])

            runtime["ID2_ABSOLUTE_PRIORITY"] = False
            self.assertFalse(runtime["apply_target_anchor_command"](
                1, 0, 0, 25, 43))
            self.assertEqual(len(resets), 2)
            self.assertEqual(legacy_applies, [1])

    def test_target_anchor_uart_frame_is_signed_and_keeps_boundaries(self):
        first = target_anchor_frame(2, -1234, 2345, 60, 19)
        second = target_anchor_frame(2, 321, -456, 40, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_named_function(source_path, "receive_command_from_host")
            calls = []
            runtime.update({
                "uart": FakeUART(first + second),
                "_cmd_rx_buf": bytearray(),
                "TARGET_ANCHOR_COMMAND": 0x09,
                "TARGET_ANCHOR_FRAME_SIZE": 11,
                "apply_target_anchor_command": (
                    lambda *values: calls.append(values)),
            })
            runtime["receive_command_from_host"]()
            self.assertEqual(calls, [(2, -1234, 2345, 60, 19)])
            self.assertEqual(len(runtime["_cmd_rx_buf"]), 11)
            runtime["receive_command_from_host"]()
            self.assertEqual(calls[-1], (2, 321, -456, 40, 20))
            self.assertEqual(runtime["_cmd_rx_buf"], bytearray())

    def test_target_anchor_uart_rejects_bad_checksum_and_resynchronizes(self):
        broken = bytearray(target_anchor_frame(2, 10, 20, 30, 1))
        broken[-1] ^= 0x01
        valid = target_anchor_frame(2, -10, -20, 30, 2)
        for source_path in RUNTIME_FILES:
            runtime = load_named_function(source_path, "receive_command_from_host")
            calls = []
            runtime.update({
                "uart": FakeUART(bytes(broken) + valid),
                "_cmd_rx_buf": bytearray(),
                "TARGET_ANCHOR_COMMAND": 0x09,
                "TARGET_ANCHOR_FRAME_SIZE": 11,
                "apply_target_anchor_command": (
                    lambda *values: calls.append(values)),
            })
            runtime["receive_command_from_host"]()
            self.assertEqual(calls, [(2, -10, -20, 30, 2)])
            self.assertEqual(runtime["_cmd_rx_buf"], bytearray())

    def test_fresh_model_search_uses_inset_model_color_roi(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            roi = runtime["color_tracking_search_roi"](True)
            model_box = runtime["model_lock"][1]
            self.assertEqual(runtime["COLOR_ROI_INSET_X_PERCENT"], 5)
            self.assertEqual(runtime["COLOR_ROI_INSET_TOP_PERCENT"], 5)
            self.assertEqual(runtime["COLOR_ROI_INSET_BOTTOM_PERCENT"], 10)
            self.assertEqual(roi, runtime["model_color_roi"](model_box))
            self.assertTrue(contains(model_box, roi))
            self.assertGreater(roi[0], model_box[0])
            self.assertGreater(roi[1], model_box[1])
            self.assertLess(roi[2], model_box[2])
            self.assertLess(roi[3], model_box[3])

    def test_model_refresh_restricts_old_blob_to_fresh_model_roi(self):
        full_blob = (78, 70, 60, 65)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_blob_box"] = full_blob
            refresh_roi = runtime["color_tracking_search_roi"](True)
            tracking_roi = runtime["color_tracking_search_roi"](False)
            self.assertEqual(
                refresh_roi,
                runtime["model_color_roi"](runtime["model_lock"][1]),
            )
            self.assertFalse(contains(refresh_roi, full_blob))
            self.assertTrue(contains(tracking_roi, full_blob))

    def test_model_refresh_does_not_rank_against_stale_blob(self):
        stale_blob = (78, 70, 60, 65)
        picked_blob = (101, 81, 18, 26)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            references = []
            runtime["color_blob_box"] = stale_blob
            runtime["color_tracking_search_roi"] = lambda observed: (0, 0, 1, 1)
            runtime["color_tracking_gate"] = lambda observed: (0, 0, 1, 1)
            runtime["find_adaptive_color_blobs"] = (
                lambda img, roi, color_id: ([object()], 1))
            runtime["pick_tracking_blob"] = (
                lambda blobs, color_id, reference, gate, minimum:
                references.append(reference) or picked_blob)

            self.assertEqual(runtime["track_color_in_model_roi"](
                object(), 2, model_observed=True), picked_blob)
            self.assertIsNone(references[-1])
            self.assertEqual(runtime["track_color_in_model_roi"](
                object(), 2, model_observed=False), picked_blob)
            self.assertEqual(references[-1], picked_blob)

    def test_blob_size_changes_do_not_move_coordinate_geometry(self):
        initial_blob = (80, 80, 40, 40)
        resized_blob = (60, 60, 80, 80)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            expected_output = runtime["output_box_from_track"]()
            expected_coordinate = runtime["coordinate_box_from_track"]()

            initial = runtime["color_blob_geometry"](initial_blob)
            resized = runtime["color_blob_geometry"](resized_blob)

            self.assertEqual(initial, (expected_output, expected_coordinate))
            self.assertEqual(resized, (expected_output, expected_coordinate))

    def test_blob_center_translation_moves_coordinate_geometry_immediately(self):
        initial_blob = (80, 80, 40, 40)
        moved_blob = (87, 74, 40, 40)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            anchor_output = runtime["output_box_from_track"]()
            anchor_coordinate = runtime["coordinate_box_from_track"]()
            runtime["color_blob_geometry"](initial_blob)

            output_box, coordinate_box = runtime["color_blob_geometry"](
                moved_blob)

            self.assertEqual(
                output_box,
                runtime["translate_tracking_box"](
                    anchor_output, 7.0, -6.0),
            )
            self.assertEqual(
                coordinate_box,
                runtime["translate_tracking_box"](
                    anchor_coordinate, 7.0, -6.0),
            )

    def test_model_refresh_reanchors_coordinate_to_latest_contact(self):
        initial_blob = (80, 80, 40, 40)
        moved_blob = (87, 74, 40, 40)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_blob_geometry"](initial_blob)
            translated = runtime["color_blob_geometry"](moved_blob)[1]

            runtime["model_track"] = [
                True, 140.0, 125.0, 30.0, 50.0,
                140.0, 150.0, 140.0, 150.0, 30, 50,
            ]
            expected_output = runtime["output_box_from_track"]()
            expected_coordinate = runtime["coordinate_box_from_track"]()
            refreshed = runtime["color_blob_geometry"](
                moved_blob, model_observed=True)

            self.assertNotEqual(translated, expected_coordinate)
            self.assertEqual(refreshed,
                             (expected_output, expected_coordinate))
            self.assertEqual(runtime["color_coordinate_anchor_blob_x"],
                             107.0)
            self.assertEqual(runtime["color_coordinate_anchor_blob_y"],
                             94.0)

    def test_display_box_keeps_its_existing_smoothing(self):
        previous = (100, 100, 20, 20)
        current = (104, 102, 30, 24)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            self.assertEqual(runtime["OUTPUT_SMOOTH_ALPHA_X100"], 70)
            smoothed = runtime["smooth_tracking_box"](previous, current)
            self.assertNotEqual(smoothed, previous)
            self.assertNotEqual(smoothed, current)
            self.assertGreater(smoothed[2], previous[2])
            self.assertLess(smoothed[2], current[2])

    def test_locked_model_candidate_updates_on_first_observation(self):
        anchor = (100, 100, 40, 40)
        candidate_box = (100, 126, 40, 40)
        for source_path in RUNTIME_FILES:
            runtime = load_named_function(
                source_path, "accept_model_candidate")
            observed = []
            runtime.update({
                "model_lock": [2, anchor, -1, None, 0, 0],
                "model_last_score": 0.90,
                "accept_first_lock_candidate": lambda candidate: False,
                "observe_model_box": (
                    lambda label, box: observed.append((label, box)) or True),
            })

            candidate = (2, candidate_box, 0.21, None, None, None)
            self.assertTrue(runtime["accept_model_candidate"](candidate))
            self.assertEqual(observed, [(2, candidate_box)])
            self.assertEqual(runtime["model_lock"][1], candidate_box)
            self.assertEqual(runtime["model_last_score"], 0.21)

    def test_coordinate_box_updates_without_temporal_filter(self):
        first = (100, 100, 20, 20)
        second = (140, 80, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, first, first)
            output_box, coordinate_box = runtime["set_color_tracking"](
                2, second, second)

            self.assertEqual(output_box, second)
            self.assertEqual(coordinate_box, second)
            self.assertEqual(runtime["color_track_coordinate_box"], second)

    def test_tracking_world_point_projects_current_coordinate_directly(self):
        coordinate_box = (100, 70, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            calls = []
            runtime["box_to_world"] = (
                lambda x, y, w, h: calls.append((x, y, w, h)) or
                (7.0, 8.0))

            self.assertEqual(runtime["tracking_world_point"](
                2, coordinate_box), (7.0, 8.0))
            self.assertEqual(calls, [coordinate_box])

    def test_large_coordinate_move_is_visible_on_first_frame(self):
        previous = (283, 210, 20, 20)
        current = (283, 171, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            _, coordinate_box = runtime["set_color_tracking"](
                2, current, current)
            runtime["box_to_world"] = (
                lambda x, y, w, h: (x + w * 0.5, y + h))

            self.assertEqual(coordinate_box, current)
            self.assertEqual(runtime["tracking_world_point"](
                2, coordinate_box), (293.0, 191))

    def test_world_x_follows_each_fresh_mesh_measurement(self):
        initial = (100, 100, 20, 20)
        moving_boxes = (
            (104, 100, 20, 20),
            (109, 100, 20, 20),
            (113, 100, 20, 20),
            (118, 100, 20, 20),
        )
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, initial, initial)

            for moving_box in moving_boxes:
                runtime["set_color_tracking"](2, moving_box, moving_box)
                raw_x = runtime["tracking_box_contact"](moving_box)[0]
                runtime["box_to_world"] = (
                    lambda x, y, w, h: (x + w * 0.5, y + h - 0.5))
                world_x, _ = runtime["tracking_world_point"](
                    2, moving_box)
                self.assertEqual(world_x, raw_x)

    def test_held_frame_preserves_last_coordinate_box(self):
        previous = (100, 100, 20, 20)
        current = (100, 80, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            runtime["set_color_tracking"](2, current, current)
            expected_coordinate = runtime["smooth_tracking_box"](
                previous, current)
            runtime["color_lost_count"] = 1

            held = runtime["held_color_tracking_result"](2)
            self.assertIsNotNone(held)
            self.assertEqual(held[2], expected_coordinate)
            self.assertEqual(runtime["color_track_coordinate_box"],
                             expected_coordinate)

    def test_new_color_uses_its_coordinate_immediately(self):
        previous = (100, 100, 20, 20)
        new_target = (100, 60, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            output_box, coordinate_box = runtime["set_color_tracking"](
                3, new_target, new_target)

            self.assertEqual(output_box, new_target)
            self.assertEqual(coordinate_box, new_target)

    def test_fresh_model_geometry_precedes_held_color_coordinates(self):
        for source_path in RUNTIME_FILES:
            runtime = load_named_function(
                source_path, "process_model_only_target")
            calls = []
            model_result = object()
            held_result = object()
            runtime.update({
                "model_runtime_enabled": True,
                "openart_mode": 1,
                "MODE_SEARCH": 1,
                "MODE_CARRY": 2,
                "run_model_best": lambda img: object(),
                "model_lock": [2, (100, 100, 20, 20), -1, None, 0, 0],
                "accept_model_candidate": lambda candidate: True,
                "host_forced_target_active": lambda: True,
                "trusted_model_color_id": lambda label: 2,
                "color_id_to_model_label": lambda color_id: 2,
                "model_track": [True],
                "adaptive_color_thresholds": [None, object()],
                "track_color_in_model_roi": (
                    lambda img, color_id, observed: None),
                "model_geometry_tracking_result": (
                    lambda color_id: calls.append("model") or model_result),
                "held_color_tracking_result": (
                    lambda color_id: calls.append("held") or held_result),
            })

            result = runtime["process_model_only_target"](
                object(), 10, True)
            self.assertIs(result, model_result)
            self.assertEqual(calls, ["model"])

    def test_low_score_near_model_geometry_follows_fresh_coordinate(self):
        previous = (100, 100, 20, 20)
        current = (100, 80, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["ground_pixel_to_world"] = lambda x, y: (0.0, 13.5)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            runtime["model_last_score"] = 0.59
            runtime["output_box_from_track"] = lambda: current
            runtime["coordinate_box_from_track"] = lambda: current

            result = runtime["model_geometry_tracking_result"](2)
            expected = runtime["smooth_tracking_box"](previous, current)
            self.assertEqual(result, (2, expected, expected, 3))

    def test_model_geometry_refresh_uses_provincial_single_stage_follow(self):
        previous = (100, 100, 20, 20)
        current = (100, 80, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["ground_pixel_to_world"] = lambda x, y: (0.0, y / 10.0)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            runtime["model_last_score"] = 0.90
            runtime["output_box_from_track"] = lambda: current
            runtime["coordinate_box_from_track"] = lambda: current
            runtime["color_coordinate_anchor_blob_x"] = 110.0
            runtime["color_coordinate_anchor_blob_y"] = 110.0
            runtime["color_coordinate_anchor_box"] = previous

            result = runtime["model_geometry_tracking_result"](2)
            expected_coordinate = runtime["smooth_tracking_box"](
                previous, current)

            self.assertEqual(result[2], expected_coordinate)
            self.assertEqual(runtime["color_track_coordinate_box"],
                             expected_coordinate)
            self.assertIsNone(runtime["color_coordinate_anchor_box"])

    def test_legacy_coordinate_filter_paths_are_removed(self):
        removed = (
            "COORDINATE_WORLD_Y_ALPHA_X100",
            "COORDINATE_FILTER_ENTER_Y_CM",
            "COORDINATE_FILTER_EXIT_Y_CM",
            "COORDINATE_SMOOTH_ALPHA_X100",
            "COORDINATE_NEAR_ALPHA_X100",
            "COORDINATE_NEAR_JUMP_CONFIRM_FRAMES",
            "COORDINATE_CONTACT_DEADBAND_PX",
            "COORDINATE_JUMP_GATE_Y_CM",
            "COORDINATE_JUMP_THRESHOLD_PX",
            "COORDINATE_JUMP_CONFIRM_PX",
            "color_track_raw_coordinate_box",
            "color_track_world_y",
            "color_track_world_y_filter_active",
            "color_track_near_jump_box",
            "stabilize_coordinate_box",
            "MODEL_UPDATE_SCORE_MIN",
            "MODEL_UPDATE_CONFIRM_FRAMES",
            "MODEL_UPDATE_DIRECT_CENTER_PX",
            "MODEL_UPDATE_DIRECT_BOTTOM_PX",
            "MODEL_UPDATE_MATCH_CENTER_PX",
            "MODEL_UPDATE_MATCH_BOTTOM_PX",
            "MODEL_UPDATE_SIZE_DELTA_PERCENT",
            "MODEL_GEOMETRY_NEAR_SCORE_MIN",
            "MODEL_GEOMETRY_NEAR_DISTANCE_CM",
            "locked_model_candidate_ready",
            "model_update_box_matches",
            "model_box_size_matches",
            "coordinate_box_world_y",
            "WORLD_COORD_SD_LOG",
            "_world_coord_log",
            "id2_coordinate_watchdog",
        )
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            for name in removed:
                self.assertNotIn(name, source)

    def test_tracking_world_point_does_not_depend_on_track_state(self):
        coordinate_box = (100, 80, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_color_id"] = 0
            runtime["box_to_world"] = lambda x, y, w, h: (7.0, 8.0)

            self.assertEqual(runtime["tracking_world_point"](
                2, coordinate_box), (7.0, 8.0))

    def test_display_and_coordinate_use_same_provincial_follow(self):
        previous = (100, 100, 20, 20)
        current = (110, 100, 20, 20)
        previous_coordinate = (80, 90, 40, 30)
        current_coordinate = (90, 84, 40, 30)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](
                2, previous, previous_coordinate)

            output_box, coordinate_box = runtime["set_color_tracking"](
                2, current, current_coordinate)
            expected_output = runtime["smooth_tracking_box"](
                previous, current)
            expected_coordinate = runtime["smooth_tracking_box"](
                previous_coordinate, current_coordinate)
            self.assertEqual(output_box, expected_output)
            self.assertEqual(coordinate_box, expected_coordinate)
            self.assertNotEqual(coordinate_box, current_coordinate)
            self.assertEqual(output_box[2:], current[2:])

    def test_coordinate_uses_only_provincial_single_stage_follow(self):
        raw_boxes = [
            (100, 108, 20, 20),
            (100, 99, 20, 20),
            (100, 107, 20, 20),
            (100, 100, 20, 20),
        ]
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            initial = (100, 100, 20, 20)
            runtime["set_color_tracking"](2, initial, initial)
            coordinate_boxes = []
            expected_boxes = []
            previous_coordinate = initial
            for raw_box in raw_boxes:
                _, coordinate_box = runtime["set_color_tracking"](
                    2, raw_box, raw_box)
                coordinate_boxes.append(coordinate_box)
                previous_coordinate = runtime["smooth_tracking_box"](
                    previous_coordinate, raw_box)
                expected_boxes.append(previous_coordinate)
            self.assertEqual(coordinate_boxes, expected_boxes)

    def test_coordinate_anchor_is_cleared_when_model_geometry_is_unavailable(self):
        blob = (80, 80, 40, 40)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            self.assertIsNotNone(runtime["color_blob_geometry"](blob))
            runtime["model_track"][0] = False

            self.assertIsNone(runtime["color_blob_geometry"](
                blob, model_observed=True))
            self.assertIsNone(runtime["color_coordinate_anchor_blob_x"])
            self.assertIsNone(runtime["color_coordinate_anchor_blob_y"])
            self.assertIsNone(
                runtime["color_coordinate_anchor_output_box"])
            self.assertIsNone(runtime["color_coordinate_anchor_box"])

    def test_provincial_center_anchor_uses_model_owned_output_geometry(self):
        removed = (
            "anchor_model_geometry",
            "fused_tracking_boxes",
            "color_anchor_blob_box",
            "color_anchor_output_box",
            "color_anchor_coordinate_box",
        )
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            for name in removed:
                self.assertNotIn(name, source)
            self.assertIn(
                "reference = None if model_observed else color_blob_box",
                source,
            )
            self.assertIn("def model_color_roi", source)
            self.assertIn("color_coordinate_anchor_blob_x", source)
            self.assertIn(
                "color_coordinate_anchor_output_box = output_box_from_track()",
                source,
            )
            self.assertIn("return (translate_tracking_box(", source)


if __name__ == "__main__":
    unittest.main()
