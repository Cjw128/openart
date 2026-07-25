import ast
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
MAIN_FILE = ROOT / "main.py"
OBSERVER_FILE = ROOT / "world_coordinate_test.py"
RUNTIME_FILES = (MAIN_FILE, ROOT / "minimain.py", OBSERVER_FILE)

FUSION_ASSIGNMENTS = {
    "ID2_ABSOLUTE_PRIORITY",
    "MODEL_HOLD_FRAMES",
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
    "COORDINATE_SMOOTH_ALPHA_X100",
    "COORDINATE_CONTACT_DEADBAND_PX",
    "COORDINATE_CONTACT_DEADBAND2",
    "COORDINATE_CONTACT_RESET_PX",
    "COORDINATE_CONTACT_RESET2",
}

FUSION_FUNCTIONS = {
    "color_id_completed",
    "color_id_available_for_search",
    "clamp_int",
    "raw_model_box",
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
    "color_blob_geometry",
    "should_run_model",
    "smooth_tracking_box",
    "tracking_box_contact",
    "coordinate_box_with_contact",
    "stabilize_coordinate_box",
    "set_color_tracking",
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


def load_fusion_runtime(source_path):
    namespace = {"math": math}
    module = ast.fix_missing_locations(ast.Module(
        body=selected_fusion_nodes(source_path), type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    namespace["dynamic_detect_roi"] = (0, 8, 320, 232)
    namespace["model_lock"] = [0, (100, 80, 20, 30), -1, None, 0, 0]
    namespace["color_blob_box"] = None
    namespace["color_lost_count"] = 0
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
            normalized_runtime_ast(MAIN_FILE),
            normalized_runtime_ast(OBSERVER_FILE, strip_observer=True),
        )

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
                [True, False, True, True, True],
            )

            runtime["completed_color_mask"] = 0
            runtime["ID2_ABSOLUTE_PRIORITY"] = False
            self.assertEqual(
                [available(color_id) for color_id in range(1, 6)],
                [True, True, True, True, True],
            )

    def test_first_blob_search_expands_incomplete_model_box(self):
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            roi = runtime["color_tracking_search_roi"](True)
            model_box = runtime["model_lock"][1]
            self.assertTrue(contains(roi, model_box))
            self.assertLess(roi[0], model_box[0])
            self.assertLess(roi[1], model_box[1])
            self.assertGreater(roi[2], model_box[2])
            self.assertGreater(roi[3], model_box[3])

    def test_model_refresh_keeps_existing_blob_inside_search_roi(self):
        full_blob = (78, 70, 60, 65)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_blob_box"] = full_blob
            refresh_roi = runtime["color_tracking_search_roi"](True)
            tracking_roi = runtime["color_tracking_search_roi"](False)
            self.assertEqual(refresh_roi, tracking_roi)
            self.assertTrue(contains(refresh_roi, full_blob))

    def test_blob_owns_output_and_coordinate_geometry(self):
        blob = (78, 70, 60, 65)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            output_box, coordinate_box = runtime["color_blob_geometry"](blob)
            self.assertEqual(output_box, blob)
            self.assertEqual(coordinate_box, blob)

    def test_blob_geometry_uses_stable_current_weight(self):
        previous = (100, 100, 20, 20)
        current = (104, 102, 30, 24)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            self.assertEqual(runtime["OUTPUT_SMOOTH_ALPHA_X100"], 35)
            self.assertEqual(runtime["COORDINATE_SMOOTH_ALPHA_X100"], 50)
            smoothed = runtime["smooth_tracking_box"](previous, current)
            coordinate_smoothed = runtime["smooth_tracking_box"](
                previous, current,
                runtime["COORDINATE_SMOOTH_ALPHA_X100"])
            self.assertNotEqual(smoothed, previous)
            self.assertNotEqual(smoothed, current)
            self.assertGreater(smoothed[2], previous[2])
            self.assertLess(smoothed[2], current[2])
            self.assertGreater(coordinate_smoothed[2], smoothed[2])
            self.assertLessEqual(coordinate_smoothed[2], current[2])

    def test_coordinate_contact_uses_spatial_deadband_and_jump_reset(self):
        previous = (100, 100, 20, 20)
        small_jitter = (101, 100, 20, 20)
        ordinary_move = (110, 100, 20, 20)
        large_jump = (118, 100, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            stabilize = runtime["stabilize_coordinate_box"]
            contact = runtime["tracking_box_contact"]

            self.assertEqual(runtime["COORDINATE_CONTACT_DEADBAND_PX"], 2.0)
            self.assertEqual(runtime["COORDINATE_CONTACT_RESET_PX"], 18.0)
            self.assertEqual(stabilize(previous, small_jitter), previous)

            stable = stabilize(previous, ordinary_move)
            previous_x, _ = contact(previous)
            current_x, _ = contact(ordinary_move)
            stable_x, _ = contact(stable)
            self.assertGreater(stable_x, previous_x)
            self.assertLess(stable_x, current_x)
            self.assertAlmostEqual(current_x - stable_x, 2.0, delta=0.5)

            self.assertEqual(stabilize(previous, large_jump), large_jump)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_raw_coordinate_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)
            _, stable_jump = runtime["set_color_tracking"](
                2, large_jump, large_jump)
            self.assertEqual(stable_jump, large_jump)
            self.assertEqual(runtime["color_track_raw_coordinate_box"],
                             large_jump)
            _, next_stable = runtime["set_color_tracking"](
                2, large_jump, large_jump)
            self.assertEqual(next_stable, large_jump)

    def test_coordinate_filter_leads_display_during_fast_approach(self):
        previous = (100, 100, 20, 20)
        approach = (100, 117, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            contact = runtime["tracking_box_contact"]
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_raw_coordinate_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)

            output_box, coordinate_box = runtime["set_color_tracking"](
                2, approach, approach)
            expected_raw_coordinate = runtime["smooth_tracking_box"](
                previous, approach,
                runtime["COORDINATE_SMOOTH_ALPHA_X100"])
            self.assertEqual(runtime["color_track_raw_coordinate_box"],
                             expected_raw_coordinate)
            previous_y = contact(previous)[1]
            output_travel = contact(output_box)[1] - previous_y
            coordinate_travel = contact(coordinate_box)[1] - previous_y
            self.assertGreater(coordinate_travel, output_travel)
            self.assertNotEqual(coordinate_box, approach)

    def test_display_box_stays_separate_from_stable_coordinate_box(self):
        previous = (100, 100, 20, 20)
        current = (110, 100, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_raw_coordinate_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, previous, previous)

            output_box, coordinate_box = runtime["set_color_tracking"](
                2, current, current)
            expected_output = runtime["smooth_tracking_box"](
                previous, current)
            self.assertEqual(output_box, expected_output)
            self.assertNotEqual(coordinate_box, output_box)
            self.assertEqual(output_box[2:], current[2:])

    def test_near_range_bottom_edge_noise_is_reduced(self):
        raw_boxes = [
            (100, 108, 20, 20),
            (100, 99, 20, 20),
            (100, 107, 20, 20),
            (100, 100, 20, 20),
        ]
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            contact = runtime["tracking_box_contact"]
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_raw_coordinate_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            initial = (100, 100, 20, 20)
            runtime["set_color_tracking"](2, initial, initial)
            raw_y = []
            stable_y = [contact(initial)[1]]
            for raw_box in raw_boxes:
                raw_y.append(contact(raw_box)[1])
                _, stable_box = runtime["set_color_tracking"](
                    2, raw_box, raw_box)
                stable_y.append(contact(stable_box)[1])
            self.assertLess(max(stable_y) - min(stable_y),
                            max(raw_y) - min(raw_y))

    def test_stable_contact_converges_after_real_motion_stops(self):
        initial = (100, 100, 20, 20)
        moved = (100, 108, 20, 20)
        for source_path in RUNTIME_FILES:
            runtime = load_fusion_runtime(source_path)
            contact = runtime["tracking_box_contact"]
            runtime["color_track_active"] = False
            runtime["color_track_box"] = None
            runtime["color_track_raw_coordinate_box"] = None
            runtime["color_track_coordinate_box"] = None
            runtime["color_track_color_id"] = 0
            runtime["set_color_tracking"](2, initial, initial)
            stable_box = initial
            for _ in range(12):
                _, stable_box = runtime["set_color_tracking"](
                    2, moved, moved)
            raw_y = contact(moved)[1]
            stable_y = contact(stable_box)[1]
            self.assertLessEqual(abs(raw_y - stable_y),
                                 runtime["COORDINATE_CONTACT_DEADBAND_PX"])

    def test_old_model_anchor_path_is_removed(self):
        removed = (
            "anchor_model_geometry",
            "fused_tracking_boxes",
            "color_anchor_blob_box",
            "model_color_roi",
        )
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            for name in removed:
                self.assertNotIn(name, source)
            self.assertIn("reference = color_blob_box", source)


if __name__ == "__main__":
    unittest.main()
