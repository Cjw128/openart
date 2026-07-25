import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (ROOT / "main.py", ROOT / "minimain.py")

FUSION_ASSIGNMENTS = {
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
}

FUSION_FUNCTIONS = {
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


def load_fusion_runtime(source_path):
    namespace = {}
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
        self.assertEqual(dumps[0], dumps[1])

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
            smoothed = runtime["smooth_tracking_box"](previous, current)
            self.assertNotEqual(smoothed, previous)
            self.assertNotEqual(smoothed, current)
            self.assertGreater(smoothed[2], previous[2])
            self.assertLess(smoothed[2], current[2])

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
