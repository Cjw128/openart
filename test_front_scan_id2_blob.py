import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (ROOT / "main.py", ROOT / "minimain.py")

SELECTED_ASSIGNMENTS = {
    "FRONT_SCAN_ID2_COLOR_ID",
    "FRONT_SCAN_ID2_MIN_PIXELS",
    "FRONT_SCAN_ID2_MIN_AREA",
    "FRONT_SCAN_ID2_MIN_DENSITY",
    "FRONT_SCAN_ID2_ASPECT_MIN_X100",
    "FRONT_SCAN_ID2_ASPECT_MAX_X100",
}

SELECTED_FUNCTIONS = {
    "front_scan_id2_blob_valid",
    "front_scan_find_id2_blobs",
    "front_scan_has_id2_blob",
    "scan_front_other_color_ids",
}


def selected_nodes(source_path):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    nodes = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {
                target.id for target in node.targets
                if isinstance(target, ast.Name)
            }
            if names & SELECTED_ASSIGNMENTS:
                nodes.append(node)
        elif (isinstance(node, ast.FunctionDef) and
              node.name in SELECTED_FUNCTIONS):
            nodes.append(node)
    return nodes


def load_runtime(source_path):
    namespace = {}
    module = ast.fix_missing_locations(ast.Module(
        body=selected_nodes(source_path), type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    namespace.update({
        "adaptive_color_thresholds": [None] * 5,
        "all_color_thresholds": ["id1", "base-id2", "id3", "id4", "id5"],
        "ENABLE_DYNAMIC_CUT": False,
        "dynamic_cut_valid": False,
        "dynamic_cut_left_y": 8,
        "CUT_BLOB_DELTA": 2,
        "front_scan_box_is_current": lambda box, current: box == current,
    })
    return namespace


class FakeBlob:
    def __init__(self, box, pixels=None, density=0.8):
        self.box = box
        self.pixel_count = pixels if pixels is not None else box[2] * box[3]
        self.fill_density = density

    def x(self):
        return self.box[0]

    def y(self):
        return self.box[1]

    def w(self):
        return self.box[2]

    def h(self):
        return self.box[3]

    def pixels(self):
        return self.pixel_count

    def density(self):
        return self.fill_density


class FakeImage:
    def __init__(self, results):
        self.results = results
        self.threshold_calls = []

    def find_blobs(self, thresholds, **kwargs):
        threshold = thresholds[0]
        self.threshold_calls.append((threshold, kwargs))
        return self.results.get(threshold, [])


class FrontScanId2BlobTests(unittest.TestCase):
    def test_helpers_are_identical_in_both_openart_builds(self):
        dumps = [ast.dump(ast.Module(
            body=selected_nodes(path), type_ignores=[]),
            include_attributes=False) for path in RUNTIME_FILES]
        self.assertEqual(dumps[0], dumps[1])

    def test_dynamic_threshold_is_used_without_base_rescan(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["adaptive_color_thresholds"][1] = "dynamic-id2"
            brick = FakeBlob((80, 60, 60, 15))
            image = FakeImage({"dynamic-id2": [brick]})

            found = runtime["front_scan_has_id2_blob"](
                image, (0, 8, 320, 142), None)

            self.assertTrue(found)
            self.assertEqual(
                [call[0] for call in image.threshold_calls],
                ["dynamic-id2"],
            )

    def test_base_threshold_is_fallback_after_dynamic_miss(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["adaptive_color_thresholds"][1] = "dynamic-id2"
            brick = FakeBlob((80, 60, 60, 15))
            image = FakeImage({"dynamic-id2": [], "base-id2": [brick]})

            found = runtime["front_scan_has_id2_blob"](
                image, (0, 8, 320, 142), None)

            self.assertTrue(found)
            self.assertEqual(
                [call[0] for call in image.threshold_calls],
                ["dynamic-id2", "base-id2"],
            )

    def test_horizontal_brick_passes_but_noise_and_lines_do_not(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            valid = runtime["front_scan_id2_blob_valid"]

            self.assertTrue(valid(FakeBlob((80, 60, 60, 15)), None))
            self.assertFalse(valid(
                FakeBlob((80, 60, 80, 10), density=0.8), None))
            self.assertFalse(valid(
                FakeBlob((80, 60, 30, 10), pixels=60), None))
            self.assertFalse(valid(
                FakeBlob((80, 60, 30, 10), density=0.3), None))

    def test_current_target_blob_is_excluded(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            box = (80, 60, 60, 15)
            self.assertFalse(runtime["front_scan_id2_blob_valid"](
                FakeBlob(box), box))

    def test_id2_blob_is_reported_when_model_is_unavailable(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            brick = FakeBlob((80, 60, 60, 15))
            image = FakeImage({"base-id2": [brick]})
            runtime.update({
                "front_scan_current_target": lambda: (None, 4),
                "front_scan_roi": lambda: (0, 8, 320, 142),
                "model_runtime_enabled": False,
                "model_net": None,
                "model_infer_error_count": 0,
                "model_last_frame": -1,
            })

            self.assertEqual(
                runtime["scan_front_other_color_ids"](image),
                (4, 1 << (2 - 1), 1),
            )


if __name__ == "__main__":
    unittest.main()
