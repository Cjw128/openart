import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (ROOT / "main.py", ROOT / "minimain.py")

SELECTED_ASSIGNMENTS = {
    "FRONT_SCAN_BEAR_COMPONENT_MIN_PIXELS",
    "FRONT_SCAN_BEAR_MIN_PIXELS",
    "FRONT_SCAN_BEAR_MIN_PIXEL_GAP",
    "FRONT_SCAN_BEAR_DOMINANCE_X100",
}

SELECTED_FUNCTIONS = {
    "sample_model_color",
    "front_scan_bear_threshold_pixels",
    "front_scan_bear_color_id",
    "front_scan_color_id",
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
        "MODEL_COLOR_IDS": ((4, 5), (3,), (1, 2)),
        "all_color_thresholds": [
            "id1", "id2", "id3", "brown", "white"],
        "BRN_BEAR_MERGE_MARGIN": 12,
        "WHT_BEAR_MERGE_MARGIN": 10,
        "COLOR_SAMPLE_MAX_IQR": (50, 55, 65),
        "HOST_FORCED_COLOR_SAMPLE_MAX_IQR": (65, 70, 85),
        "target_color_id": 0,
        "host_forced_target_active": lambda: False,
        "color_id_available_for_search": lambda color_id: 1 <= color_id <= 5,
        "model_sample_roi": lambda label, box: box,
        "sample_box_lab_stats": lambda img, label, box, limit: None,
        "sample_color_id_from_stats": (
            lambda label, sample, forced_color_id=0: 0),
        "build_dynamic_threshold": (
            lambda color_id, sample: ("dynamic", color_id)),
    })
    return namespace


class FakeBlob:
    def __init__(self, pixels):
        self.pixel_count = pixels

    def pixels(self):
        return self.pixel_count


class FakeImage:
    def __init__(self, pixels_by_threshold):
        self.pixels_by_threshold = pixels_by_threshold
        self.calls = []

    def find_blobs(self, thresholds, **kwargs):
        threshold = thresholds[0]
        self.calls.append((threshold, kwargs))
        pixels = self.pixels_by_threshold.get(threshold, 0)
        return [] if pixels <= 0 else [FakeBlob(pixels)]


class FrontScanBearColorTests(unittest.TestCase):
    def test_helpers_are_identical_in_both_openart_builds(self):
        dumps = [ast.dump(ast.Module(
            body=selected_nodes(path), type_ignores=[]),
            include_attributes=False) for path in RUNTIME_FILES]
        self.assertEqual(dumps[0], dumps[1])

    def test_brown_and_white_use_threshold_pixel_dominance(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            classify = runtime["front_scan_color_id"]
            box = (80, 40, 60, 60)

            brown = FakeImage({"brown": 80, "white": 20})
            white = FakeImage({"brown": 18, "white": 90})

            self.assertEqual(classify(brown, 0, box), 4)
            self.assertEqual(classify(white, 0, box), 5)
            self.assertEqual(
                [call[0] for call in brown.calls], ["brown", "white"])

    def test_ambiguous_bear_is_not_forced_to_an_id(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            classify = runtime["front_scan_color_id"]
            box = (80, 40, 60, 60)

            self.assertEqual(classify(
                FakeImage({"brown": 50, "white": 45}), 0, box), 0)
            self.assertEqual(classify(
                FakeImage({"brown": 10, "white": 4}), 0, box), 0)

    def test_regular_bear_lock_uses_pixel_dominance(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["sample_box_lab_stats"] = (
                lambda img, label, box, limit: "background-biased")
            runtime["sample_color_id_from_stats"] = (
                lambda label, sample, forced_color_id=0: 5)

            result = runtime["sample_model_color"](
                FakeImage({"brown": 80, "white": 20}),
                0, (80, 40, 60, 60))

            self.assertEqual(result, (4, "brown"))

    def test_regular_bear_lock_keeps_matching_dynamic_threshold(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["sample_box_lab_stats"] = (
                lambda img, label, box, limit: "brown-sample")
            runtime["sample_color_id_from_stats"] = (
                lambda label, sample, forced_color_id=0: 4)

            result = runtime["sample_model_color"](
                FakeImage({"brown": 80, "white": 20}),
                0, (80, 40, 60, 60))

            self.assertEqual(result, (4, ("dynamic", 4)))

    def test_regular_bear_lock_keeps_ambiguous_pixels_unknown(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)

            result = runtime["sample_model_color"](
                FakeImage({"brown": 50, "white": 45}),
                0, (80, 40, 60, 60))

            self.assertEqual(result, (0, None))

    def test_forced_bear_id_must_win_pixel_competition(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["target_color_id"] = 4
            runtime["host_forced_target_active"] = lambda: True

            brown = runtime["sample_model_color"](
                FakeImage({"brown": 80, "white": 20}),
                0, (80, 40, 60, 60))
            white = runtime["sample_model_color"](
                FakeImage({"brown": 18, "white": 90}),
                0, (80, 40, 60, 60))

            self.assertEqual(brown, (4, "brown"))
            self.assertEqual(white, (0, None))

    def test_other_model_classes_keep_existing_color_path(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            classify = runtime["front_scan_color_id"]
            runtime["sample_box_lab_stats"] = (
                lambda img, label, box, limit: "sample")
            runtime["sample_color_id_from_stats"] = (
                lambda label, sample: 2)

            self.assertEqual(classify(FakeImage({}), 1, (0, 0, 10, 10)), 3)
            self.assertEqual(classify(FakeImage({}), 2, (0, 0, 10, 10)), 2)

    def test_threshold_read_failure_is_ambiguous(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)

            class BrokenImage:
                def find_blobs(self, thresholds, **kwargs):
                    raise RuntimeError("camera failure")

            self.assertEqual(runtime["front_scan_color_id"](
                BrokenImage(), 0, (0, 0, 20, 20)), 0)


if __name__ == "__main__":
    unittest.main()
