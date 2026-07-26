import ast
import csv
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (
    ROOT / "main.py",
    ROOT / "minimain.py",
    ROOT / "world_coordinate_test.py",
)
MESH_PATH = ROOT / "camera_ground_mesh.txt"
POINTS_PATH = ROOT / "ground_mesh_24_points_template.csv"

RUNTIME_ASSIGNMENTS = {
    "WORLD_X_LIMIT_CM",
    "WORLD_Y_MAX_CM",
    "CAMERA_GROUND_MESH_PATH",
    "CAMERA_GROUND_MESH_ROLE",
    "GROUND_IMAGE_W",
    "GROUND_IMAGE_H",
    "GROUND_TRIANGLE_EPSILON",
    "GROUND_HOMOGRAPHY_EPSILON",
    "GROUND_OUTSIDE_DEADBAND_PX",
    "GROUND_REQUIRED_NEAR_Y_CM",
    "GROUND_CENTER_X_ON_IMAGE",
    "MESH_INVALID",
    "MESH_VALID",
    "MESH_TOO_NEAR",
    "MESH_TOO_FAR",
    "MESH_LEFT",
    "MESH_RIGHT",
    "BOUNDARY_TO_STATUS",
    "DEFAULT_GROUND_FALLBACK_H",
    "ground_mesh_triangles",
    "ground_mesh_boundaries",
    "ground_mesh_near_v_max",
    "ground_fallback_h",
    "_mesh_last_triangle",
    "ground_center_x_cache",
}

RUNTIME_FUNCTIONS = {
    "_parse_ground_float_list",
    "load_ground_projection",
    "mesh_ground_pixel_to_world",
    "nearest_ground_boundary",
    "classify_ground_outside",
    "ground_homography_pixel_to_world",
    "ground_far_limit_x",
    "fallback_ground_pixel_to_world",
    "ground_pixel_to_world",
    "center_line_world_x_for_row",
    "box_to_world",
    "world_cm_to_mm",
}


def load_runtime_projection(source_path):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    selected = []
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {
                target.id for target in node.targets
                if isinstance(target, ast.Name)
            }
            if names & RUNTIME_ASSIGNMENTS:
                selected.append(node)
        elif (isinstance(node, ast.FunctionDef) and
              node.name in RUNTIME_FUNCTIONS):
            selected.append(node)

    namespace = {}
    module = ast.fix_missing_locations(ast.Module(body=selected, type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    loaded = namespace["load_ground_projection"](str(MESH_PATH))
    if loaded is None:
        raise AssertionError("runtime rejected {}".format(MESH_PATH.name))
    namespace["ground_mesh_triangles"] = loaded[0]
    namespace["ground_mesh_boundaries"] = loaded[1]
    namespace["ground_mesh_near_v_max"] = loaded[2]
    namespace["ground_fallback_h"] = loaded[3]
    return namespace


def load_fit_points():
    with POINTS_PATH.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


class GroundProjectionTests(unittest.TestCase):
    def test_master_and_slave_projection_blocks_are_identical(self):
        pattern_start = "WORLD_X_LIMIT_CM = 250.0"
        pattern_end = "def clamp_int("
        blocks = []
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            start = source.index(pattern_start)
            end = source.index(pattern_end, start)
            blocks.append(source[start:end])
        for block in blocks[1:]:
            self.assertEqual(blocks[0], block)

    def test_mesh_metadata_and_fit_vertices(self):
        fit_points = load_fit_points()
        self.assertEqual(len(fit_points), 28)
        for source_path in RUNTIME_FILES:
            runtime = load_runtime_projection(source_path)
            self.assertEqual(len(runtime["ground_mesh_triangles"]), 36)
            self.assertEqual(len(runtime["ground_mesh_boundaries"]), 18)
            for row in fit_points:
                actual = runtime["ground_pixel_to_world"](
                    float(row["u"]), float(row["v"]))
                self.assertIsNotNone(actual, row["point_id"])
                self.assertAlmostEqual(actual[0], float(row["Xcm"]), places=9)
                self.assertAlmostEqual(actual[1], float(row["Ycm"]), places=9)

    def test_full_qvga_frame_has_bounded_coordinates(self):
        runtime = load_runtime_projection(RUNTIME_FILES[0])
        projection = runtime["ground_pixel_to_world"]
        for py in range(240):
            for px in range(320):
                world = projection(px, py)
                self.assertIsNotNone(world, (px, py))
                self.assertGreaterEqual(world[0], -250.0)
                self.assertLessEqual(world[0], 250.0)
                self.assertGreater(world[1], 0.0)
                self.assertLessEqual(world[1], 164.0)

    def test_output_boxes_recenter_x_on_image_midline(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime_projection(source_path)
            projection = runtime["ground_pixel_to_world"]
            box_to_world = runtime["box_to_world"]
            self.assertTrue(runtime["GROUND_CENTER_X_ON_IMAGE"])

            for py in (35, 40, 52, 69, 100, 123, 160, 200, 220, 235):
                raw = projection(160, py)
                actual = box_to_world(159.0, py - 0.5, 2.0, 1.0)
                self.assertIsNotNone(raw)
                self.assertIsNotNone(actual)
                self.assertAlmostEqual(actual[0], 0.0, places=9)
                self.assertAlmostEqual(actual[1], raw[1], places=9)

            runtime["GROUND_CENTER_X_ON_IMAGE"] = False
            raw = projection(160, 69)
            self.assertEqual(box_to_world(159.0, 68.5, 2.0, 1.0), raw)

    def test_competition_uart_units_remain_millimetres(self):
        runtime = load_runtime_projection(RUNTIME_FILES[0])
        convert = runtime["world_cm_to_mm"]
        self.assertEqual(convert(164.0), 1640)
        self.assertEqual(convert(1.25), 13)
        self.assertEqual(convert(-1.25), -13)

    def test_build_04_uses_final_model_field_exposure_and_id2_switch(self):
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            self.assertIn("optional ID2-first gate", source.splitlines()[0])
            self.assertIn("MODEL_PATH = '/sd/80lite0.5SS.tflite'", source)
            self.assertIn("EXPOSURE_INIT = 880", source)
            self.assertIn("ID2_ABSOLUTE_PRIORITY = True", source)


if __name__ == "__main__":
    unittest.main()
