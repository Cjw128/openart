import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME_FILES = (ROOT / "main.py", ROOT / "minimain.py")

SELECTED_ASSIGNMENTS = {
    "ORBIT_Y_CUT",
    "ORBIT_ROI_COMMAND",
    "orbit_y_cut_active",
}

SELECTED_FUNCTIONS = {
    "orbit_y_cut_allows_box",
    "clear_orbit_y_cut",
    "begin_orbit_y_cut",
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


def load_runtime(source_path, color_id=2):
    namespace = {}
    module = ast.fix_missing_locations(ast.Module(
        body=selected_nodes(source_path), type_ignores=[]))
    exec(compile(module, str(source_path), "exec"), namespace)
    namespace["active_selected_color_id"] = lambda: color_id
    return namespace


def function_node(source_path, name):
    tree = ast.parse(source_path.read_text(encoding="utf-8"),
                     filename=str(source_path))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError("missing function: " + name)


def called_names(node):
    return {
        child.func.id for child in ast.walk(node)
        if isinstance(child, ast.Call) and isinstance(child.func, ast.Name)
    }


def loaded_names(node):
    return {
        child.id for child in ast.walk(node)
        if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load)
    }


def command_branch_calls(source_path, command_value):
    receive = function_node(source_path, "receive_command_from_host")
    for node in ast.walk(receive):
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Compare):
            continue
        test = node.test
        if (not isinstance(test.left, ast.Name) or test.left.id != "command" or
                len(test.ops) != 1 or not isinstance(test.ops[0], ast.Eq) or
                len(test.comparators) != 1):
            continue
        comparator = test.comparators[0]
        matches = (
            isinstance(command_value, str) and
            isinstance(comparator, ast.Name) and
            comparator.id == command_value
        ) or (
            isinstance(command_value, int) and
            isinstance(comparator, ast.Constant) and
            comparator.value == command_value
        )
        if matches:
            calls = set()
            for statement in node.body:
                calls.update(called_names(statement))
            return calls
    return set()


class FakeUart:
    def __init__(self, data):
        self.data = bytearray(data)

    def any(self):
        return len(self.data)

    def read(self, count):
        chunk = self.data[:count]
        self.data = self.data[count:]
        return chunk


class OrbitYCutTests(unittest.TestCase):
    def test_helpers_are_identical_in_both_openart_builds(self):
        dumps = [ast.dump(ast.Module(
            body=selected_nodes(path), type_ignores=[]),
            include_attributes=False) for path in RUNTIME_FILES]
        self.assertEqual(dumps[0], dumps[1])

    def test_cut_is_full_width_and_rejects_only_boxes_above_y(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            allows = runtime["orbit_y_cut_allows_box"]
            cut_y = runtime["ORBIT_Y_CUT"]

            self.assertTrue(allows((0, 20, 20, 20)))
            self.assertTrue(runtime["begin_orbit_y_cut"]())
            self.assertFalse(allows((100, cut_y - 40, 30, 39)))
            self.assertTrue(allows((0, cut_y - 40, 30, 40)))
            self.assertTrue(allows((290, cut_y, 30, 40)))

    def test_begin_requires_target_and_is_idempotent(self):
        for source_path in RUNTIME_FILES:
            missing = load_runtime(source_path, color_id=0)
            self.assertFalse(missing["begin_orbit_y_cut"]())
            self.assertFalse(missing["orbit_y_cut_active"])

            runtime = load_runtime(source_path)
            self.assertTrue(runtime["begin_orbit_y_cut"]())
            self.assertTrue(runtime["orbit_y_cut_active"])
            self.assertTrue(runtime["begin_orbit_y_cut"]())
            self.assertTrue(runtime["orbit_y_cut_active"])

    def test_clear_restores_unrestricted_search(self):
        for source_path in RUNTIME_FILES:
            runtime = load_runtime(source_path)
            runtime["begin_orbit_y_cut"]()
            runtime["clear_orbit_y_cut"]()
            self.assertFalse(runtime["orbit_y_cut_active"])
            self.assertTrue(runtime["orbit_y_cut_allows_box"](
                (100, 20, 20, 20)))

    def test_model_blob_and_uart_paths_apply_the_cut(self):
        for source_path in RUNTIME_FILES:
            model = function_node(source_path, "run_model_best")
            gate = function_node(source_path, "color_tracking_gate")
            host_color = function_node(source_path,
                                       "apply_host_hybrid_color")
            reset = function_node(source_path,
                                  "reset_target_tracking_state")

            self.assertIn("orbit_y_cut_allows_box", called_names(model))
            self.assertIn("intersect_rois", called_names(gate))
            self.assertIn("ORBIT_Y_CUT", loaded_names(gate))
            self.assertIn("orbit_y_cut_active", loaded_names(gate))
            self.assertIn("clear_orbit_y_cut", called_names(host_color))
            self.assertIn("clear_orbit_y_cut", called_names(reset))
            self.assertIn("begin_orbit_y_cut", command_branch_calls(
                source_path, "ORBIT_ROI_COMMAND"))
            self.assertIn("clear_orbit_y_cut", command_branch_calls(
                source_path, 0x01))

    def test_orbit_command_uses_existing_short_uart_frame(self):
        for source_path in RUNTIME_FILES:
            entered = []
            namespace = {
                "uart": FakeUart((0xAA, 0x55, 0x0B, 0x0B)),
                "_cmd_rx_buf": bytearray(),
                "TARGET_SCAN_COMMAND": 0x09,
                "TARGET_SELECT_COMMAND": 0x0A,
                "ORBIT_ROI_COMMAND": 0x0B,
                "TARGET_COMMAND_FRAME_SIZE": 6,
                "begin_orbit_y_cut": lambda: entered.append(True),
            }
            module = ast.fix_missing_locations(ast.Module(
                body=[function_node(source_path,
                                    "receive_command_from_host")],
                type_ignores=[]))
            exec(compile(module, str(source_path), "exec"), namespace)
            namespace["receive_command_from_host"]()
            self.assertEqual(entered, [True])
            self.assertEqual(namespace["_cmd_rx_buf"], bytearray())

    def test_five_frame_roi_state_is_removed(self):
        for source_path in RUNTIME_FILES:
            source = source_path.read_text(encoding="utf-8")
            self.assertNotIn("orbit_roi_samples", source)
            self.assertNotIn("update_orbit_roi_lock", source)
            self.assertNotIn("ORBIT_ROI_SAMPLE_FRAMES", source)


if __name__ == "__main__":
    unittest.main()
