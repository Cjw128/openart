#!/usr/bin/env python3
"""Build a raw-pixel to ground-coordinate mesh from red-bag samples.

CSV columns:
    point_id,u,v,Xcm,Ycm[,split]

Rows named R1C1..RnC4 with split=fit are used as mesh vertices. Row labels are
stable IDs; rows are ordered from near to far by their measured Y coordinates.
Optional split=verify rows are never fitted and are used only for checks.
"""

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np


IMAGE_W = 320
IMAGE_H = 240
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_GROUND_CSV = SCRIPT_DIR / "ground_mesh_16_points_template.csv"
DEFAULT_MESH_OUTPUT = SCRIPT_DIR / "camera_ground_mesh.txt"
DEFAULT_REPORT_OUTPUT = SCRIPT_DIR / "camera_ground_mesh_report.json"
BOUNDARY_TOO_NEAR = 1
BOUNDARY_TOO_FAR = 2
BOUNDARY_LEFT = 3
BOUNDARY_RIGHT = 4


class CalibrationError(RuntimeError):
    pass


def read_ground_csv(path, expected_points, max_x_cm, max_y_cm):
    fit = []
    verify = []
    with open(path, "r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"u", "v", "Xcm", "Ycm"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise CalibrationError("CSV must contain u,v,Xcm,Ycm columns")
        for line_number, row in enumerate(reader, start=2):
            try:
                u = float(row["u"])
                v = float(row["v"])
                x = float(row["Xcm"])
                y = float(row["Ycm"])
            except (TypeError, ValueError) as exc:
                raise CalibrationError("invalid number on CSV line {}".format(line_number)) from exc
            if not all(math.isfinite(value) for value in (u, v, x, y)):
                raise CalibrationError("non-finite value on CSV line {}".format(line_number))
            if not (0.0 <= u < IMAGE_W and 0.0 <= v < IMAGE_H):
                raise CalibrationError("pixel outside QVGA on CSV line {}".format(line_number))
            if not (-max_x_cm <= x <= max_x_cm and 0.0 < y <= max_y_cm):
                raise CalibrationError("world coordinate outside configured range on CSV line {}".format(line_number))
            split = (row.get("split") or "fit").strip().lower()
            point = {"line": line_number, "point_id": (row.get("point_id") or "").strip(),
                     "u": u, "v": v, "Xcm": x, "Ycm": y, "split": split}
            if split in ("fit", "train", "calibrate"):
                fit.append(point)
            elif split in ("verify", "validation", "holdout", "test"):
                verify.append(point)
            else:
                raise CalibrationError("unknown split '{}' on CSV line {}".format(split, line_number))

    if expected_points is not None and len(fit) != expected_points:
        raise CalibrationError(
            "expected exactly {} fit points, found {}".format(expected_points, len(fit))
        )
    for i in range(len(fit)):
        for j in range(i + 1, len(fit)):
            du = fit[i]["u"] - fit[j]["u"]
            dv = fit[i]["v"] - fit[j]["v"]
            if du*du + dv*dv < 0.25:
                raise CalibrationError("fit pixels on lines {} and {} are duplicates".format(
                    fit[i]["line"], fit[j]["line"]))
    return fit, verify


def _match_vertex(vertex, pixels):
    delta = pixels - np.asarray(vertex, dtype=np.float64)[None, :]
    distance2 = np.sum(delta * delta, axis=1)
    index = int(np.argmin(distance2))
    return index if distance2[index] <= 0.01 else None


def _triangle_min_angle_deg(pixel_triangle):
    angles = []
    for i in range(3):
        center = pixel_triangle[i]
        a = pixel_triangle[(i + 1) % 3] - center
        b = pixel_triangle[(i + 2) % 3] - center
        na = float(np.linalg.norm(a))
        nb = float(np.linalg.norm(b))
        if na <= 1e-12 or nb <= 1e-12:
            return 0.0
        cosine = float(np.dot(a, b) / (na * nb))
        cosine = max(-1.0, min(1.0, cosine))
        angles.append(math.degrees(math.acos(cosine)))
    return min(angles)


def _structured_grid_topology(points, pixels):
    grid = {}
    for index, point in enumerate(points):
        match = re.fullmatch(r"R([1-9][0-9]*)C([1-4])", point.get("point_id", "").upper())
        if match is None:
            return None
        key = (int(match.group(1)), int(match.group(2)))
        if key in grid:
            raise CalibrationError("duplicate structured point ID {}".format(point["point_id"]))
        grid[key] = index
    rows = sorted({key[0] for key in grid})
    columns = sorted({key[1] for key in grid})
    if (len(rows) < 4 or columns != [1, 2, 3, 4] or
            len(grid) != len(rows) * len(columns)):
        return None
    row_count = len(rows)
    column_count = len(columns)
    row_order = sorted(rows, key=lambda row: sum(
        points[grid[(row, column)]]["Ycm"]
        for column in range(1, column_count + 1)
    ) / column_count)

    for near_row, far_row in zip(row_order, row_order[1:]):
        for column in range(1, column_count + 1):
            near_point = points[grid[(near_row, column)]]
            far_point = points[grid[(far_row, column)]]
            if near_point["Ycm"] >= far_point["Ycm"]:
                raise CalibrationError(
                    "structured rows overlap in Y; each physical row must be farther than the previous row"
                )
    for row in rows:
        for column in range(1, column_count):
            left_point = points[grid[(row, column)]]
            right_point = points[grid[(row, column + 1)]]
            if left_point["Xcm"] >= right_point["Xcm"]:
                raise CalibrationError(
                    "structured columns must increase in X from C1 (left) to C4 (right)"
                )

    triangles = []
    for near_row, far_row in zip(row_order, row_order[1:]):
        for column in range(1, column_count):
            p00 = grid[(near_row, column)]
            p10 = grid[(near_row, column + 1)]
            p01 = grid[(far_row, column)]
            p11 = grid[(far_row, column + 1)]
            diagonal_00_11 = float(np.sum((pixels[p00] - pixels[p11]) ** 2))
            diagonal_10_01 = float(np.sum((pixels[p10] - pixels[p01]) ** 2))
            if diagonal_00_11 <= diagonal_10_01:
                triangles.append((p00, p10, p11))
                triangles.append((p00, p11, p01))
            else:
                triangles.append((p00, p10, p01))
                triangles.append((p10, p11, p01))
    boundaries = []
    boundary_paths = (
        (BOUNDARY_TOO_NEAR, [grid[(row_order[0], column)]
                            for column in range(1, column_count + 1)]),
        (BOUNDARY_TOO_FAR, [grid[(row_order[-1], column)]
                            for column in range(1, column_count + 1)]),
        (BOUNDARY_LEFT, [grid[(row, 1)] for row in row_order]),
        (BOUNDARY_RIGHT, [grid[(row, column_count)] for row in row_order]),
    )
    for status, path in boundary_paths:
        for start, end in zip(path, path[1:]):
            boundaries.append({
                "status": status,
                "indices": [start, end],
                "values": [
                    float(pixels[start, 0]), float(pixels[start, 1]),
                    float(pixels[end, 0]), float(pixels[end, 1]),
                ],
            })
    return {
        "triangles": triangles,
        "boundaries": boundaries,
        "grid_rows": row_count,
        "grid_columns": column_count,
        "grid_row_order": row_order,
    }


def build_mesh(points, min_triangle_angle_deg=1.0):
    if len(points) < 4:
        raise CalibrationError("at least four points are required")
    pixels = np.asarray([[point["u"], point["v"]] for point in points], dtype=np.float64)
    worlds = np.asarray([[point["Xcm"], point["Ycm"]] for point in points], dtype=np.float64)
    hull = cv2.convexHull(pixels.astype(np.float32)).reshape(-1, 2)
    hull_area = float(cv2.contourArea(hull))
    if hull_area <= 1.0:
        raise CalibrationError("fit points have a degenerate pixel convex hull")

    structured_topology = _structured_grid_topology(points, pixels)
    if structured_topology is not None:
        index_triangles = structured_topology["triangles"]
        boundaries = structured_topology["boundaries"]
        grid_rows = structured_topology["grid_rows"]
        grid_columns = structured_topology["grid_columns"]
        grid_row_order = structured_topology["grid_row_order"]
        triangulation_method = "structured_{}x{}".format(grid_rows, grid_columns)
    else:
        boundaries = []
        grid_rows = None
        grid_columns = None
        grid_row_order = None
        triangulation_method = "delaunay"
        subdiv = cv2.Subdiv2D((0, 0, IMAGE_W, IMAGE_H))
        for u, v in pixels:
            subdiv.insert((float(u), float(v)))
        delaunay_triangles = set()
        for raw in subdiv.getTriangleList():
            vertices = np.asarray(raw, dtype=np.float64).reshape(3, 2)
            indices = [_match_vertex(vertex, pixels) for vertex in vertices]
            if any(index is None for index in indices) or len(set(indices)) != 3:
                continue
            delaunay_triangles.add(tuple(sorted(indices)))
        index_triangles = sorted(delaunay_triangles)
        if not index_triangles:
            raise CalibrationError("Delaunay triangulation produced no usable triangles")

    triangles = []
    orientation_sign = 0
    min_angle = 180.0
    for indices in index_triangles:
        p = pixels[list(indices)].copy()
        w = worlds[list(indices)].copy()
        pixel_area2 = float((p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) -
                            (p[2, 0] - p[0, 0]) * (p[1, 1] - p[0, 1]))
        if abs(pixel_area2) <= 1.0:
            raise CalibrationError("triangle {} has near-zero pixel area".format(indices))
        if pixel_area2 < 0.0:
            p[[1, 2]] = p[[2, 1]]
            w[[1, 2]] = w[[2, 1]]
            indices = (indices[0], indices[2], indices[1])
            pixel_area2 = -pixel_area2

        world_area2 = float((w[1, 0] - w[0, 0]) * (w[2, 1] - w[0, 1]) -
                            (w[2, 0] - w[0, 0]) * (w[1, 1] - w[0, 1]))
        if abs(world_area2) <= 1e-4:
            raise CalibrationError("triangle {} has near-zero world area".format(indices))
        sign = 1 if world_area2 > 0.0 else -1
        if orientation_sign == 0:
            orientation_sign = sign
        elif sign != orientation_sign:
            raise CalibrationError(
                "mesh folds between image and world coordinates; check point correspondences"
            )

        angle = _triangle_min_angle_deg(p)
        min_angle = min(min_angle, angle)
        if angle < min_triangle_angle_deg:
            raise CalibrationError(
                "triangle {} minimum angle {:.3f} deg is below {:.3f} deg".format(
                    indices, angle, min_triangle_angle_deg
                )
            )
        triangles.append(
            {
                "indices": list(indices),
                "values": [
                    float(p[0, 0]), float(p[0, 1]), float(w[0, 0]), float(w[0, 1]),
                    float(p[1, 0]), float(p[1, 1]), float(w[1, 0]), float(w[1, 1]),
                    float(p[2, 0]), float(p[2, 1]), float(w[2, 0]), float(w[2, 1]),
                ],
                "pixel_area2": pixel_area2,
                "world_area2": world_area2,
                "min_angle_deg": angle,
            }
        )

    homography, _ = cv2.findHomography(pixels, worlds, method=0)
    if homography is None or not np.all(np.isfinite(homography)):
        raise CalibrationError("could not fit pixel-to-ground fallback homography")
    if abs(float(homography[2, 2])) <= 1e-12:
        raise CalibrationError("fallback homography has an invalid scale")
    homography = homography / homography[2, 2]
    fallback_predictions = cv2.perspectiveTransform(
        pixels.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    fallback_errors = np.linalg.norm(fallback_predictions - worlds, axis=1)
    fallback = {
        "model": "homography",
        "values": [float(value) for value in homography.reshape(-1)],
        "fit_rms_cm": float(np.sqrt(np.mean(fallback_errors * fallback_errors))),
        "fit_mean_cm": float(np.mean(fallback_errors)),
        "fit_max_cm": float(np.max(fallback_errors)),
    }
    return {
        "triangles": triangles,
        "hull": hull.astype(np.float64),
        "hull_area_px2": hull_area,
        "hull_coverage": hull_area / float(IMAGE_W * IMAGE_H),
        "min_triangle_angle_deg": min_angle,
        "triangulation_method": triangulation_method,
        "boundaries": boundaries,
        "grid_rows": grid_rows,
        "grid_columns": grid_columns,
        "grid_row_order": grid_row_order,
        "fallback": fallback,
    }


def interpolate_mesh(mesh, u, v, epsilon=1e-7):
    for triangle in mesh["triangles"]:
        values = triangle["values"]
        u0, v0, x0, y0 = values[0], values[1], values[2], values[3]
        u1, v1, x1, y1 = values[4], values[5], values[6], values[7]
        u2, v2, x2, y2 = values[8], values[9], values[10], values[11]
        denominator = (v1 - v2)*(u0 - u2) + (u2 - u1)*(v0 - v2)
        if abs(denominator) <= 1e-12:
            continue
        a = ((v1 - v2)*(u - u2) + (u2 - u1)*(v - v2)) / denominator
        b = ((v2 - v0)*(u - u2) + (u0 - u2)*(v - v2)) / denominator
        c = 1.0 - a - b
        if a >= -epsilon and b >= -epsilon and c >= -epsilon:
            return (a*x0 + b*x1 + c*x2, a*y0 + b*y1 + c*y2)
    return None


def verify_mesh(mesh, points):
    rows = []
    errors = []
    invalid = 0
    for point in points:
        predicted = interpolate_mesh(mesh, point["u"], point["v"])
        if predicted is None:
            invalid += 1
            rows.append(dict(point, valid=False))
            continue
        error_x = predicted[0] - point["Xcm"]
        error_y = predicted[1] - point["Ycm"]
        error = math.sqrt(error_x*error_x + error_y*error_y)
        errors.append(error)
        rows.append(dict(point, valid=True, predicted_Xcm=predicted[0],
                         predicted_Ycm=predicted[1], error_cm=error))
    return {
        "rows": rows,
        "count": len(points),
        "valid_count": len(errors),
        "invalid_count": invalid,
        "rms_cm": math.sqrt(sum(error*error for error in errors) / len(errors)) if errors else None,
        "mean_cm": sum(errors) / len(errors) if errors else None,
        "max_cm": max(errors) if errors else None,
    }


def leave_one_out_diagnostics(points, min_triangle_angle_deg):
    rows = []
    errors = []
    for index, point in enumerate(points):
        reduced = points[:index] + points[index + 1:]
        try:
            mesh = build_mesh(reduced, min_triangle_angle_deg)
            predicted = interpolate_mesh(mesh, point["u"], point["v"])
        except CalibrationError as exc:
            rows.append({"point_index": index, "build_error": str(exc)})
            continue
        if predicted is None:
            rows.append({"point_index": index, "inside_reduced_hull": False})
            continue
        error_x = predicted[0] - point["Xcm"]
        error_y = predicted[1] - point["Ycm"]
        error = math.sqrt(error_x*error_x + error_y*error_y)
        errors.append(error)
        rows.append({"point_index": index, "inside_reduced_hull": True,
                     "predicted_Xcm": predicted[0], "predicted_Ycm": predicted[1],
                     "error_cm": error})
    return {
        "rows": rows,
        "valid_count": len(errors),
        "rms_cm": math.sqrt(sum(error*error for error in errors) / len(errors)) if errors else None,
        "max_cm": max(errors) if errors else None,
    }


def write_mesh(path, args, fit_points, mesh):
    def numbers(values):
        return ",".join("{:.12g}".format(float(value)) for value in values)

    expected_boundary_count = 2 * (mesh["grid_rows"] - 1) + 2 * (mesh["grid_columns"] - 1) \
        if mesh["grid_rows"] is not None else 0
    if (not mesh["triangulation_method"].startswith("structured_") or
            len(mesh["boundaries"]) != expected_boundary_count):
        raise CalibrationError(
            "OpenART export requires a complete R1C1..RnC4 grid"
        )
    calibrated_y_min = min(point["Ycm"] for point in fit_points)
    calibrated_y_max = max(point["Ycm"] for point in fit_points)

    lines = [
        "# OpenART raw-pixel ground mesh; generated by calibrate_ground_camera.py",
        "# Coordinates: X right, Y forward; values use cm.",
        "version=4",
        "model=triangle_mesh",
        "role={}".format(args.role),
        "image_w={}".format(IMAGE_W),
        "image_h={}".format(IMAGE_H),
        "software_hmirror={}".format(args.software_hmirror),
        "sensor_vflip={}".format(args.sensor_vflip),
        "lens_corr=0",
        "units=cm",
        "max_x_cm={:.12g}".format(args.max_x_cm),
        "max_y_cm={:.12g}".format(args.max_y_cm),
        "calibrated_y_min_cm={:.12g}".format(calibrated_y_min),
        "calibrated_y_max_cm={:.12g}".format(calibrated_y_max),
        "point_count={}".format(len(fit_points)),
        "grid_rows={}".format(mesh["grid_rows"]),
        "grid_columns={}".format(mesh["grid_columns"]),
        "grid_row_order={}".format(",".join(str(row) for row in mesh["grid_row_order"])),
        "triangle_count={}".format(len(mesh["triangles"])),
        "boundary_count={}".format(len(mesh["boundaries"])),
        "fallback_model={}".format(mesh["fallback"]["model"]),
        "fallback_h={}".format(numbers(mesh["fallback"]["values"])),
        "fallback_fit_rms_cm={:.12g}".format(mesh["fallback"]["fit_rms_cm"]),
        "fallback_fit_max_cm={:.12g}".format(mesh["fallback"]["fit_max_cm"]),
    ]
    for index, point in enumerate(fit_points):
        lines.append("point{}={}".format(index, numbers(
            (point["u"], point["v"], point["Xcm"], point["Ycm"]))))
    for index, triangle in enumerate(mesh["triangles"]):
        lines.append("triangle{}={}".format(index, numbers(triangle["values"])))
    for index, boundary in enumerate(mesh["boundaries"]):
        lines.append("boundary{}={},{}".format(
            index, boundary["status"], numbers(boundary["values"])
        ))
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="ascii")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Build an OpenART ground-coordinate mesh from a 4-column red-bag grid."
    )
    parser.add_argument(
        "--ground-csv", default=str(DEFAULT_GROUND_CSV),
        help="CSV with point_id,u,v,Xcm,Ycm[,split] (default: checked-in 24-point data)"
    )
    parser.add_argument("--role", choices=("master", "slave"), default="master")
    parser.add_argument("--output", default=str(DEFAULT_MESH_OUTPUT))
    parser.add_argument("--report", default=str(DEFAULT_REPORT_OUTPUT))
    parser.add_argument(
        "--expected-points", type=int, default=24,
        help="optional exact fit-point count check (for example 24 for a 6x4 grid)"
    )
    parser.add_argument("--software-hmirror", type=int, choices=(0, 1), default=1)
    parser.add_argument("--sensor-vflip", type=int, choices=(0, 1), default=1)
    parser.add_argument("--max-x-cm", type=float, default=250.0)
    parser.add_argument("--max-y-cm", type=float, default=164.0)
    parser.add_argument(
        "--required-near-y-cm", type=float, default=9.0,
        help="reject export if the nearest fit row does not reach this pickup distance"
    )
    parser.add_argument("--min-triangle-angle-deg", type=float, default=1.0)
    parser.add_argument("--max-verify-rms-cm", type=float, default=2.0)
    parser.add_argument("--max-verify-error-cm", type=float, default=3.0)
    parser.add_argument("--allow-poor-fit", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.required_near_y_cm is not None and not (
                0.0 < args.required_near_y_cm <= args.max_y_cm):
            raise CalibrationError("required near Y must be inside (0, max-y-cm]")
        fit_points, verify_points = read_ground_csv(
            args.ground_csv, args.expected_points, args.max_x_cm, args.max_y_cm
        )
        mesh = build_mesh(fit_points, args.min_triangle_angle_deg)
        fit_check = verify_mesh(mesh, fit_points)
        if fit_check["invalid_count"] or (fit_check["max_cm"] is not None and fit_check["max_cm"] > 1e-7):
            raise CalibrationError("mesh does not reproduce its fit vertices")
        verify = verify_mesh(mesh, verify_points)
        leave_one_out = leave_one_out_diagnostics(fit_points, args.min_triangle_angle_deg)

        failures = []
        calibrated_y_min = min(point["Ycm"] for point in fit_points)
        calibrated_y_max = max(point["Ycm"] for point in fit_points)
        if (args.required_near_y_cm is not None and
                calibrated_y_min > args.required_near_y_cm):
            failures.append(
                "nearest calibration row is {:.3f} cm, beyond required pickup distance {:.3f} cm".format(
                    calibrated_y_min, args.required_near_y_cm
                )
            )
        if abs(calibrated_y_max - args.max_y_cm) > 1e-6:
            failures.append(
                "farthest calibration row must equal max Y {:.3f} cm, found {:.3f} cm".format(
                    args.max_y_cm, calibrated_y_max
                )
            )
        if verify_points:
            if verify["invalid_count"]:
                failures.append("one or more verify points lie outside the fit-point convex hull")
            if verify["rms_cm"] is not None and verify["rms_cm"] > args.max_verify_rms_cm:
                failures.append("verify RMS exceeds threshold")
            if verify["max_cm"] is not None and verify["max_cm"] > args.max_verify_error_cm:
                failures.append("verify maximum error exceeds threshold")
        exported = not failures or args.allow_poor_fit
        if exported:
            write_mesh(args.output, args, fit_points, mesh)

        report = {
            "schema": 4,
            "model": "triangle_mesh",
            "role": args.role,
            "raw_image_pipeline": {
                "width": IMAGE_W,
                "height": IMAGE_H,
                "software_hmirror": bool(args.software_hmirror),
                "sensor_vflip": bool(args.sensor_vflip),
                "lens_corr": False,
            },
            "fit_points": fit_points,
            "verify": verify,
            "leave_one_out": leave_one_out,
            "mesh": {
                "triangulation_method": mesh["triangulation_method"],
                "grid_rows": mesh["grid_rows"],
                "grid_columns": mesh["grid_columns"],
                "grid_row_order_near_to_far": mesh["grid_row_order"],
                "triangle_count": len(mesh["triangles"]),
                "triangles": mesh["triangles"],
                "boundaries": mesh["boundaries"],
                "pixel_hull": mesh["hull"].tolist(),
                "pixel_hull_area_px2": mesh["hull_area_px2"],
                "image_coverage_fraction": mesh["hull_coverage"],
                "minimum_triangle_angle_deg": mesh["min_triangle_angle_deg"],
                "calibrated_y_min_cm": calibrated_y_min,
                "calibrated_y_max_cm": calibrated_y_max,
                "fallback": mesh["fallback"],
            },
            "qa": {
                "passed": not failures,
                "has_independent_verify_points": bool(verify_points),
                "failures": failures,
                "thresholds": {
                    "max_verify_rms_cm": args.max_verify_rms_cm,
                    "max_verify_error_cm": args.max_verify_error_cm,
                    "required_near_y_cm": args.required_near_y_cm,
                },
            },
            "mesh_exported": exported,
        }
        report_path = Path(args.report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        print("fit points / triangles: {} / {}".format(len(fit_points), len(mesh["triangles"])))
        print("triangulation: {}".format(mesh["triangulation_method"]))
        print("calibrated Y range: {:.3f}..{:.3f} cm".format(
            calibrated_y_min, calibrated_y_max))
        print("pixel convex-hull coverage: {:.1f}%".format(mesh["hull_coverage"] * 100.0))
        print("minimum triangle angle: {:.2f} deg".format(mesh["min_triangle_angle_deg"]))
        print("fallback homography RMS/max: {:.3f}/{:.3f} cm".format(
            mesh["fallback"]["fit_rms_cm"], mesh["fallback"]["fit_max_cm"]))
        if leave_one_out["rms_cm"] is not None:
            print("leave-one-out RMS/max: {:.3f}/{:.3f} cm ({} interior points)".format(
                leave_one_out["rms_cm"], leave_one_out["max_cm"], leave_one_out["valid_count"]))
        if verify_points:
            print("verify RMS/max: {:.3f}/{:.3f} cm".format(
                verify["rms_cm"], verify["max_cm"]))
        else:
            print("warning: no independent verify points; validate on the OpenART test", file=sys.stderr)
        if not exported:
            print("mesh rejected: " + "; ".join(failures), file=sys.stderr)
            return 4
        print("wrote {} and {}".format(args.output, args.report))
        return 0
    except (CalibrationError, cv2.error, OSError, ValueError) as exc:
        print("mesh calibration failed: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
