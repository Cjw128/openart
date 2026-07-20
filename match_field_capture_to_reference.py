#!/usr/bin/env python3
"""Calibrate field captures without changing the original images.

The default fixed-wb mode estimates one global RGB correction from a neutral
object. The optional blue-selective mode matches only blue-dominant pixels to
the reference and cannot be reproduced by a camera white-balance setting.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}
DEFAULT_WB = (101.0, 64.0, 97.0)
DEFAULT_NEUTRAL_ROI = (135, 72, 70, 48)


def parse_args() -> argparse.Namespace:
    desktop = Path.home() / "Desktop"
    parser = argparse.ArgumentParser(
        description="Color-match field captures to image.png using the blue floor."
    )
    parser.add_argument(
        "--reference",
        type=Path,
        default=Path(__file__).resolve().with_name("image.png"),
        help="Reference image (default: image.png beside this script).",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=desktop / "field_capture",
        help="Directory containing the real-field images.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=desktop / "field_capture_image_matched",
        help="New directory for corrected images.",
    )
    parser.add_argument(
        "--strength",
        type=float,
        default=1.0,
        help="Correction strength; 0 is unchanged, 1 applies the full correction.",
    )
    parser.add_argument(
        "--mode",
        choices=("fixed-wb", "blue-selective"),
        default="fixed-wb",
        help="fixed-wb is camera-reproducible; blue-selective is offline only.",
    )
    parser.add_argument(
        "--neutral-image",
        type=Path,
        default=None,
        help="Image containing a neutral object (default: input/field_00140.jpg).",
    )
    parser.add_argument(
        "--neutral-roi",
        type=int,
        nargs=4,
        metavar=("X", "Y", "W", "H"),
        default=DEFAULT_NEUTRAL_ROI,
        help="Neutral-object ROI in pixels (default: white bear in frame 140).",
    )
    parser.add_argument(
        "--neutral-min",
        type=float,
        default=100.0,
        help="Minimum RGB channel value for neutral sample pixels.",
    )
    parser.add_argument(
        "--current-wb",
        type=float,
        nargs=3,
        metavar=("R", "G", "B"),
        default=DEFAULT_WB,
        help="Current fixed camera WB tuple used to estimate the new tuple.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Process only the first N images (useful for a quick test).",
    )
    return parser.parse_args()


def image_paths(directory: Path) -> list[Path]:
    paths = [
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    ]
    return sorted(paths, key=lambda path: path.name.lower())


def load_rgb(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.uint8)


def blue_floor_median(rgb: np.ndarray, path: Path) -> np.ndarray:
    """Return median RGB for blue floor pixels in the lower scene region."""
    height, width, _ = rgb.shape
    x0, x1 = int(width * 0.05), int(width * 0.95)
    y0, y1 = int(height * 0.35), int(height * 0.93)
    crop = rgb[y0:y1, x0:x1].astype(np.float32)

    red = crop[..., 0]
    green = crop[..., 1]
    blue = crop[..., 2]
    mask = (
        (blue >= 45.0)
        & (blue < 250.0)
        & (blue >= red * 1.35)
        & (blue >= green * 1.25)
    )
    pixels = crop[mask]
    minimum = max(500, int(crop.shape[0] * crop.shape[1] * 0.02))
    if pixels.shape[0] < minimum:
        raise ValueError(
            f"{path}: only {pixels.shape[0]} blue-floor pixels found; "
            f"need at least {minimum}"
        )
    return np.median(pixels, axis=0)


def sequence_blue_median(paths: list[Path]) -> tuple[np.ndarray, list[list[float]]]:
    per_image = []
    for path in paths:
        per_image.append(blue_floor_median(load_rgb(path), path))
    medians = np.asarray(per_image, dtype=np.float64)
    return np.median(medians, axis=0), medians.tolist()


def neutral_object_median(
    rgb: np.ndarray, roi: tuple[int, int, int, int], minimum: float, path: Path
) -> np.ndarray:
    x, y, width, height = roi
    image_height, image_width, _ = rgb.shape
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError(f"Invalid neutral ROI: {roi}")
    if x + width > image_width or y + height > image_height:
        raise ValueError(
            f"Neutral ROI {roi} exceeds {image_width}x{image_height}: {path}"
        )

    crop = rgb[y : y + height, x : x + width].astype(np.float32)
    maximum = crop.max(axis=2)
    minimum_channel = crop.min(axis=2)
    saturation = (maximum - minimum_channel) / np.maximum(maximum, 1.0)
    mask = (minimum_channel >= minimum) & (maximum < 250.0) & (saturation < 0.28)
    pixels = crop[mask]
    if pixels.shape[0] < 100:
        raise ValueError(
            f"{path}: only {pixels.shape[0]} neutral pixels found in ROI {roi}"
        )
    return np.median(pixels, axis=0)


def smoothstep(values: np.ndarray, low: float, high: float) -> np.ndarray:
    scaled = np.clip((values - low) / (high - low), 0.0, 1.0)
    return scaled * scaled * (3.0 - 2.0 * scaled)


def blue_correction_weight(rgb: np.ndarray) -> np.ndarray:
    """Return a soft 0..1 mask that protects white, grey, red, and yellow."""
    red = rgb[..., 0]
    green = rgb[..., 1]
    blue = rgb[..., 2]
    strongest_other = np.maximum(red, green)
    blue_ratio = (blue + 8.0) / (strongest_other + 8.0)
    blue_excess = blue - strongest_other
    return smoothstep(blue_ratio, 1.08, 1.28) * smoothstep(
        blue_excess, 8.0, 28.0
    )


def corrected_rgb(
    rgb: np.ndarray, gains: np.ndarray, blue_selective: bool
) -> np.ndarray:
    rgb = rgb.astype(np.float32)
    if blue_selective:
        weight = blue_correction_weight(rgb)
    else:
        weight = np.ones(rgb.shape[:2], dtype=np.float32)
    pixel_gains = np.exp(weight[..., None] * np.log(gains).reshape(1, 1, 3))
    return np.clip(np.rint(rgb * pixel_gains), 0, 255).astype(np.uint8)


def apply_channel_gains(
    source: Path, destination: Path, gains: np.ndarray, blue_selective: bool
) -> None:
    corrected = corrected_rgb(load_rgb(source), gains, blue_selective)
    output = Image.fromarray(corrected)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.suffix.lower() in {".jpg", ".jpeg"}:
        output.save(destination, quality=95, subsampling=0)
    else:
        output.save(destination)


def rounded(values: np.ndarray, digits: int = 4) -> list[float]:
    return [round(float(value), digits) for value in values]


def main() -> None:
    args = parse_args()
    reference = args.reference.resolve()
    input_dir = args.input.resolve()
    output_dir = args.output.resolve()
    neutral_image = (
        args.neutral_image.resolve()
        if args.neutral_image is not None
        else input_dir / "field_00140.jpg"
    )

    if not 0.0 <= args.strength <= 1.5:
        raise SystemExit("--strength must be between 0 and 1.5")
    if not reference.is_file():
        raise SystemExit(f"Reference image not found: {reference}")
    if not input_dir.is_dir():
        raise SystemExit(f"Input directory not found: {input_dir}")
    if args.mode == "fixed-wb" and not neutral_image.is_file():
        raise SystemExit(f"Neutral reference image not found: {neutral_image}")
    if output_dir == input_dir:
        raise SystemExit("Output directory must differ from input directory")

    sources = image_paths(input_dir)
    if args.limit is not None:
        if args.limit <= 0:
            raise SystemExit("--limit must be positive")
        sources = sources[: args.limit]
    if not sources:
        raise SystemExit(f"No supported images found in: {input_dir}")

    target_median = blue_floor_median(load_rgb(reference), reference).astype(
        np.float64
    )
    source_median, per_image_medians = sequence_blue_median(sources)
    floor_match_gains = target_median / np.maximum(source_median, 1.0)
    neutral_before = None
    if args.mode == "fixed-wb":
        neutral_before = neutral_object_median(
            load_rgb(neutral_image),
            tuple(args.neutral_roi),
            args.neutral_min,
            neutral_image,
        ).astype(np.float64)
        full_gains = neutral_before[1] / np.maximum(neutral_before, 1.0)
    else:
        full_gains = floor_match_gains
    applied_gains = np.exp(np.log(full_gains) * args.strength)

    output_dir.mkdir(parents=True, exist_ok=True)
    for index, source in enumerate(sources, start=1):
        apply_channel_gains(
            source,
            output_dir / source.name,
            applied_gains,
            blue_selective=args.mode == "blue-selective",
        )
        if index == 1 or index % 50 == 0 or index == len(sources):
            print(f"[{index:3d}/{len(sources)}] {source.name}")

    corrected_paths = [output_dir / source.name for source in sources]
    corrected_median, _ = sequence_blue_median(corrected_paths)
    relative_wb = applied_gains / applied_gains[1]
    current_wb = np.asarray(args.current_wb, dtype=np.float64)
    estimated_wb = current_wb * relative_wb
    neutral_after = None
    if args.mode == "fixed-wb":
        neutral_after = neutral_object_median(
            corrected_rgb(load_rgb(neutral_image), applied_gains, False),
            tuple(args.neutral_roi),
            args.neutral_min,
            neutral_image,
        ).astype(np.float64)

    report = {
        "purpose": (
            "camera-reproducible fixed white balance"
            if args.mode == "fixed-wb"
            else "offline blue-field domain matching"
        ),
        "reference": str(reference),
        "input_directory": str(input_dir),
        "output_directory": str(output_dir),
        "image_count": len(sources),
        "strength": args.strength,
        "correction_mode": args.mode,
        "reference_blue_median_rgb": rounded(target_median, 2),
        "source_blue_median_rgb": rounded(source_median, 2),
        "corrected_blue_median_rgb": rounded(corrected_median, 2),
        "blue_floor_only_match_gains": rounded(floor_match_gains),
        "full_rgb_pixel_gains": rounded(full_gains),
        "applied_rgb_pixel_gains": rounded(applied_gains),
        "neutral_reference_image": str(neutral_image),
        "neutral_roi_xywh": list(args.neutral_roi),
        "neutral_before_rgb": (
            rounded(neutral_before, 2) if neutral_before is not None else None
        ),
        "neutral_after_rgb": (
            rounded(neutral_after, 2) if neutral_after is not None else None
        ),
        "current_openart_wb": rounded(current_wb, 2),
        "estimated_openart_wb": (
            rounded(estimated_wb, 2) if args.mode == "fixed-wb" else None
        ),
        "blue_floor_per_image_medians_rgb": per_image_medians,
        "sampling": {
            "normalized_roi": [0.05, 0.35, 0.95, 0.93],
            "mask": "B>=45, B<250, B>=1.35*R, B>=1.25*G",
        },
        "application_mask": {
            "blue_ratio_smooth_range": [1.08, 1.28],
            "blue_excess_smooth_range": [8.0, 28.0],
        },
        "note": (
            "The estimated WB assumes camera tuple values are approximately "
            "linear. Replace the white-bear sample with a neutral grey card "
            "at the venue for final calibration."
            if args.mode == "fixed-wb"
            else "Blue-selective correction cannot be implemented as one "
            "global camera WB tuple."
        ),
    }
    report_path = output_dir / "matching_report.json"
    report_path.write_text(
        json.dumps(report, ensure_ascii=True, indent=2), encoding="utf-8"
    )

    print("\nColor matching complete")
    print(f"  target median : {rounded(target_median, 2)}")
    print(f"  source median : {rounded(source_median, 2)}")
    print(f"  corrected     : {rounded(corrected_median, 2)}")
    print(f"  RGB gains     : {rounded(applied_gains)}")
    print(f"  mode          : {args.mode}")
    if neutral_before is not None:
        print(f"  neutral before: {rounded(neutral_before, 2)}")
        print(f"  neutral after : {rounded(neutral_after, 2)}")
        print(f"  trial WB      : {rounded(estimated_wb, 2)}")
    print(f"  output        : {output_dir}")
    print(f"  report        : {report_path}")


if __name__ == "__main__":
    main()
