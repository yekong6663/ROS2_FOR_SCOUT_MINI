#!/usr/bin/env python3
"""Mark verified robot start areas as free in a Nav2 PNG map.

This is deliberately a small, explicit repair tool.  It is only for a
physical position that has been verified to be clear; it does not change map
boundaries or infer free space from lidar data.
"""

import argparse
import math
from pathlib import Path

import yaml
from PIL import Image


def world_to_pixel(metadata, image, x, y):
    resolution = float(metadata["resolution"])
    origin_x, origin_y, origin_yaw = [float(value) for value in metadata["origin"]]
    delta_x = x - origin_x
    delta_y = y - origin_y
    grid_x = (math.cos(origin_yaw) * delta_x + math.sin(origin_yaw) * delta_y) / resolution
    grid_y = (-math.sin(origin_yaw) * delta_x + math.cos(origin_yaw) * delta_y) / resolution
    return math.floor(grid_x), image.height - 1 - math.floor(grid_y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--center", nargs=2, action="append", required=True, type=float,
                        metavar=("X", "Y"))
    parser.add_argument("--radius", type=float, default=0.80)
    parser.add_argument("--backup", action="store_true")
    args = parser.parse_args()

    metadata = yaml.safe_load(args.map_yaml.read_text(encoding="utf-8"))
    image_path = args.map_yaml.parent / metadata["image"]
    image = Image.open(image_path).convert("L")
    pixels = image.load()
    radius_pixels = math.ceil(args.radius / float(metadata["resolution"]))

    if args.backup:
        backup = image_path.with_name(image_path.stem + ".before_start_area_repair.png")
        if not backup.exists():
            image.save(backup)

    changed = 0
    for x, y in args.center:
        center_col, center_row = world_to_pixel(metadata, image, x, y)
        for row in range(max(0, center_row - radius_pixels), min(image.height, center_row + radius_pixels + 1)):
            for col in range(max(0, center_col - radius_pixels), min(image.width, center_col + radius_pixels + 1)):
                if math.hypot(col - center_col, row - center_row) <= radius_pixels:
                    if pixels[col, row] != 255:
                        pixels[col, row] = 255
                        changed += 1
        print(f"cleared verified start area at x={x:.3f}, y={y:.3f}")

    image.save(image_path)
    print(f"saved {image_path}; changed {changed} cells")


if __name__ == "__main__":
    main()
