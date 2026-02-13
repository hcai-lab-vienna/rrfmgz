#!/usr/bin/env python3

import csv
from sys import argv

import numpy as np
from matplotlib import pyplot as plt

SCAN_RADIUS = 3
RESOLUTION = 0.1  # smaller = more accurate, slower
SCAN_RADIUS_SAMPLED = int(SCAN_RADIUS / RESOLUTION)


def load_file(file_path):
    positions = []
    configs = dict()
    with open(file_path, "r") as f:
        header = f.readline()
        header = header.replace("#", "").strip().split(";")[:-1]
        for pair in header:
            key, val = pair.split("=")
            configs[key] = val
        reader = csv.reader(f)
        for row in reader:
            positions.append((float(row[0]), float(row[1])))
    return positions, configs


def calculate_coverage(positions, configs):
    grid_size = [float(x) for x in configs["RRFM_BOX_SIZE"].split(",")]
    cells_x = int(grid_size[0] / RESOLUTION)
    cells_y = int(grid_size[1] / RESOLUTION)
    covered = np.zeros((cells_x, cells_y))
    for x, y in positions:
        rx = int((x + grid_size[0] / 2) / RESOLUTION)
        ry = int((y + grid_size[1] / 2) / RESOLUTION)
        min_x = rx - SCAN_RADIUS_SAMPLED
        max_x = rx + SCAN_RADIUS_SAMPLED
        min_y = ry - SCAN_RADIUS_SAMPLED
        max_y = ry + SCAN_RADIUS_SAMPLED
        for ix in range(min_x, max_x):
            for iy in range(min_y, max_y):
                try:
                    covered[cells_y - iy, ix] = 1
                except IndexError:
                    pass
    # plt.matshow(covered)
    # plt.show()
    total_cells = cells_x * cells_y
    covered_cells = np.count_nonzero(covered)
    coverage_percent = 100.0 * covered_cells / total_cells
    return coverage_percent


def main(args=None):
    if args is None or len(args) != 2:
        print("Usage: python3 coverage.py recorded_positions.csv")
        exit(1)
    file_path = args[1]
    positions, configs = load_file(file_path)
    coverage_percent = calculate_coverage(positions, configs)
    for key, val in configs.items():
        print(f"{key}={val}")
    print(f"Coverage: {coverage_percent:.2f}%")


if __name__ == "__main__":
    main(argv)
