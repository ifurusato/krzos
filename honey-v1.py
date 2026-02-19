#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-02-19
# modified: 2026-02-19

import math
import sys

# =========================
# Geometry functions
# =========================
HEX_SIZE = 100.0  # distance between hex centers in mm

def hex_to_pixel(q, r, size=HEX_SIZE):
    x = size * math.sqrt(3) * (q + r/2)
    y = size * 1.5 * r
    return x, y

def polygon_vertices(q, r, size=HEX_SIZE):
    cx, cy = hex_to_pixel(q, r, size)
    return [(cx + size * math.cos(math.radians(angle)),
             cy + size * math.sin(math.radians(angle)))
            for angle in range(30, 391, 60)]

# =========================
# Hex cell
# =========================
class HexCell:
    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.occupied = False
        self.origin = False

# =========================
# Quad tree node
# =========================
class QuadTreeNode:
    def __init__(self, x_min, y_min, x_max, y_max, depth=0, max_depth=4):
        self.bounds = (x_min, y_min, x_max, y_max)
        self.depth = depth
        self.max_depth = max_depth
        self.children = []
        self.hexes = []

    def contains(self, hex_cell):
        x, y = hex_to_pixel(hex_cell.q, hex_cell.r)
        x_min, y_min, x_max, y_max = self.bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def insert(self, hex_cell):
        if self.depth == self.max_depth or not self.children:
            self.hexes.append(hex_cell)
            if self.depth < self.max_depth and len(self.hexes) > 4:
                self.subdivide()
        else:
            for child in self.children:
                if child.contains(hex_cell):
                    child.insert(hex_cell)
                    return

    def subdivide(self):
        x_min, y_min, x_max, y_max = self.bounds
        xm = (x_min + x_max)/2
        ym = (y_min + y_max)/2
        self.children = [
            QuadTreeNode(x_min, y_min, xm, ym, self.depth+1, self.max_depth),
            QuadTreeNode(xm, y_min, x_max, ym, self.depth+1, self.max_depth),
            QuadTreeNode(x_min, ym, xm, y_max, self.depth+1, self.max_depth),
            QuadTreeNode(xm, ym, x_max, y_max, self.depth+1, self.max_depth)
        ]
        old_hexes = self.hexes
        self.hexes = []
        for h in old_hexes:
            for child in self.children:
                if child.contains(h):
                    child.insert(h)
                    break

# =========================
# Hex map
# =========================
class HexMap:
    def __init__(self, max_distance_mm=4000, hex_size_mm=HEX_SIZE):
        self.hex_size = hex_size_mm
        self.max_distance = max_distance_mm
        self.hexes = {}
        steps = int(math.ceil(max_distance_mm / hex_size_mm))
        for q in range(-steps, steps+1):
            r1 = max(-steps, -q - steps)
            r2 = min(steps, -q + steps)
            for r in range(r1, r2+1):
                self.hexes[(q,r)] = HexCell(q,r)
        self.origin = self.hexes[(0,0)]
        self.origin.origin = True
        self.origin.occupied = True

        # Build quad tree
        min_x = min(hex_to_pixel(q,r)[0] for q,r in self.hexes)
        max_x = max(hex_to_pixel(q,r)[0] for q,r in self.hexes)
        min_y = min(hex_to_pixel(q,r)[1] for q,r in self.hexes)
        max_y = max(hex_to_pixel(q,r)[1] for q,r in self.hexes)
        self.quadtree = QuadTreeNode(min_x, min_y, max_x, max_y)
        for cell in self.hexes.values():
            self.quadtree.insert(cell)

    def process_sensor_reading(self, response_str):
        readings = [int(r) for r in response_str.split()]
        if len(readings) != 8:
            raise ValueError("Expected 8 sensor readings, got {}".format(len(readings)))
        SENSOR_DIRS = [
            (0,-1), (1,-1), (1,0), (0,1),
            (0,1), (-1,1), (-1,0), (-1,-1)
        ]
        for i, dist_mm in enumerate(readings):
            if dist_mm in (0, 9999):
                continue
            dq, dr = SENSOR_DIRS[i]
            steps = max(1, int(round(dist_mm / self.hex_size)))
            q = 0 + dq * steps
            r = 0 + dr * steps
            cell = self.hexes.get((q,r))
            if cell and not cell.origin:
                cell.occupied = True

# =========================
# Visualizer
# =========================
class HexVisualizer:
    def __init__(self, hex_map):
        self.map = hex_map

    def generate_svg(self, filename="hex_map.svg"):
        positions = [hex_to_pixel(c.q,c.r) for c in self.map.hexes.values()]
        min_x = min(x for x,_ in positions) - self.map.hex_size
        max_x = max(x for x,_ in positions) + self.map.hex_size
        min_y = min(y for _,y in positions) - self.map.hex_size
        max_y = max(y for _,y in positions) + self.map.hex_size
        width = max_x - min_x
        height = max_y - min_y

        with open(filename,"w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}mm" height="{height}mm" '
                    f'viewBox="0 0 {width} {height}">\n')
            f.write('<g stroke-width="0.2">\n')
            for cell in self.map.hexes.values():
                x_off, y_off = hex_to_pixel(cell.q, cell.r)

                # rotate so north is up
                x_off, y_off = x_off, -y_off  

                x_off -= min_x
                y_off -= min_y
                if cell.origin:
                    stroke, fill = "red", "red"
                elif cell.occupied:
                    stroke, fill = "black", "lightgray"
                else:
                    stroke, fill = "black", "none"
                verts = polygon_vertices(cell.q, cell.r)
                points = [f"{x-min_x},{y-min_y}" for x,y in verts]
                f.write(f'<polygon stroke="{stroke}" fill="{fill}" points="{" ".join(points)}" />\n')
            f.write('</g>\n</svg>\n')
        print(f"SVG written to {filename}, total hexes: {len(self.map.hexes)}")

# =========================
# Main
# =========================
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python hex_map.py \"<8 sensor readings>\"")
        print("Example: python hex_map.py \"0053 0906 2195 0284 0221 1260 2270 0335\"")
        sys.exit(1)

    sensor_input = sys.argv[1]
    hex_map = HexMap()
    hex_map.process_sensor_reading(sensor_input)

    visualizer = HexVisualizer(hex_map)
    visualizer.generate_svg("hex_map.svg")

#EOF
