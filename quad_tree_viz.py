
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-02-19
# modified: 2026-02-20

import sys
import math
from core.cardinal import Cardinal

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Node:
    '''
    Represents a single node in the occupancy grid.
    '''
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.state = 'unknown'  # 'unknown', 'free', 'occupied'
        self.is_origin = False

    def mark_free(self):
        if self.state != 'occupied':
            self.state = 'free'

    def mark_occupied(self):
        self.state = 'occupied'

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class QuadTreeNode:
    '''
    Spatial indexing structure for future optimization.
    '''
    def __init__(self, x_min, y_min, x_max, y_max, depth=0, max_depth=4):
        self.bounds = (x_min, y_min, x_max, y_max)
        self.depth = depth
        self.max_depth = max_depth
        self.children = []
        self.cells = []

    def contains(self, x, y):
        x_min, y_min, x_max, y_max = self.bounds
        return x_min <= x <= x_max and y_min <= y <= y_max

    def insert(self, node):
        if self.depth == self.max_depth or not self.children:
            self.cells.append(node)
            if self.depth < self.max_depth and len(self.cells) > 4:
                self.subdivide()
        else:
            for child in self.children:
                if child.contains(node.x, node.y):
                    child.insert(node)
                    return

    def subdivide(self):
        x_min, y_min, x_max, y_max = self.bounds
        xm = (x_min + x_max) / 2
        ym = (y_min + y_max) / 2
        self.children = [
            QuadTreeNode(x_min, y_min, xm, ym, self.depth + 1, self.max_depth),
            QuadTreeNode(xm, y_min, x_max, ym, self.depth + 1, self.max_depth),
            QuadTreeNode(x_min, ym, xm, y_max, self.depth + 1, self.max_depth),
            QuadTreeNode(xm, ym, x_max, y_max, self.depth + 1, self.max_depth)
        ]
        old_cells = self.cells
        self.cells = []
        for node in old_cells:
            for child in self.children:
                if child.contains(node.x, node.y):
                    child.insert(node)
                    break

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class OccupancyGrid:
    '''
    Manages the occupancy grid, processes sensor readings, and provides query methods.
    '''
    def __init__(self, cell_size=50, max_range=4000):
        self.cell_size = cell_size
        self.max_range = max_range
        self.cells = {}
        # create grid cells
        for x in range(-max_range, max_range + cell_size, cell_size):
            for y in range(-max_range, max_range + cell_size, cell_size):
                node = Node(x, y, cell_size)
                self.cells[(x, y)] = node
        # mark origin
        origin = self.cells.get((0, 0))
        if origin:
            origin.is_origin = True
        # build quad tree
        self.quadtree = QuadTreeNode(-max_range, -max_range, max_range, max_range)
        for node in self.cells.values():
            self.quadtree.insert(node)
        # cardinal direction vectors
        self._direction_vectors = {
            Cardinal.NORTH:     (0, 1),
            Cardinal.NORTHEAST: (1, 1),
            Cardinal.EAST:      (1, 0),
            Cardinal.SOUTHEAST: (1, -1),
            Cardinal.SOUTH:     (0, -1),
            Cardinal.SOUTHWEST: (-1, -1),
            Cardinal.WEST:      (-1, 0),
            Cardinal.NORTHWEST: (-1, 1)
        }

    def get_cell(self, x, y):
        '''
        return node at position, snapping to grid
        '''
        grid_x = round(x / self.cell_size) * self.cell_size
        grid_y = round(y / self.cell_size) * self.cell_size
        return self.cells.get((grid_x, grid_y))

    def process_sensor_reading(self, reading_string, robot_x=0, robot_y=0, robot_heading=0):
        '''
        parse sensor reading string and update grid cells
        reading_string: space-delimited string of 8 distance values in mm
        order: N NE E SE S SW W NW
        '''
        readings = [int(r) for r in reading_string.split()]
        if len(readings) != 8:
            raise ValueError("expected 8 sensor readings, got {}".format(len(readings)))
        for i in range(8):
            distance_mm = readings[i]
            if distance_mm in (0, 9999):
                continue
            cardinal = Cardinal.from_index(i)
            self._trace_ray(robot_x, robot_y, cardinal, distance_mm)

    def _trace_ray(self, start_x, start_y, cardinal, distance_mm):
        '''
        Trace a ray from start position in given direction, marking cells as free or occupied.
        '''
        dx, dy = self._direction_vectors[cardinal]
        # normalize direction vector
        length = math.sqrt(dx * dx + dy * dy)
        dx /= length
        dy /= length
        # mark cells along ray as free, final node as occupied
        current_distance = 0
        while current_distance <= distance_mm:
            x = start_x + dx * current_distance
            y = start_y + dy * current_distance
            node = self.get_cell(x, y)
            if not node:
                current_distance += self.cell_size
                continue
            if current_distance + self.cell_size > distance_mm:
                # at obstacle distance, mark as occupied
                if not node.is_origin:
                    node.mark_occupied()
                break
            else:
                # before obstacle, mark as free
                if not node.is_origin:
                    node.mark_free()
            current_distance += self.cell_size

    def x_trace_ray(self, start_x, start_y, cardinal, distance_mm):
        '''
        trace a ray from start position in given direction, marking cells as free or occupied
        '''
        dx, dy = self._direction_vectors[cardinal]
        # normalize direction vector
        length = math.sqrt(dx * dx + dy * dy)
        dx /= length
        dy /= length
        # trace along ray
        steps = int(distance_mm / self.cell_size) + 1
        for step in range(steps):
            ray_distance = step * self.cell_size
            x = start_x + dx * ray_distance
            y = start_y + dy * ray_distance
            node = self.get_cell(x, y)
            if not node:
                continue
            if ray_distance >= distance_mm - self.cell_size / 2:
                # at or beyond obstacle distance, mark as occupied
                if not node.is_origin:
                    node.mark_occupied()
                break
            else:
                # before obstacle, mark as free
                if not node.is_origin:
                    node.mark_free()

    def get_obstacle_distance(self, x, y, cardinal):
        '''
        return distance to nearest occupied node in given direction
        '''
        dx, dy = self._direction_vectors[cardinal]
        length = math.sqrt(dx * dx + dy * dy)
        dx /= length
        dy /= length
        distance = 0
        while distance < self.max_range:
            distance += self.cell_size
            check_x = x + dx * distance
            check_y = y + dy * distance
            node = self.get_cell(check_x, check_y)
            if not node:
                return self.max_range
            if node.state == 'occupied':
                return distance
        return self.max_range

    def get_obstacle_distances(self, x, y):
        '''
        return distances to nearest occupied node in all 8 cardinal directions
        '''
        result = {}
        for i in range(8):
            cardinal = Cardinal.from_index(i)
            result[cardinal] = self.get_obstacle_distance(x, y, cardinal)
        return result

    def get_clear_distance(self, x, y, cardinal):
        '''
        return distance through free space before hitting occupied or unknown
        '''
        dx, dy = self._direction_vectors[cardinal]
        length = math.sqrt(dx * dx + dy * dy)
        dx /= length
        dy /= length
        distance = 0
        while distance < self.max_range:
            distance += self.cell_size
            check_x = x + dx * distance
            check_y = y + dy * distance
            node = self.get_cell(check_x, check_y)
            if not node:
                return distance - self.cell_size
            if node.state != 'free':
                return distance - self.cell_size
        return self.max_range

    def get_clear_distances(self, x, y):
        '''
        return clear distances in all 8 cardinal directions
        '''
        result = {}
        for i in range(8):
            cardinal = Cardinal.from_index(i)
            result[cardinal] = self.get_clear_distance(x, y, cardinal)
        return result


# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class GridVisualiser:
    FREE_CELL_COLOR = "#E6F3FF"  # light blue background for free cells
    '''
    Generates SVG visualisation of the occupancy grid.
    '''
    def __init__(self, grid):
        self.grid = grid
        # scale factor to keep coordinates reasonable (1mm -> 0.1 SVG units)
        self._scale = 0.1

    def generate_svg(self, filename="grid_map-v5.svg"):
        '''
        generate SVG file showing the occupancy grid
        '''
        # calculate bounds in grid coordinates
        min_x     = -self.grid.max_range * self._scale
        max_x     = self.grid.max_range * self._scale
        min_y     = -self.grid.max_range * self._scale
        max_y     = self.grid.max_range * self._scale
        width     = max_x - min_x
        height    = max_y - min_y
        cell_size = self.grid.cell_size * self._scale
        # generate content and write to file
        with open(filename, "w") as file:
            file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            file.write('<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800" viewBox="0 0 {} {}">\n'.format(
                width, height))
            # draw background for free cells first
            file.write('<g>\n')
            for node in self.grid.cells.values():
                if node.state == 'free':
                    svg_x = (node.x * self._scale) - min_x
                    svg_y = (max_y - node.y * self._scale) - cell_size
                    file.write('<rect x="{}" y="{}" width="{}" height="{}" fill="{}" stroke="none"/>\n'.format(
                        svg_x, svg_y, cell_size, cell_size, self.FREE_CELL_COLOR))
            file.write('</g>\n')
            # draw occupied cells and origin
            file.write('<g>\n')
            for node in self.grid.cells.values():
                svg_x = (node.x * self._scale) - min_x
                svg_y = (max_y - node.y * self._scale) - cell_size
                if node.is_origin:
                    stroke, fill = "red", "red"
                    file.write('<rect x="{}" y="{}" width="{}" height="{}" stroke="{}" fill="{}" stroke-width="{}"/>\n'.format(
                        svg_x, svg_y, cell_size, cell_size, stroke, fill, 0.5 * self._scale))
                elif node.state == 'occupied':
                    distance_mm = math.sqrt(node.x * node.x + node.y * node.y)
                    rgb = self._color_for_distance(distance_mm)
                    fill = self._rgb_to_svg_color(rgb)
                    file.write('<rect x="{}" y="{}" width="{}" height="{}" stroke="black" fill="{}" stroke-width="{}"/>\n'.format(
                        svg_x, svg_y, cell_size, cell_size, fill, 0.5 * self._scale))
            file.write('</g>\n</svg>\n')

        occupied_count = sum(1 for c in self.grid.cells.values() if c.state == 'occupied')
        free_count = sum(1 for c in self.grid.cells.values() if c.state == 'free')
        print("SVG written to {}, occupied: {}, free: {}, total cells: {}".format(
            filename, occupied_count, free_count, len(self.grid.cells)))

    def _color_for_distance(self, distance_mm):
        '''
        Return RGB color for a given distance, matching NeoPixel ring logic.
        '''
        min_distance_mm = 50
        max_distance_mm = 4000
        if distance_mm <= min_distance_mm:
            return (255, 0, 0)
        ratio = distance_mm / max_distance_mm
        hue = ratio * 300
        return self._hsv_to_rgb(hue, 1.0, 1.0)

    def _hsv_to_rgb(self, h, s, v):
        '''
        Convert HSV color to RGB, matching the NeoPixel ring coloring.
        '''
        if s == 0:
            val = int(v * 255)
            return val, val, val
        h = h % 360
        h_sector = h // 60
        f = (h / 60) - h_sector
        p = v * (1 - s)
        q = v * (1 - s * f)
        t = v * (1 - s * (1 - f))
        if h_sector == 0:
            r, g, b = v, t, p
        elif h_sector == 1:
            r, g, b = q, v, p
        elif h_sector == 2:
            r, g, b = p, v, t
        elif h_sector == 3:
            r, g, b = p, q, v
        elif h_sector == 4:
            r, g, b = t, p, v
        else:
            r, g, b = v, p, q
        return int(r * 255), int(g * 255), int(b * 255)

    def _rgb_to_svg_color(self, rgb):
        '''
        Convert RGB tuple to SVG hex color string.
        '''
        return "#{:02x}{:02x}{:02x}".format(rgb[0], rgb[1], rgb[2])


# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python grid_map.py <x> <y> <8 sensor readings>")
        print("Example: python grid_map.py 0 0 0500 2320 1694 1859 3659 1675 1000 1722")
        print("         python grid_map.py 1000 0 0500 2320 1694 1859 3659 1675 1000 1722")
        print("Sensor order: N NE E SE S SW W NW")
        print("Position in mm, heading assumed north (0°)")
        sys.exit(1)

    svg_filename = "grid_map.svg"

    robot_x = int(sys.argv[1])
    robot_y = int(sys.argv[2])
    sensor_readings = " ".join(sys.argv[3:11])

    if len(sys.argv[3:11]) != 8:
        print("Error: expected exactly 8 sensor readings")
        sys.exit(1)

    grid = OccupancyGrid(cell_size=50, max_range=4000)
    grid.process_sensor_reading(sensor_readings, robot_x, robot_y, robot_heading=0)

    visualiser = GridVisualiser(grid)
    visualiser.generate_svg(svg_filename)

    print("\nObstacle distances from origin:")
    obstacles = grid.get_obstacle_distances(0, 0)
    for cardinal, distance in obstacles.items():
        print("  {}: {}mm".format(cardinal.label, distance))

    print("\nClear distances from origin:")
    clear = grid.get_clear_distances(0, 0)
    for cardinal, distance in clear.items():
        print("  {}: {}mm".format(cardinal.label, distance))

#EOF
