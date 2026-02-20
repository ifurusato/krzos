
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
class AdaptiveCell:
    '''
    Represents a single cell in the adaptive occupancy map.
    Can represent different resolution levels (50mm, 100mm, 200mm, etc.)
    '''
    def __init__(self, x_min, y_min, x_max, y_max):
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.size = x_max - x_min  # assumes square cells
        self.center_x = (x_min + x_max) / 2
        self.center_y = (y_min + y_max) / 2
        self.state = 'unknown'  # 'unknown', 'free', 'occupied', 'mixed'
        self.is_origin = False
        self.is_robot_position = False
        self.observation_count = 0      # total times this cell was observed
        self.occupied_count = 0         # times it was observed as occupied
        self.occupancy_probability = 0.0  # occupied_count / observation_count
        self.children = None  # None = leaf node, list = subdivided

    def is_leaf(self):
        return self.children is None

    def contains_point(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def subdivide(self):
        '''
        split this cell into 4 children at half the resolution
        '''
        if not self.is_leaf():
            return  # already subdivided
        
        mid_x = (self.x_min + self.x_max) / 2
        mid_y = (self.y_min + self.y_max) / 2
        
        self.children = [
            AdaptiveCell(self.x_min, self.y_min, mid_x, mid_y),      # SW
            AdaptiveCell(mid_x, self.y_min, self.x_max, mid_y),      # SE
            AdaptiveCell(self.x_min, mid_y, mid_x, self.y_max),      # NW
            AdaptiveCell(mid_x, mid_y, self.x_max, self.y_max)       # NE
        ]
        
        # propagate state to children if not mixed
        if self.state in ('free', 'occupied'):
            for child in self.children:
                child.state = self.state
        
        self.state = 'mixed'  # parent is now mixed

    def mark_free(self):
        if self.is_leaf():
            self.observation_count += 1
            if self.state == 'unknown':
                self.state = 'free'
            self._update_occupancy_probability()

    def mark_occupied(self):
        if self.is_leaf():
            self.observation_count += 1
            self.occupied_count += 1
            self.state = 'occupied'
            self._update_occupancy_probability()

    def _update_occupancy_probability(self):
        if self.observation_count > 0:
            self.occupancy_probability = self.occupied_count / self.observation_count

    def get_all_leaf_cells(self):
        '''
        recursively collect all leaf cells (for visualization)
        '''
        if self.is_leaf():
            return [self]
        else:
            leaves = []
            for child in self.children:
                leaves.extend(child.get_all_leaf_cells())
            return leaves

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class AdaptiveOccupancyMap:
    '''
    Adaptive resolution occupancy map using recursive spatial subdivision.
    High resolution (50mm) where robot has been, coarse resolution elsewhere.
    '''
    def __init__(self, min_cell_size=50, initial_size=8000):
        self.min_cell_size = min_cell_size  # finest resolution: 50mm
        self.initial_size = initial_size
        # root cell covers entire mapped area
        half = initial_size / 2
        self.root = AdaptiveCell(-half, -half, half, half)
        # we'll mark the origin node upon the first scan location rather than mark the entire quad tree
#       self.root.is_origin = True
        self.trim_bounds = None
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

    def _find_cell_for_point(self, x, y, required_resolution=None):
        '''
        find the cell containing point (x,y), optionally subdividing
        to reach required resolution
        '''
        if required_resolution is None:
            required_resolution = self.min_cell_size
        cell = self.root
        # traverse down tree
        while True:
            if not cell.contains_point(x, y):
                return None
            # if we've reached required resolution, stop
            if cell.size <= required_resolution:
                return cell
            # if leaf but too coarse, subdivide
            if cell.is_leaf():
                cell.subdivide()
            # descend to appropriate child
            for child in cell.children:
                if child.contains_point(x, y):
                    cell = child
                    break
            else:
                # shouldn't happen
                return cell

    def _expand_root_if_needed(self, x, y):
        '''
        expand root cell if point is outside current bounds
        '''
        if self.root.contains_point(x, y):
            return
        # save all leaf cells
        old_leaves = self.root.get_all_leaf_cells()
        # determine new size needed
        current_half = self.root.size / 2
        new_half = current_half
        while not (-new_half <= x <= new_half and -new_half <= y <= new_half):
            new_half *= 2
        # create new root
        self.root = AdaptiveCell(-new_half, -new_half, new_half, new_half)
        # reinsert all old data by directly subdividing and copying
        for leaf in old_leaves:
            if leaf.state == 'unknown' and not leaf.is_origin and not leaf.is_robot_position:
                continue  # skip empty cells
            # find or create cell at this position
            cell = self._find_cell_for_point(leaf.center_x, leaf.center_y, self.min_cell_size)
            if cell:
                cell.state = leaf.state
                cell.is_origin = leaf.is_origin
                cell.is_robot_position = leaf.is_robot_position
                cell.observation_count = leaf.observation_count
                cell.occupied_count = leaf.occupied_count
                cell.occupancy_probability = leaf.occupancy_probability

    def x_expand_root_if_needed(self, x, y):
        '''
        expand root cell if point is outside current bounds
        '''
        while not self.root.contains_point(x, y):
            # double the root size in all directions
            old_root = self.root
            new_size = old_root.size * 2
            half = new_size / 2
            # create new larger root
            self.root = AdaptiveCell(-half, -half, half, half)
            self.root.state = 'mixed'
            self.root.subdivide()
            # find which quadrant the old root belongs in and place it there
            for child in self.root.children:
                if child.contains_point(old_root.center_x, old_root.center_y):
                    # replace this child with the old root
                    self.root.children[self.root.children.index(child)] = old_root
                    break

    def get_cell(self, x, y):
        '''
        Return leaf cell at position, creating at minimum resolution if needed.
        '''
        self._expand_root_if_needed(x, y)
        return self._find_cell_for_point(x, y, self.min_cell_size)

    def process_sensor_reading(self, reading_string, robot_x=0, robot_y=0, robot_heading=0):
        '''
        Parse sensor reading string and update cells.
        reading_string: space-delimited string of 8 distance values in mm
        order: N NE E SE S SW W NW
        '''
        readings = [int(r) for r in reading_string.split()]
        if len(readings) != 8:
            raise ValueError("expected 8 sensor readings, got {}".format(len(readings)))
        
        # mark robot position
        robot_cell = self.get_cell(robot_x, robot_y)
        if robot_cell:
            robot_cell.is_robot_position = True
            robot_cell.mark_free()  # robot position is definitely free
            # mark origin on first reading at 0,0
            if robot_x == 0 and robot_y == 0 and robot_cell.observation_count == 1:
                robot_cell.is_origin = True
        
        for i in range(8):
            distance_mm = readings[i]
            if distance_mm in (0, 9999):
                continue
            cardinal = Cardinal.from_index(i)
            self._trace_ray(robot_x, robot_y, cardinal, distance_mm)

    def _trace_ray(self, start_x, start_y, cardinal, distance_mm):
        '''
        trace a ray from start position, marking cells as free or occupied
        '''
        dx, dy = self._direction_vectors[cardinal]
        # normalize direction vector
        length = math.sqrt(dx * dx + dy * dy)
        dx /= length
        dy /= length
        # trace at minimum resolution
        current_distance = 0
        while current_distance <= distance_mm:
            x = start_x + dx * current_distance
            y = start_y + dy * current_distance
            cell = self.get_cell(x, y)
            if not cell:
                current_distance += self.min_cell_size
                continue
            if current_distance + self.min_cell_size > distance_mm:
                # at obstacle distance, mark as occupied
                if not cell.is_origin:
                    cell.mark_occupied()
                break
            else:
                # before obstacle, mark as free
                if not cell.is_origin:
                    cell.mark_free()
            current_distance += self.min_cell_size

    def get_all_cells(self):
        '''
        return all leaf cells for visualization
        '''
        return self.root.get_all_leaf_cells()

    def get_original_bounds(self):
        '''
        Return current bounds of the map, if it hasn't been enlarged.
        '''
        return self.root.x_min, self.root.y_min, self.root.x_max, self.root.y_max

    def get_bounds(self):
        '''
        Return actual bounds of the map for visualization.
        '''
        if self.trim_bounds is not None:
            return self.trim_bounds
        cells = self.get_all_cells()
        if not cells:
            return self.root.x_min, self.root.y_min, self.root.x_max, self.root.y_max
        min_x = min(cell.x_min for cell in cells)
        max_x = max(cell.x_max for cell in cells)
        min_y = min(cell.y_min for cell in cells)
        max_y = max(cell.y_max for cell in cells)
        return min_x, min_y, max_x, max_y

    def trim_to_data(self, margin_mm=100):
        '''
        calculate tight bounds around non-unknown cells and apply margin
        '''
        cells = self.get_all_cells()
        # find cells with data (not unknown)
        data_cells = [c for c in cells if c.state != 'unknown']
        if not data_cells:
            return  # no data to trim to
        # find tight bounds
        min_x = min(cell.x_min for cell in data_cells)
        max_x = max(cell.x_max for cell in data_cells)
        min_y = min(cell.y_min for cell in data_cells)
        max_y = max(cell.y_max for cell in data_cells)
        # apply margin
        self.trim_bounds = (
            min_x - margin_mm,
            min_y - margin_mm,
            max_x + margin_mm,
            max_y + margin_mm
        )

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MapVisualizer:
    FREE_CELL_COLOR = "#E6F3FF"  # light blue background for free cells
    ROBOT_POSITION_COLOR = "#404040"  # dark gray for robot scan positions
    '''
    Generates SVG visualization of the adaptive occupancy map.
    '''
    def __init__(self, occupancy_map):
        self.map = occupancy_map
        # scale factor to keep coordinates reasonable (1mm -> 0.1 SVG units)
        self._scale = 0.1

    def generate_svg(self, filename="occupancy_map.svg"):
        '''
        generate SVG file showing the occupancy map
        '''
        x_min, y_min, x_max, y_max = self.map.get_bounds()
        
#       # calculate SVG bounds
#       min_x = x_min * self._scale
#       max_x = x_max * self._scale
#       min_y = y_min * self._scale
#       max_y = y_max * self._scale
#       width = max_x - min_x
#       height = max_y - min_y
        
        # calculate SVG bounds with Y-flip
        min_x = x_min * self._scale
        max_x = x_max * self._scale
        # flip Y: since we negate Y coords when writing rects, viewBox needs flipped bounds
        min_y = -y_max * self._scale
        max_y = -y_min * self._scale
        width = max_x - min_x
        height = max_y - min_y
        
        cells = self.map.get_all_cells()
        
        # generate content and write to file
        with open(filename, "w") as file:
            file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
            file.write('<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800" viewBox="{} {} {} {}">\n'.format(
                min_x, min_y, width, height))
            
            # draw free cells first
            file.write('<g id="free-cells">\n')
            for cell in cells:
                if cell.state == 'free':
                    self._write_cell_rect(file, cell, self.FREE_CELL_COLOR, "none")
            file.write('</g>\n')
            
            # draw occupied cells and origin
            file.write('<g id="occupied-cells">\n')
            for cell in cells:
                if cell.is_origin:
                    self._write_cell_rect(file, cell, "red", "red", stroke_width=0.5)
                elif cell.state == 'occupied':
                    distance_mm = math.sqrt(cell.center_x * cell.center_x + cell.center_y * cell.center_y)
                    rgb = self._color_for_distance(distance_mm)
                    fill = self._rgb_to_svg_color(rgb)
                    self._write_cell_rect(file, cell, fill, "black", stroke_width=0.5)
            file.write('</g>\n')
            
            # draw robot positions
            file.write('<g id="robot-positions">\n')
            for cell in cells:
                if cell.is_robot_position and not cell.is_origin:
                    self._write_cell_rect(file, cell, self.ROBOT_POSITION_COLOR, "black", stroke_width=0.5)
            file.write('</g>\n')
            
            file.write('</svg>\n')

        occupied_count = sum(1 for c in cells if c.state == 'occupied')
        free_count = sum(1 for c in cells if c.state == 'free')
        robot_positions = sum(1 for c in cells if c.is_robot_position)
        print("SVG written to {}, occupied: {}, free: {}, robot positions: {}, total cells: {}".format(
            filename, occupied_count, free_count, robot_positions, len(cells)))

    def _write_cell_rect(self, file, cell, fill, stroke, stroke_width=None):
        '''
        write a single cell rectangle to SVG
        '''
        x = cell.x_min * self._scale
        # flip Y coordinate: SVG Y increases downward, we want Y increasing upward
        y = -cell.y_max * self._scale
        w = cell.size * self._scale
        h = cell.size * self._scale
        
        if stroke_width:
            file.write('<rect x="{}" y="{}" width="{}" height="{}" fill="{}" stroke="{}" stroke-width="{}"/>\n'.format(
                x, y, w, h, fill, stroke, stroke_width * self._scale))
        else:
            file.write('<rect x="{}" y="{}" width="{}" height="{}" fill="{}" stroke="{}"/>\n'.format(
                x, y, w, h, fill, stroke))

    def _color_for_distance(self, distance_mm):
        '''
        return RGB color for a given distance
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
        convert HSV color to RGB
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
        convert RGB tuple to SVG hex color string
        '''
        return "#{:02x}{:02x}{:02x}".format(rgb[0], rgb[1], rgb[2])


# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python adaptive_occupancy_map.py <x> <y> <heading> <8 sensor readings>")
        print("Example: python adaptive_occupancy_map.py 0 0 0 0500 2320 1694 1859 3659 1675 1000 1722")
        print("Sensor order: N NE E SE S SW W NW")
        print("Position in mm, heading in degrees")
        sys.exit(1)

    robot_x = int(sys.argv[1])
    robot_y = int(sys.argv[2])
    robot_heading = int(sys.argv[3])
    sensor_readings = " ".join(sys.argv[4:12])

    if len(sys.argv[4:12]) != 8:
        print("Error: expected exactly 8 sensor readings")
        sys.exit(1)

    occupancy_map = AdaptiveOccupancyMap(min_cell_size=50, initial_size=8000)
    occupancy_map.process_sensor_reading(sensor_readings, robot_x, robot_y, robot_heading)

    visualizer = MapVisualizer(occupancy_map)
    visualizer.generate_svg("occupancy_map.svg")

#EOF
