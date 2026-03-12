#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-09
# modified: 2026-03-10
#
# Ground-truth floorplan model for robotic navigation.
#
# Loads a coordinated SVG + YAML pair and provides a unified query interface
# over rooms, doors, and landmarks (including SVG-only text objects).
#
# Intended to integrate with the kros config loader pattern:
#
#     _cfg = config['kros'].get('navigation').get('floorplan')
#     fp = Floorplan(_cfg)
#
# Or directly from file paths:
#
#     fp = Floorplan.from_files(svg_path, yaml_path)
#
# Query interface:
#     fp.query("MASTER")          -> RoomResult
#     fp.query("D1")              -> DoorResult
#     fp.query("BUNNY")           -> LandmarkResult
#
#     fp.initial_pose()           -> Pose
#     fp.route("ORIGIN", "BUNNY") -> RouteIterator
#     fp.adjacency()              -> {"Master": ["Corridor"], ...}
#     fp.rooms_as_rectangles()    -> [(id, Bounds), ...]
#     fp.walls()                  -> [Segment, ...]

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional
import yaml

# result types ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def __iter__(self):
        yield self.x
        yield self.y

    def __repr__(self):
        return "Point({}, {})".format(self.x, self.y)


@dataclass(frozen=True)
class Pose:
    '''
    A positioned, oriented robot pose.
    '''
    x: float
    y: float
    heading: float  # degrees, 0=north, 90=east

    @property
    def position(self) -> Point:
        return Point(self.x, self.y)

    def __repr__(self):
        return "Pose(x={}, y={}, heading={})".format(self.x, self.y, self.heading)


@dataclass(frozen=True)
class Bounds:
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @property
    def centre(self) -> Point:
        return Point((self.xmin + self.xmax) / 2, (self.ymin + self.ymax) / 2)

    @property
    def width(self) -> float:
        return self.xmax - self.xmin

    @property
    def height(self) -> float:
        return self.ymax - self.ymin

    def contains(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def __iter__(self):
        yield self.xmin
        yield self.xmax
        yield self.ymin
        yield self.ymax

    def __repr__(self):
        return "Bounds(xmin={}, xmax={}, ymin={}, ymax={})".format(
            self.xmin, self.xmax, self.ymin, self.ymax)


@dataclass(frozen=True)
class Segment:
    '''
    A wall segment or door opening defined by two endpoints.
    '''
    p1: Point
    p2: Point

    @property
    def midpoint(self) -> Point:
        return Point((self.p1.x + self.p2.x) / 2, (self.p1.y + self.p2.y) / 2)

    @property
    def length(self) -> float:
        return ((self.p2.x - self.p1.x) ** 2 + (self.p2.y - self.p1.y) ** 2) ** 0.5

    def __iter__(self):
        yield self.p1
        yield self.p2

    def __repr__(self):
        return "Segment({}, {})".format(self.p1, self.p2)


@dataclass(frozen=True)
class RoomResult:
    type: str = field(default="room", init=False)
    id: str
    bounds: list       # list of Bounds; single-rect rooms have one entry
    doors: list

    @property
    def bbox(self) -> Bounds:
        '''
        overall bounding box of all constituent bounds
        '''
        return Bounds(
            min(b.xmin for b in self.bounds),
            max(b.xmax for b in self.bounds),
            min(b.ymin for b in self.bounds),
            max(b.ymax for b in self.bounds),
        )

    @property
    def centre(self) -> Point:
        return self.bbox.centre

    def contains(self, x: float, y: float) -> bool:
        return any(b.contains(x, y) for b in self.bounds)

    def __repr__(self):
        return "RoomResult(id={!r}, bounds={}, doors={})".format(
            self.id, self.bounds, self.doors)


@dataclass(frozen=True)
class DoorResult:
    type: str = field(default="door", init=False)
    id: str
    segment: Segment
    connects: list
    traversable: bool
    width: int
    orientation: str

    @property
    def position(self) -> Point:
        return self.segment.midpoint

    def __repr__(self):
        return "DoorResult(id={!r}, segment={}, traversable={}, connects={})".format(
            self.id, self.segment, self.traversable, self.connects)


@dataclass(frozen=True)
class LandmarkResult:
    type: str = field(default="landmark", init=False)
    id: str
    position: Point
    room: Optional[str] = None
    notes: Optional[str] = None

    def __iter__(self):
        yield self.position.x
        yield self.position.y

    def __repr__(self):
        return "LandmarkResult(id={!r}, position={}, room={!r})".format(
            self.id, self.position, self.room)


#@dataclass
@dataclass(frozen=True)
class Waypoint:
    label: str
    position: Point
    kind: str                       # "landmark", "door", or "room"
    arrival_radius: float = 200.0   # mm
    heading: float = None           # optional target heading in degrees; None means don't care

    def reached(self, x: float, y: float) -> bool:
        '''
        Return True if (x, y) is within arrival_radius of this waypoint.
        '''
        dx = x - self.position.x
        dy = y - self.position.y
        return (dx * dx + dy * dy) ** 0.5 <= self.arrival_radius

    def __repr__(self):
        return "Waypoint({!r}, pos=({}, {}), kind={!r})".format(
            self.label, self.position.x, self.position.y, self.kind)


# route iterator ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class RouteIterator:
    '''
    Stateful iterator over a list of Waypoints.

    Intended for use by the robot's navigation behaviour loop:

        iterator = fp.route("ORIGIN", "BUNNY")
        # called periodically:
        if iterator.current.reached(robot_x, robot_y):
            iterator.advance()
        if iterator.completed:
            # arrived at destination
    '''

    def __init__(self, waypoints: list):
        if not waypoints:
            raise ValueError("RouteIterator requires at least one waypoint")
        self._waypoints = list(waypoints)
        self._index = 0

    @property
    def current(self) -> Waypoint:
        '''
        The waypoint the robot is currently driving toward.
        '''
        return self._waypoints[self._index]

    @property
    def completed(self) -> bool:
        '''
        True when the final waypoint has been reached and advanced past.
        '''
        return self._index >= len(self._waypoints)

    @property
    def remaining(self) -> int:
        '''
        Number of waypoints not yet reached, including current.
        '''
        return max(0, len(self._waypoints) - self._index)

    @property
    def index(self) -> int:
        '''
        Index of the current waypoint.
        '''
        return self._index

    @property
    def all_waypoints(self) -> list:
        '''
        All waypoints in the route, regardless of current position.
        '''
        return list(self._waypoints)

    def advance(self):
        '''
        Mark the current waypoint as reached and move to the next.
        Safe to call when already completed.
        '''
        if not self.completed:
            self._index += 1

    def save(self, path: str):
        '''
        Save this route to a YAML file.
        '''
        import yaml
        _data = {
            'route': [
                {
                    'label':          wp.label,
                    'kind':           wp.kind,
                    'position':       {'x': wp.position.x, 'y': wp.position.y},
                    'arrival_radius': wp.arrival_radius,
                } for wp in self._waypoints
            ]
        }
        with open(path, 'w') as f:
            yaml.dump(_data, f, default_flow_style=False)

    def reset(self):
        '''
        Restart the route from the beginning.
        '''
        self._index = 0

    def __repr__(self):
        if self.completed:
            return "RouteIterator(completed, {} waypoints)".format(
                len(self._waypoints))
        return "RouteIterator({}/{}: {})".format(
            self._index + 1, len(self._waypoints), self.current)


# SVG parser ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# static utility class for SVG coordinate extraction

class SVGParser:
    '''
    Static utility methods for parsing SVG files and extracting
    text object positions in floorplan coordinates.
    '''

    _DOOR_RE = re.compile(r'^D\d+$', re.IGNORECASE)

    @staticmethod
    def parse_mm(value: str) -> float:
        '''
        Parse an SVG length attribute into millimetres.
        '''
        value = value.strip()
        for unit, factor in [("mm", 1), ("cm", 10), ("in", 25.4),
                              ("pt", 25.4 / 72), ("px", 25.4 / 96)]:
            if value.endswith(unit):
                return float(value[: -len(unit)]) * factor
        return float(value)

    @staticmethod
    def apply_transform(x: float, y: float, transform: str) -> tuple:
        '''
        Apply a single SVG transform string to point (x, y).
        '''
        m = re.search(
            r"matrix\(\s*([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)[\s,]+"
            r"([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)\s*\)", transform)
        if m:
            a, b, c, d, e, f = (float(v) for v in m.groups())
            return a * x + c * y + e, b * x + d * y + f

        m = re.search(
            r"scale\(\s*([^\s,)]+)(?:[\s,]+([^\s,)]+))?\s*\)", transform)
        if m:
            sx = float(m.group(1))
            sy = float(m.group(2)) if m.group(2) else sx
            return x * sx, y * sy

        m = re.search(
            r"translate\(\s*([^\s,)]+)(?:[\s,]+([^\s,)]+))?\s*\)", transform)
        if m:
            tx = float(m.group(1))
            ty = float(m.group(2)) if m.group(2) else 0.0
            return x + tx, y + ty

        return x, y

    @staticmethod
    def find_path(node, goal, path: list) -> Optional[list]:
        '''
        Return the path of elements from node to goal, or None.
        '''
        if node is goal:
            return path + [node]
        for child in node:
            result = SVGParser.find_path(child, goal, path + [node])
            if result:
                return result
        return None

    @staticmethod
    def get_xy(elem) -> tuple:
        '''
        Return (x, y) from elem or its first child carrying both attributes.
        '''
        for node in [elem, *elem.iter()]:
            x, y = node.get("x"), node.get("y")
            if x is not None and y is not None:
                return float(x), float(y)
        return None, None

    @staticmethod
    def extract_landmarks(svg_path: str) -> dict:
        '''
        Parse all <text> elements from an SVG file.
        Returns a dict of uppercased name -> Point in floorplan mm coordinates,
        with Y axis flipped so that Y increases northward.
        '''
        tree = ET.parse(svg_path)
        root = tree.getroot()

        vb = root.get("viewBox", "")
        w_attr = root.get("width", "")
        if not vb:
            raise ValueError("SVG has no viewBox attribute")

        _, _, vb_w, vb_h = (float(p) for p in vb.strip().replace(",", " ").split())
        uu_per_mm = vb_w / SVGParser.parse_mm(w_attr) if w_attr else 1.0

        landmarks = {}
        for elem in root.iter():
            if elem.tag.split("}")[-1] != "text":
                continue
            # only top-level <text> elements (direct children of root)
            if elem not in root:
                continue
            name = "".join(elem.itertext()).strip()
            if not name:
                continue
            local_x, local_y = SVGParser.get_xy(elem)
            if local_x is None:
                continue
            path = SVGParser.find_path(root, elem, [])
            if path is None:
                continue
            x, y = local_x, local_y
            for ancestor in reversed(path[1:-1]):
                t = ancestor.get("transform")
                if t:
                    x, y = SVGParser.apply_transform(x, y, t)
            landmarks[name.upper()] = Point(
                round(x / uu_per_mm),
                round((vb_h - y) / uu_per_mm)
            )
        return landmarks

    @staticmethod
    def _apply_group_matrix(x, y, w, h, transform):
        '''
        apply an SVG transform to a rect defined by (x, y, w, h),
        returning (xmin, ymin, xmax, ymax) in the parent coordinate space.
        handles matrix(), scale(), and rotate() forms used in this SVG.
        transforms all four corners and takes min/max to handle rotations.
        '''
        import math

        t = transform.strip()

        # matrix(a,b,c,d,e,f)
        m = re.match(
            r'matrix\(\s*([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)[\s,]+'
            r'([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)\s*\)', t)
        if m:
            a, b, c, d, e, f = (float(v) for v in m.groups())
            corners = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
            xs = [a*cx + c*cy + e for cx, cy in corners]
            ys = [b*cx + d*cy + f for cx, cy in corners]
            return min(xs), min(ys), max(xs), max(ys)

        # scale(sx) or scale(sx,sy)
        m = re.match(r'scale\(\s*([^\s,)]+)(?:[\s,]+([^\s,)]+))?\s*\)', t)
        if m:
            sx = float(m.group(1))
            sy = float(m.group(2)) if m.group(2) else sx
            corners = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
            xs = [sx * cx for cx, cy in corners]
            ys = [sy * cy for cx, cy in corners]
            return min(xs), min(ys), max(xs), max(ys)

        # rotate(angle) — origin rotation, degrees
        m = re.match(r'rotate\(\s*([^\s,)]+)\s*\)', t)
        if m:
            angle = math.radians(float(m.group(1)))
            cos_a, sin_a = math.cos(angle), math.sin(angle)
            corners = [(x, y), (x+w, y), (x, y+h), (x+w, y+h)]
            xs = [cos_a*cx - sin_a*cy for cx, cy in corners]
            ys = [sin_a*cx + cos_a*cy for cx, cy in corners]
            return min(xs), min(ys), max(xs), max(ys)

        return None

    @staticmethod
    def x_apply_group_matrix(x, y, w, h, transform):
        '''
        apply a matrix(a,b,c,d,e,f) group transform to a rect defined by
        (x, y, w, h) in local coords, returning (xmin, ymin, xmax, ymax)
        in document coords. transforms all four corners and takes min/max
        to handle rotated rects correctly.
        '''
        m = re.match(
            r'matrix\(\s*([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)[\s,]+'
            r'([^\s,]+)[\s,]+([^\s,]+)[\s,]+([^\s,]+)\s*\)',
            transform.strip())
        if not m:
            return None
        a, b, c, d, e, f = (float(v) for v in m.groups())
        corners = [
            (x,     y),
            (x + w, y),
            (x,     y + h),
            (x + w, y + h),
        ]
        xs = [a * cx + c * cy + e for cx, cy in corners]
        ys = [b * cx + d * cy + f for cx, cy in corners]
        return min(xs), min(ys), max(xs), max(ys)

    @staticmethod
    def _extract_groups(svg_path):
        '''
        parse all named <g> elements and unlabelled top-level <rect> elements.
        returns:
          groups   : list of {'label': str, 'rects': [(xmin, ymin, xmax, ymax)]}
                     in floorplan mm coords (Y increasing northward)
          obstacles: list of (xmin, ymin, xmax, ymax) in floorplan mm coords
        '''
        tree = ET.parse(svg_path)
        root = tree.getroot()

        vb = root.get('viewBox', '')
        w_attr = root.get('width', '')
        _, _, vb_w, vb_h = (float(p) for p in vb.strip().replace(',', ' ').split())
        uu_per_mm = vb_w / SVGParser.parse_mm(w_attr) if w_attr else 1.0
        doc_h = vb_h / uu_per_mm  # document height in mm

        def flip_y(ymin_svg, ymax_svg):
            return doc_h - ymax_svg, doc_h - ymin_svg

        def r2_v0(v):
            return round(v)

        def r2(v):
            '''round to nearest 50, then snap to nearest 100 if within 2'''
            r = round(v)
            nearest_100 = round(r / 100) * 100
            if abs(r - nearest_100) <= 2:
                return nearest_100
            return r

        def resolve_rect(rx, ry, rw, rh, inner_t, group_t):
            '''
            apply inner rect transform then group transform,
            returning (xmin, ymin, xmax, ymax) in SVG doc units
            '''
            if inner_t:
                result = SVGParser._apply_group_matrix(rx, ry, rw, rh, inner_t)
                if result is None:
                    return None
                lx0, ly0, lx1, ly1 = result
            else:
                lx0, ly0, lx1, ly1 = rx, ry, rx + rw, ry + rh
            if group_t:
                result = SVGParser._apply_group_matrix(
                    lx0, ly0, lx1 - lx0, ly1 - ly0, group_t)
                if result is None:
                    return None
                return result
            return lx0, ly0, lx1, ly1

        groups = []
        obstacles = []

        for elem in root:
            local = elem.tag.split('}')[-1]

            if local == 'defs':
                continue

            if local == 'g':
                group_t = elem.get('transform', '')
                label = None

                # text label from direct <text> children
                for child in elem:
                    if child.tag.split('}')[-1] == 'text':
                        text = ''.join(child.itertext()).strip()
                        if text:
                            label = ' '.join(text.split()).upper()
                            break

                # aria-label fallback for path-rendered text (e.g. MASTER)
                if label is None:
                    for child in elem:
                        al = child.get('aria-label')
                        if al:
                            label = al.strip().upper()
                            break

                if label is None:
                    continue

                rects = []
                for child in elem:
                    if child.tag.split('}')[-1] != 'rect':
                        continue
                    rx = float(child.get('x', 0))
                    ry = float(child.get('y', 0))
                    rw = float(child.get('width', 0))
                    rh = float(child.get('height', 0))
                    inner_t = child.get('transform', '')
                    coords = resolve_rect(rx, ry, rw, rh, inner_t, group_t)
                    if coords is None:
                        continue
                    sx0, sy0, sx1, sy1 = coords
                    sx0 /= uu_per_mm
                    sy0 /= uu_per_mm
                    sx1 /= uu_per_mm
                    sy1 /= uu_per_mm
                    ymin_fp, ymax_fp = flip_y(sy0, sy1)
                    rects.append((
                        r2(min(sx0, sx1)), r2(ymin_fp),
                        r2(max(sx0, sx1)), r2(ymax_fp),
                    ))

                if rects:
                    groups.append({'label': label, 'rects': rects})

            elif local == 'rect':
                # top-level unlabelled rect → obstacle
                rx = float(elem.get('x', 0))
                ry = float(elem.get('y', 0))
                rw = float(elem.get('width', 0))
                rh = float(elem.get('height', 0))
                inner_t = elem.get('transform', '')
                coords = resolve_rect(rx, ry, rw, rh, inner_t, '')
                if coords is None:
                    continue
                sx0, sy0, sx1, sy1 = coords
                sx0 /= uu_per_mm
                sy0 /= uu_per_mm
                sx1 /= uu_per_mm
                sy1 /= uu_per_mm
                ymin_fp, ymax_fp = flip_y(sy0, sy1)
                obstacles.append((
                    r2(min(sx0, sx1)), r2(ymin_fp),
                    r2(max(sx0, sx1)), r2(ymax_fp),
                ))

        return groups, obstacles

    @staticmethod
    def export_yaml(svg_path, output_path):
        '''
        generate a YAML file from the SVG, extracting rooms, doors,
        obstacles, and landmarks. door connects and orientation fields
        are derived where possible and marked TODO for manual verification.
        '''
        import datetime

        groups, obstacles = SVGParser._extract_groups(svg_path)
        landmarks = SVGParser.extract_landmarks(svg_path)

        rooms = [g for g in groups if not SVGParser._DOOR_RE.match(g['label'])]
        doors = [g for g in groups if SVGParser._DOOR_RE.match(g['label'])]

        # build room bounds lookup for spatial door-connectivity queries
        room_bounds = {}
        for r in rooms:
            room_bounds[r['label']] = [
                Bounds(x0, x1, y0, y1) for x0, y0, x1, y1 in r['rects']
            ]

        def rooms_adjacent_to_door(door_rects):
            x0, y0, x1, y1 = door_rects[0]
            margin = 200
            # expand the door rect by margin on all sides to ensure it
            # overlaps both rooms it separates
            dx0, dy0, dx1, dy1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
            adjacent = []
            for rname, rects in room_bounds.items():
                for b in rects:
                    if (dx0 < b.xmax and dx1 > b.xmin and
                            dy0 < b.ymax and dy1 > b.ymin):
                        adjacent.append(rname.capitalize())
                        break
            return adjacent

        def door_orientation(door_rects):
            x0, y0, x1, y1 = door_rects[0]
            if abs(y1 - y0) >= abs(x1 - x0):
                return 'east'   # long axis N-S → door on E-W wall
            else:
                return 'north'  # long axis E-W → door on N-S wall

        # build room -> door membership by inverting adjacency
        room_doors = {r['label']: [] for r in rooms}
        for d in doors:
            adjacent = rooms_adjacent_to_door(d['rects'])
            for room_label in adjacent:
                key = room_label.upper()
                if key in room_doors and d['label'] not in room_doors[key]:
                    room_doors[key].append(d['label'])

        lines = []
        lines.append('# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
        lines.append('#              House floorplan model for robotic navigation')
        lines.append('#')
        lines.append('# generated: {}'.format(
            datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        lines.append('# source:    {}'.format(svg_path))
        lines.append('# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
        lines.append('')
        lines.append('origin:')
        lines.append('  x: 0')
        lines.append('  y: 0')
        lines.append('')
        lines.append('axes:')
        lines.append('  X: {range: "East-West", positive: "East"}')
        lines.append('  Y: {range: "North-South", positive: "North"}')
        lines.append('')
        lines.append('rooms:')
        for r in rooms:
            lines.append('  - id: {}'.format(r['label'].capitalize()))
            if len(r['rects']) == 1:
                x0, y0, x1, y1 = r['rects'][0]
                lines.append('    bounds:')
                lines.append('      xmin: {}'.format(x0))
                lines.append('      xmax: {}'.format(x1))
                lines.append('      ymin: {}'.format(y0))
                lines.append('      ymax: {}'.format(y1))
            else:
                lines.append('    bounds:')
                for x0, y0, x1, y1 in r['rects']:
                    lines.append('      - {{xmin: {}, xmax: {}, ymin: {}, ymax: {}}}'.format(
                        x0, x1, y0, y1))
            door_ids = room_doors.get(r['label'], [])
            if door_ids:
                lines.append('    doors:')
                for did in door_ids:
                    lines.append('      - {}'.format(did))
            else:
                lines.append('    doors: []')
            lines.append('')
        lines.append('doors:')
        for d in doors:
            x0, y0, x1, y1 = d['rects'][0]
            door_width = round(max(abs(x1 - x0), abs(y1 - y0)))
            cx = round((x0 + x1) / 2)
            cy = round((y0 + y1) / 2)
            orientation = door_orientation(d['rects'])
            adjacent = rooms_adjacent_to_door(d['rects'])
            connects_str = '[{}]  # TODO: verify'.format(
                ', '.join(adjacent) if adjacent else 'null, null')
            lines.append('  - id: {}'.format(d['label']))
            lines.append('    connects: {}'.format(connects_str))
            lines.append('    traversable: true')
            lines.append('    position: {{x: {}, y: {}}}'.format(cx, cy))
            lines.append('    width: {}'.format(door_width))
            lines.append('    orientation: {}  # TODO: verify'.format(orientation))
            lines.append('')
        if obstacles:
            lines.append('obstacles:')
            for i, (x0, y0, x1, y1) in enumerate(obstacles):
                lines.append('  - id: OBS_{}'.format(i + 1))
                lines.append('    bounds:')
                lines.append('      xmin: {}'.format(x0))
                lines.append('      xmax: {}'.format(x1))
                lines.append('      ymin: {}'.format(y0))
                lines.append('      ymax: {}'.format(y1))
                lines.append('')
        if landmarks:
            lines.append('landmarks:')
            for name, point in sorted(landmarks.items()):
                lines.append('  - id: {}'.format(name))
                lines.append('    position: {{x: {}, y: {}}}'.format(
                    point.x, point.y))
                lines.append('    room: null  # TODO: verify')
                lines.append('')

        with open(output_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

    @staticmethod
    def x_export_yaml(svg_path, output_path):
        '''
        generate a YAML file from the SVG, extracting rooms, doors,
        obstacles, and landmarks. door connects and orientation fields
        are derived where possible and marked TODO for manual verification.
        '''
        import datetime

        groups, obstacles = SVGParser._extract_groups(svg_path)
        landmarks = SVGParser.extract_landmarks(svg_path)

        rooms = [g for g in groups if not SVGParser._DOOR_RE.match(g['label'])]
        doors = [g for g in groups if SVGParser._DOOR_RE.match(g['label'])]

        # build room bounds lookup for spatial door-connectivity queries
        room_bounds = {}
        for r in rooms:
            room_bounds[r['label']] = [
                Bounds(x0, x1, y0, y1) for x0, y0, x1, y1 in r['rects']
            ]

        def rooms_adjacent_to_door(door_rects):
            x0, y0, x1, y1 = door_rects[0]
            margin = 200
            # expand the door rect by margin on all sides to ensure it
            # overlaps both rooms it separates
            dx0, dy0, dx1, dy1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
            adjacent = []
            for rname, rects in room_bounds.items():
                for b in rects:
                    if (dx0 < b.xmax and dx1 > b.xmin and
                            dy0 < b.ymax and dy1 > b.ymin):
                        adjacent.append(rname.capitalize())
                        break
            return adjacent

        def door_orientation(door_rects):
            x0, y0, x1, y1 = door_rects[0]
            if abs(y1 - y0) >= abs(x1 - x0):
                return 'east'   # long axis N-S → door on E-W wall
            else:
                return 'north'  # long axis E-W → door on N-S wall

        lines = []
        lines.append('# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
        lines.append('#              House floorplan model for robotic navigation')
        lines.append('#')
        lines.append('# generated: {}'.format(
            datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
        lines.append('# source:    {}'.format(svg_path))
        lines.append('# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈')
        lines.append('')
        lines.append('origin:')
        lines.append('  x: 0')
        lines.append('  y: 0')
        lines.append('')
        lines.append('axes:')
        lines.append('  X: {range: "East-West", positive: "East"}')
        lines.append('  Y: {range: "North-South", positive: "North"}')
        lines.append('')
        lines.append('rooms:')
        for r in rooms:
            lines.append('  - id: {}'.format(r['label'].capitalize()))
            if len(r['rects']) == 1:
                x0, y0, x1, y1 = r['rects'][0]
                lines.append('    bounds:')
                lines.append('      xmin: {}'.format(x0))
                lines.append('      xmax: {}'.format(x1))
                lines.append('      ymin: {}'.format(y0))
                lines.append('      ymax: {}'.format(y1))
            else:
                lines.append('    bounds:')
                for x0, y0, x1, y1 in r['rects']:
                    lines.append('      - {{xmin: {}, xmax: {}, ymin: {}, ymax: {}}}'.format(
                        x0, x1, y0, y1))
            lines.append('    doors: []  # TODO: add door ids')
            lines.append('')
        lines.append('doors:')
        for d in doors:
            x0, y0, x1, y1 = d['rects'][0]
            door_width = round(max(abs(x1 - x0), abs(y1 - y0)))
            cx = round((x0 + x1) / 2)
            cy = round((y0 + y1) / 2)
            orientation = door_orientation(d['rects'])
            adjacent = rooms_adjacent_to_door(d['rects'])
            connects_str = '[{}]  # TODO: verify'.format(
                ', '.join(adjacent) if adjacent else 'null, null')
            lines.append('  - id: {}'.format(d['label']))
            lines.append('    connects: {}'.format(connects_str))
            lines.append('    traversable: true')
            lines.append('    position: {{x: {}, y: {}}}'.format(cx, cy))
            lines.append('    width: {}'.format(door_width))
            lines.append('    orientation: {}  # TODO: verify'.format(orientation))
            lines.append('')
        if obstacles:
            lines.append('obstacles:')
            for i, (x0, y0, x1, y1) in enumerate(obstacles):
                lines.append('  - id: OBS_{}'.format(i + 1))
                lines.append('    bounds:')
                lines.append('      xmin: {}'.format(x0))
                lines.append('      xmax: {}'.format(x1))
                lines.append('      ymin: {}'.format(y0))
                lines.append('      ymax: {}'.format(y1))
                lines.append('')
        if landmarks:
            lines.append('landmarks:')
            for name, point in sorted(landmarks.items()):
                lines.append('  - id: {}'.format(name))
                lines.append('    position: {{x: {}, y: {}}}'.format(
                    point.x, point.y))
                lines.append('    room: null  # TODO: verify')
                lines.append('')

        with open(output_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')


# door geometry ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# static utility class for door segment computation

class DoorGeometry:
    '''
    Static utility methods for computing door geometry from YAML data.
    '''

    @staticmethod
    def segment_from_yaml(position: dict, width: int, orientation: str) -> Segment:
        '''
        Compute a door's wall segment from its YAML position, width,
        and orientation.

        Orientation is the direction the door faces into:
          east/west   -> door sits on a N-S wall; segment runs N-S
          north/south -> door sits on an E-W wall; segment runs E-W
        '''
        px, py = position["x"], position["y"]
        half = width / 2
        if orientation in ("east", "west"):
            return Segment(Point(px, py - half), Point(px, py + half))
        else:
            return Segment(Point(px - half, py), Point(px + half, py))

# floorplan validator ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
# validates loaded data for internal consistency

class FloorplanValidator:
    '''
    Validates a loaded floorplan for internal consistency.
    Raises ValueError listing all problems found, so every issue is
    reported in a single pass rather than failing on the first.
    '''

    @staticmethod
    def validate(rooms: dict, doors: dict, landmarks: dict):
        errors = []

        # every traversable door's connected rooms must exist
        # (non-traversable doors are treated as walls; their connects is documentation only)
        for door in doors.values():
            if not door.traversable:
                continue
            for room_name in door.connects:
                if room_name.upper() not in rooms:
                    errors.append(
                        "Door {!r} connects to unknown room {!r}".format(
                            door.id, room_name))

        # every door id referenced by a room must exist
        for room in rooms.values():
            for door_id in room.doors:
                if door_id.upper() not in doors:
                    errors.append(
                        "Room {!r} references unknown door {!r}".format(
                            room.id, door_id))

        # every YAML landmark claiming a room must fall within its bounds
        for lm in landmarks.values():
            if lm.room is None:
                continue
            room_key = lm.room.upper()
            if room_key not in rooms:
                errors.append(
                    "Landmark {!r} claims unknown room {!r}".format(
                        lm.id, lm.room))
            else:
                room = rooms[room_key]
                if not room.contains(lm.position.x, lm.position.y):
                    errors.append(
                        "Landmark {!r} at ({}, {}) is outside its "
                        "claimed room {!r} {}".format(
                            lm.id, lm.position.x, lm.position.y,
                            lm.room, room.bounds))

        # every traversable door must lie on the boundary between its rooms
        for door in doors.values():
            if not door.traversable or len(door.connects) != 2:
                continue
            a_key, b_key = door.connects[0].upper(), door.connects[1].upper()
            if a_key not in rooms or b_key not in rooms:
                continue  # already reported above
            ba, bb = rooms[a_key].bbox, rooms[b_key].bbox
            mid = door.segment.midpoint
            combined = Bounds(
                min(ba.xmin, bb.xmin), max(ba.xmax, bb.xmax),
                min(ba.ymin, bb.ymin), max(ba.ymax, bb.ymax),
            )
            if not combined.contains(mid.x, mid.y):
                errors.append(
                    "Door {!r} midpoint ({}, {}) does not lie between "
                    "rooms {!r} and {!r}".format(
                        door.id, mid.x, mid.y,
                        door.connects[0], door.connects[1]))

        if errors:
            raise ValueError(
                "Floorplan validation failed with {} error(s):\n  {}".format(
                    len(errors), "\n  ".join(errors)))

# floorplan ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

class Floorplan:
    '''
    Ground-truth floorplan model for robotic navigation.

    Accepts a config dict (kros config loader pattern) or use
    Floorplan.from_files(svg_path, yaml_path) for direct construction.

    The YAML is the authoritative source for rooms, doors, and named
    landmarks. The SVG supplies additional text object positions
    (BUNNY, ORIGIN, etc.) not present in the YAML.
    '''

    def __init__(self, cfg: dict):
        '''
        Construct from a config dict, e.g.:
            cfg = config['kros']['navigation']['floorplan']
        Expected keys: 'svg', 'yaml', and optionally 'initial_pose'.
        '''
        self._svg_path  = cfg['svg']
        self._yaml_path = cfg['yaml']
        self._pose_cfg  = cfg.get('initial_pose', None)

        with open(self._yaml_path) as f:
            self._yaml = yaml.safe_load(f)

        self._rooms     = {}
        self._doors     = {}
        self._landmarks = {}

        self._load_yaml()
        self._load_svg_landmarks()
        FloorplanValidator.validate(self._rooms, self._doors, self._landmarks)

    @classmethod
    def from_files(cls, svg_path: str, yaml_path: str,
                   initial_pose: dict = None) -> 'Floorplan':
        '''
        Convenience constructor from explicit file paths.
        '''
        cfg = {'svg': svg_path, 'yaml': yaml_path}
        if initial_pose:
            cfg['initial_pose'] = initial_pose
        return cls(cfg)

    # YAML loading ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _load_yaml(self):
        y = self._yaml

        room_entries = list(y.get("rooms", []))
        if "corridor" in y:
            room_entries.append(y["corridor"])

        for r in room_entries:
            b = r["bounds"]
            # bounds may be a single dict or a list of dicts
            if isinstance(b, list):
                bounds_list = [
                    Bounds(e["xmin"], e["xmax"], e["ymin"], e["ymax"]) for e in b
                ]
            else:
                bounds_list = [Bounds(b["xmin"], b["xmax"], b["ymin"], b["ymax"])]
            room = RoomResult(
                id=r["id"],
                bounds=bounds_list,
                doors=r.get("doors", []),
            )
            self._rooms[r["id"].upper()] = room

        for d in y.get("doors", []):
            seg = DoorGeometry.segment_from_yaml(
                d["position"], d["width"], d["orientation"])
            door = DoorResult(
                id=d["id"],
                segment=seg,
                connects=d.get("connects", []),
                traversable=d.get("traversable", True),
                width=d["width"],
                orientation=d["orientation"],
            )
            self._doors[d["id"].upper()] = door

        for lm in y.get("landmarks", []):
            p = lm["position"]
            landmark = LandmarkResult(
                id=lm["id"],
                position=Point(p["x"], p["y"]),
                room=lm.get("room"),
                notes=lm.get("notes"),
            )
            self._landmarks[lm["id"].upper()] = landmark

    def _load_svg_landmarks(self):
        '''
        Add SVG text objects not already present in the YAML landmarks.
        YAML takes precedence for anything defined in both.
        '''
        svg_points = SVGParser.extract_landmarks(self._svg_path)
        for name, point in svg_points.items():
            if name not in self._landmarks:
                self._landmarks[name] = LandmarkResult(
                    id=name,
                    position=point,
                    room=self._room_id_at(point.x, point.y),
                )

    # query interface ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def query(self, name: str):
        '''
        Look up a name (case-insensitive) across rooms, doors, and landmarks.
        Returns RoomResult, DoorResult, or LandmarkResult.
        Raises KeyError if not found.
        '''
        key = name.strip().upper()
        if key in self._rooms:
            return self._rooms[key]
        if key in self._doors:
            return self._doors[key]
        if key in self._landmarks:
            return self._landmarks[key]
        raise KeyError("No room, door, or landmark named {!r}".format(name))

    def __getitem__(self, name: str):
        return self.query(name)

    # pose ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def initial_pose(self) -> Pose:
        '''
        Return the configured initial robot pose.

        Reads from the 'initial_pose' key in the config dict if present,
        otherwise falls back to the ORIGIN landmark position with heading 0.
        Raises KeyError if neither source is available.
        '''
        if self._pose_cfg:
            return Pose(
                x=self._pose_cfg['x'],
                y=self._pose_cfg['y'],
                heading=self._pose_cfg.get('heading', 0.0),
            )
        if 'ORIGIN' in self._landmarks:
            lm = self._landmarks['ORIGIN']
            return Pose(x=lm.position.x, y=lm.position.y, heading=0.0)
        raise KeyError(
            "No initial_pose in config and no ORIGIN landmark found")

    # spatial helpers ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _room_id_at(self, x: float, y: float) -> Optional[str]:
        '''
        Return the id of the room containing (x, y), or None.
        '''
        for room in self._rooms.values():
            if room.contains(x, y):
                return room.id
        return None

    def room_at(self, x: float, y: float) -> Optional[RoomResult]:
        '''
        Return the RoomResult containing (x, y), or None.
        '''
        for room in self._rooms.values():
            if room.contains(x, y):
                return room
        return None

    def closest_landmark(self, x: float, y: float) -> Optional[LandmarkResult]:
        '''
        Return the nearest landmark to (x, y), or None if none exist.
        '''
        best, best_dist = None, float('inf')
        for lm in self._landmarks.values():
            dx = lm.position.x - x
            dy = lm.position.y - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < best_dist:
                best, best_dist = lm, dist
        return best

    def doors_in_room(self, room_name: str) -> list:
        '''
        Return full DoorResult objects for all doors in the named room.
        '''
        key = room_name.strip().upper()
        if key not in self._rooms:
            raise KeyError("Unknown room {!r}".format(room_name))
        room = self._rooms[key]
        return [self._doors[d.upper()] for d in room.doors
                if d.upper() in self._doors]

    # routing ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def _resolve_to_room_and_point(self, name: str) -> tuple:
        '''
        Resolve a name to (room_id, Point). Raises KeyError if not found.
        '''
        key = name.strip().upper()

        if key in self._rooms:
            room = self._rooms[key]
            return room.id, room.centre

        if key in self._doors:
            door = self._doors[key]
            for room_id in door.connects:
                if room_id.upper() in self._rooms:
                    return room_id, door.segment.midpoint
            raise KeyError(
                "Door {!r} connects to unknown rooms".format(name))

        if key in self._landmarks:
            lm = self._landmarks[key]
            room_id = lm.room or self._room_id_at(lm.position.x, lm.position.y)
            if room_id is None:
                raise KeyError(
                    "Landmark {!r} is not inside any known room".format(name))
            return room_id, lm.position

        raise KeyError("No room, door, or landmark named {!r}".format(name))

    def _bfs_rooms(self, start_room: str, end_room: str) -> list:
        '''
        BFS over the traversable room adjacency graph.
        Returns ordered list of room ids from start to end inclusive.
        Raises ValueError if no path exists.
        '''
        if start_room == end_room:
            return [start_room]
        graph = self.adjacency()
        queue = [[start_room]]
        visited = {start_room}
        while queue:
            path = queue.pop(0)
            for neighbour in graph.get(path[-1], []):
                if neighbour not in visited:
                    new_path = path + [neighbour]
                    if neighbour == end_room:
                        return new_path
                    visited.add(neighbour)
                    queue.append(new_path)
        raise ValueError(
            "No traversable path from {!r} to {!r}".format(
                start_room, end_room))

    def _door_between(self, room_a: str, room_b: str) -> Optional[DoorResult]:
        '''
        Return the traversable door connecting room_a and room_b, or None.
        '''
        for door in self._doors.values():
            if not door.traversable:
                continue
            connects = [c.upper() for c in door.connects]
            if room_a.upper() in connects and room_b.upper() in connects:
                return door
        return None

    def route(self, start: str, end: str) -> RouteIterator:
        '''
        Compute a waypoint route between any two named entities
        (rooms, doors, or landmarks) and return a RouteIterator.

        Waypoint sequence:
          start -> door -> [intermediate room -> door ->] ... -> end

        Usage:
            iterator = fp.route("ORIGIN", "BUNNY")
            if iterator.current.reached(robot_x, robot_y):
                iterator.advance()
            if iterator.completed:
                pass  # arrived
        '''
        start_room, start_point = self._resolve_to_room_and_point(start)
        end_room, end_point = self._resolve_to_room_and_point(end)
        room_path = self._bfs_rooms(start_room, end_room)

        waypoints = []

        waypoints.append(Waypoint(
            label=start.upper(),
            position=start_point,
            kind="landmark" if start.upper() in self._landmarks else "room",
        ))

        for i in range(len(room_path) - 1):
            current, next_room = room_path[i], room_path[i + 1]
            door = self._door_between(current, next_room)
            if door:
                waypoints.append(Waypoint(
                    label=door.id,
                    position=door.segment.midpoint,
                    kind="door",
                    arrival_radius=door.width / 2,
                ))
            if i + 1 < len(room_path) - 1:
                room = self._rooms[next_room.upper()]
                waypoints.append(Waypoint(
                    label=next_room,
                    position=room.centre,
                    kind="room",
                ))

        waypoints.append(Waypoint(
            label=end.upper(),
            position=end_point,
            kind="landmark" if end.upper() in self._landmarks else "room",
        ))

        return RouteIterator(waypoints)

    # graph / geometry exports ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def adjacency(self) -> dict:
        '''
        Return a dict mapping each room id to its list of adjacent room ids,
        considering only traversable doors.
        '''
        graph = {r.id: [] for r in self._rooms.values()}
        for door in self._doors.values():
            if not door.traversable:
                continue
            a, b = door.connects[0], door.connects[1]
            if a in graph and b not in graph[a]:
                graph[a].append(b)
            if b in graph and a not in graph[b]:
                graph[b].append(a)
        return graph

    def rooms_as_rectangles(self) -> list:
        '''
        Return [(room_id, Bounds), ...] for all rooms, using bbox for
        multi-rect rooms.
        '''
        return [(r.id, r.bbox) for r in self._rooms.values()]

    def walls(self) -> list:
        '''
        Derive wall segments from room bounding rectangles.
        Returns deduplicated segments — shared walls between adjacent
        rooms appear only once.
        '''
        seen = set()
        segments = []
        for room in self._rooms.values():
            b = room.bbox
            candidates = [
                Segment(Point(b.xmin, b.ymin), Point(b.xmax, b.ymin)),  # south
                Segment(Point(b.xmax, b.ymin), Point(b.xmax, b.ymax)),  # east
                Segment(Point(b.xmax, b.ymax), Point(b.xmin, b.ymax)),  # north
                Segment(Point(b.xmin, b.ymax), Point(b.xmin, b.ymin)),  # west
            ]
            for seg in candidates:
                key = tuple(sorted([(seg.p1.x, seg.p1.y), (seg.p2.x, seg.p2.y)]))
                if key not in seen:
                    seen.add(key)
                    segments.append(seg)
        return segments

    # introspection ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def room_names(self) -> list:
        return [r.id for r in self._rooms.values()]

    def door_names(self) -> list:
        return [d.id for d in self._doors.values()]

    def landmark_names(self) -> list:
        return [lm.id for lm in self._landmarks.values()]

    def __repr__(self):
        return "Floorplan(rooms={}, doors={}, landmarks={})".format(
            self.room_names(), self.door_names(), len(self._landmarks))

#EOF
