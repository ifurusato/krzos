#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2026 by Ichiro Furusato. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Ichiro Furusato
# created:  2026-03-09
# modified: 2026-03-12
#
# Interactive CLI for querying the Floorplan model.
#
# Usage:
#     python floorplan_cli.py <svg_file> <yaml_file>
#
# Example:
#     python floorplan_cli.py floorplan.svg floorplan.yaml
#

import sys
from navigate.floorplan import Floorplan, RoomResult, DoorResult, LandmarkResult, SVGParser

def format_result(result) -> str:
    if isinstance(result, RoomResult):
        bb = result.bbox
        lines = [
            "Room:    {}".format(result.id),
            "Bounds:  x {}..{}  y {}..{}".format(bb.xmin, bb.xmax, bb.ymin, bb.ymax),
            "Centre:  {}, {}".format(bb.centre.x, bb.centre.y),
            "Size:    {} x {} mm".format(bb.width, bb.height),
            "Parts:   {}".format(len(result.bounds)),
            "Doors:   {}".format(", ".join(result.doors) if result.doors else "none"),
        ]
    elif isinstance(result, DoorResult):
        p1, p2 = result.segment
        status = "open" if result.traversable else "CLOSED (treated as wall)"
        lines = [
            "Door:    {}".format(result.id),
            "From:    ({}, {})  to  ({}, {})".format(p1.x, p1.y, p2.x, p2.y),
            "Width:   {} mm".format(result.width),
            "Faces:   {}".format(result.orientation),
            "Links:   {}".format(" <-> ".join(result.connects)),
            "Status:  {}".format(status),
        ]
    elif isinstance(result, LandmarkResult):
        lines = [
            "Landmark: {}".format(result.id),
            "Position: {}, {}".format(result.position.x, result.position.y),
            "Room:     {}".format(result.room or "unknown/external"),
        ]
        if result.notes:
            lines.append("Notes:    {}".format(result.notes))
    else:
        lines = [str(result)]
    return "\n".join("  {}".format(line) for line in lines)

def format_route(iterator) -> str:
    waypoints = iterator.all_waypoints
    lines = ["  {:<3}  {:<12}  {:<10}  {:<20}  {}".format(
        "#", "label", "kind", "position", "arrival_r")]
    lines.append("  {:<3}  {:<12}  {:<10}  {:<20}  {}".format(
        "---", "------------", "----------", "--------------------", "---------"))
    for i, wp in enumerate(waypoints):
        pos = "({}, {})".format(wp.position.x, wp.position.y)
        lines.append("  {:<3}  {:<12}  {:<10}  {:<20}  {:.0f} mm".format(
            i, wp.label, wp.kind, pos, wp.arrival_radius))
    return "\n".join(lines)

def print_help():
    print("  Usage: floorplan_cli.py [--svg FILE] [--yaml FILE]")
    print("")
    print("  Commands:")
    print("    <name>            query a room, door, or landmark")
    print("    route <a> <b>     waypoint route between any two named entities")
    print("    pose              show the initial robot pose")
    print("    rooms             list all room names")
    print("    doors             list all door names")
    print("    landmarks         list all landmark names")
    print("    adjacency         show room adjacency graph")
    print("    walls             show wall count (deduplicated)")
    print("    at <x> <y>        find room containing point (x, y)")
    print("    near <x> <y>      closest landmark to point (x, y)")
    print("    save [filename]   save last route to YAML file")
    print("    export            export SVG to timestamped YAML file")
    print("    help              show this message")
    print("    quit              exit")


def main():

#   if len(sys.argv) != 3:
#       print("Usage: {} <svg_file> <yaml_file>".format(sys.argv[0]))
#       sys.exit(1)
#   svg_path, yaml_path = sys.argv[1], sys.argv[2]

    import argparse

    parser = argparse.ArgumentParser(description='Floorplan query CLI')
    parser.add_argument('--svg',  default='navigate/floorplan.svg',  help='SVG file path')
    parser.add_argument('--yaml', default='navigate/floorplan.yaml', help='YAML file path')
    args = parser.parse_args()
    svg_path, yaml_path = args.svg, args.yaml

    print("Loading {} + {} ...".format(svg_path, yaml_path), end=" ", flush=True)
    try:
        fp = Floorplan.from_files(svg_path, yaml_path)
    except Exception as e:
        print("\nError loading floorplan: {}".format(e))
        sys.exit(1)
    print("done.")
    print("  {} rooms, {} doors, {} landmarks".format(
        len(fp.room_names()), len(fp.door_names()), len(fp.landmark_names())))
    print("Type 'help' for commands, 'quit' to exit.\n")

    _last_iterator = None
    while True:
        try:
            raw = input("query> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not raw:
            continue

        cmd = raw.lower()

        if cmd in ("quit", "exit", "q"):
            break

        elif cmd == "help":
            print_help()

        elif cmd == "export":
            import datetime
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            out_path = 'floorplan_{}.yaml'.format(timestamp)
            try:
                SVGParser.export_yaml(svg_path, out_path)
                print('  exported to: {}'.format(out_path))
            except Exception as e:
                print('  export failed: {}'.format(e))

        elif cmd == "pose":
            try:
                print("  {}".format(fp.initial_pose()))
            except KeyError as e:
                print("  {}".format(e))

        elif cmd == "rooms":
            print("  " + ", ".join(fp.room_names()))

        elif cmd == "doors":
            print("  " + ", ".join(fp.door_names()))

        elif cmd == "landmarks":
            print("  " + ", ".join(fp.landmark_names()))

        elif cmd == "walls":
            print("  {} deduplicated wall segments".format(len(fp.walls())))

        elif cmd == "adjacency":
            for room, neighbours in fp.adjacency().items():
                print("  {}: {}".format(room, ", ".join(neighbours)))

        elif cmd.startswith("at "):
            parts = cmd.split()
            if len(parts) != 3:
                print("  Usage: at <x> <y>")
            else:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    room = fp.room_at(x, y)
                    if room:
                        print(format_result(room))
                    else:
                        print("  No room contains ({}, {})".format(x, y))
                except ValueError:
                    print("  Usage: at <x> <y>  (numeric coordinates)")

        elif cmd.startswith("near "):
            parts = cmd.split()
            if len(parts) != 3:
                print("  Usage: near <x> <y>")
            else:
                try:
                    x, y = float(parts[1]), float(parts[2])
                    lm = fp.closest_landmark(x, y)
                    if lm:
                        print(format_result(lm))
                    else:
                        print("  No landmarks found")
                except ValueError:
                    print("  Usage: near <x> <y>  (numeric coordinates)")

        elif cmd.startswith("route "):
            parts = raw.split()
            if len(parts) != 3:
                print("  Usage: route <start> <end>")
            else:
                try:
                    iterator = fp.route(parts[1], parts[2])
                    print(format_route(iterator))
                    _last_iterator = iterator
                except KeyError as e:
                    print("  Unknown name: {}".format(e))
                except ValueError as e:
                    print("  No path found: {}".format(e))

        elif cmd.startswith("save"):
            parts = raw.split()
            if len(parts) >= 2:
                _filename = parts[1]
            else:
                try:
                    _filename = input("  filename: ").strip()
                except (EOFError, KeyboardInterrupt):
                    print()
                    _filename = None
            if not _filename:
                print("  save cancelled.")
            elif _last_iterator is None:
                print("  no route to save; use 'route <a> <b>' first.")
            else:
                try:
                    _last_iterator.save(_filename)
                    print("  route saved to: {}".format(_filename))
                except Exception as e:
                    print("  save failed: {}".format(e))

        else:
            try:
                result = fp.query(raw)
                print(format_result(result))
            except KeyError:
                print("  Not found: {!r}".format(raw))
                print("  (try 'rooms', 'doors', or 'landmarks' to list all)")

        print()


if __name__ == "__main__":
    main()

#EOF
