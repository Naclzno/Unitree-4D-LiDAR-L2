#!/usr/bin/env python3

import argparse
from collections import deque
import math
import os
import struct
from typing import Dict, Iterable, List, Sequence, Set, Tuple


Point2 = Tuple[float, float]
Cell = Tuple[int, int]


def parse_pcd_header(path: str) -> Tuple[Dict[str, str], int]:
    header: Dict[str, str] = {}
    offset = 0
    with open(path, 'rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header ended before DATA line')
            offset += len(line)
            text = line.decode('utf-8', errors='replace').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.strip()
            if key.upper() == 'DATA':
                return header, offset


def field_layout(header: Dict[str, str]) -> Tuple[List[str], List[int], int]:
    fields = header.get('FIELDS', '').split()
    sizes = [int(value) for value in header.get('SIZE', '').split()]
    counts = [int(value) for value in header.get('COUNT', '').split()]
    if not counts:
        counts = [1] * len(fields)
    if len(fields) != len(sizes) or len(fields) != len(counts):
        raise ValueError('PCD FIELDS/SIZE/COUNT lengths do not match')
    offsets: List[int] = []
    point_step = 0
    for size, count in zip(sizes, counts):
        offsets.append(point_step)
        point_step += size * count
    return fields, offsets, point_step


def scalar_format(type_name: str, size: int) -> str:
    formats = {
        ('F', 4): 'f', ('F', 8): 'd',
        ('I', 1): 'b', ('I', 2): 'h', ('I', 4): 'i', ('I', 8): 'q',
        ('U', 1): 'B', ('U', 2): 'H', ('U', 4): 'I', ('U', 8): 'Q',
    }
    try:
        return formats[(type_name.upper(), size)]
    except KeyError as exc:
        raise ValueError(f'Unsupported PCD scalar type {type_name}{size}') from exc


def read_xy(path: str) -> List[Point2]:
    header, data_offset = parse_pcd_header(path)
    fields, offsets, point_step = field_layout(header)
    try:
        x_index = fields.index('x')
        y_index = fields.index('y')
    except ValueError as exc:
        raise ValueError('PCD must contain x and y fields') from exc

    sizes = [int(value) for value in header['SIZE'].split()]
    types = header.get('TYPE', 'F ' * len(fields)).split()
    if len(types) != len(fields):
        raise ValueError('PCD FIELDS/TYPE lengths do not match')
    x_unpack = struct.Struct('<' + scalar_format(types[x_index], sizes[x_index]))
    y_unpack = struct.Struct('<' + scalar_format(types[y_index], sizes[y_index]))
    point_count = int(header.get('POINTS', header.get('WIDTH', '0')))
    data_type = header['DATA'].lower()
    points: List[Point2] = []

    if data_type == 'binary':
        with open(path, 'rb') as stream:
            stream.seek(data_offset)
            data = stream.read(point_count * point_step)
        if len(data) < point_count * point_step:
            raise ValueError('Binary PCD payload is shorter than its header declares')
        for index in range(point_count):
            base = index * point_step
            x = float(x_unpack.unpack_from(data, base + offsets[x_index])[0])
            y = float(y_unpack.unpack_from(data, base + offsets[y_index])[0])
            if math.isfinite(x) and math.isfinite(y):
                points.append((x, y))
    elif data_type == 'ascii':
        with open(path, 'r', encoding='utf-8', errors='replace') as stream:
            while stream.readline().strip().lower() != 'data ascii':
                pass
            for line in stream:
                values = line.split()
                if len(values) < len(fields):
                    continue
                x = float(values[x_index])
                y = float(values[y_index])
                if math.isfinite(x) and math.isfinite(y):
                    points.append((x, y))
    else:
        raise ValueError(f'Unsupported PCD DATA type: {data_type}')
    return points


def rasterize(points: Iterable[Point2], resolution: float) -> Set[Cell]:
    return {
        (math.floor(x / resolution), math.floor(y / resolution))
        for x, y in points
    }


def dilate(cells: Set[Cell], radius: int) -> Set[Cell]:
    if radius <= 0:
        return set(cells)
    offsets = [
        (dx, dy)
        for dx in range(-radius, radius + 1)
        for dy in range(-radius, radius + 1)
        if dx * dx + dy * dy <= radius * radius
    ]
    return {(x + dx, y + dy) for x, y in cells for dx, dy in offsets}


def largest_component(cells: Set[Cell]) -> Set[Cell]:
    remaining = set(cells)
    largest: Set[Cell] = set()
    neighbors = [
        (-1, -1), (-1, 0), (-1, 1), (0, -1),
        (0, 1), (1, -1), (1, 0), (1, 1),
    ]
    while remaining:
        start = remaining.pop()
        component = {start}
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in neighbors:
                neighbor = (x + dx, y + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        if len(component) > len(largest):
            largest = component
    return largest


def cross(origin: Point2, a: Point2, b: Point2) -> float:
    return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])


def convex_hull(points: Sequence[Point2]) -> List[Point2]:
    unique = sorted(set(points))
    if len(unique) < 3:
        raise ValueError('At least three non-collinear points are required')
    lower: List[Point2] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: List[Point2] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    if len(hull) < 3:
        raise ValueError('Ground cells are collinear')
    return hull


def cell_corners(cells: Iterable[Cell], resolution: float) -> List[Point2]:
    corners: List[Point2] = []
    for x, y in cells:
        x0 = x * resolution
        y0 = y * resolution
        corners.extend([
            (x0, y0), (x0 + resolution, y0),
            (x0 + resolution, y0 + resolution), (x0, y0 + resolution),
        ])
    return corners


def polygon_area(points: Sequence[Point2]) -> float:
    return 0.5 * abs(sum(
        x1 * y2 - x2 * y1
        for (x1, y1), (x2, y2) in zip(points, points[1:] + points[:1])
    ))


def write_yaml(path: str, boundary: Sequence[Point2], args: argparse.Namespace) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write('frame_id: map\n\n')
        stream.write('# Candidate convex boundary generated from ground_map.pcd.\n')
        stream.write('# Verify and edit this polygon in RViz before commanding robot motion.\n')
        stream.write('boundary:\n')
        for x, y in boundary:
            stream.write(f'  - [{x:.3f}, {y:.3f}]\n')
        stream.write('\nexclusions: []\n\n')
        stream.write('robot:\n')
        stream.write(f'  width: {args.robot_width:.3f}\n')
        stream.write(f'  coverage_width: {args.coverage_width:.3f}\n')
        stream.write(f'  min_turning_radius: {args.min_turning_radius:.3f}\n')
        stream.write(f'  cruise_speed: {args.cruise_speed:.3f}\n')
        stream.write(f'  turn_speed: {args.turn_speed:.3f}\n\n')
        stream.write('planner:\n')
        stream.write(f'  headland_swaths: {args.headland_swaths}\n')
        stream.write('  swath_angle_deg: null\n')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Generate a candidate Fields2Cover boundary from a Stage5 ground PCD.')
    parser.add_argument('--ground-pcd', required=True, help='Input ground_map.pcd path')
    parser.add_argument('--output', required=True, help='Output coverage area YAML path')
    parser.add_argument('--resolution', type=float, default=0.25, help='Ground raster resolution in meters')
    parser.add_argument('--connect-gap-cells', type=int, default=2, help='Raster dilation radius used to bridge scan gaps')
    parser.add_argument('--robot-width', type=float, default=1.50)
    parser.add_argument('--coverage-width', type=float, default=1.00)
    parser.add_argument('--min-turning-radius', type=float, default=1.0)
    parser.add_argument('--cruise-speed', type=float, default=0.40)
    parser.add_argument('--turn-speed', type=float, default=0.20)
    parser.add_argument('--headland-swaths', type=int, default=3)
    args = parser.parse_args()

    if args.resolution <= 0.0 or args.connect_gap_cells < 0:
        parser.error('resolution must be positive and connect-gap-cells must be nonnegative')
    input_path = os.path.abspath(os.path.expanduser(args.ground_pcd))
    output_path = os.path.abspath(os.path.expanduser(args.output))
    points = read_xy(input_path)
    if len(points) < 3:
        raise ValueError(f'{input_path} contains fewer than three valid ground points')
    observed_cells = rasterize(points, args.resolution)
    connected_cells = dilate(observed_cells, args.connect_gap_cells)
    component = largest_component(connected_cells)
    if len(component) < 3:
        raise ValueError('No usable connected ground component was found')
    boundary = convex_hull(cell_corners(component, args.resolution))
    area = polygon_area(boundary)
    if area <= 0.0:
        raise ValueError('Generated candidate boundary has zero area')
    write_yaml(output_path, boundary, args)
    print(f'input_points: {len(points)}')
    print(f'observed_cells: {len(observed_cells)}')
    print(f'largest_connected_cells: {len(component)}')
    print(f'boundary_vertices: {len(boundary)}')
    print(f'candidate_area_m2: {area:.3f}')
    print(f'output: {output_path}')
    print('warning: convex candidate only; verify boundary and add exclusions before execution')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
