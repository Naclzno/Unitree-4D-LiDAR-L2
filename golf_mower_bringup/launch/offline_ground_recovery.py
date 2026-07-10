import json
import math
import os
import struct
from collections import deque


def _parse_header(path):
    header = {}
    offset = 0
    with open(path, 'rb') as pcd:
        while True:
            line = pcd.readline()
            if not line:
                raise ValueError(f'PCD header ended before DATA: {path}')
            offset += len(line)
            text = line.decode('ascii', errors='strict').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.strip()
            if key.upper() == 'DATA':
                return header, offset


def read_xyz_pcd(path):
    if not os.path.isfile(path):
        raise FileNotFoundError(path)
    header, data_offset = _parse_header(path)
    fields = header.get('FIELDS', '').split()
    sizes = [int(value) for value in header.get('SIZE', '').split()]
    counts = [int(value) for value in header.get('COUNT', '').split()] or [1] * len(fields)
    if len(fields) != len(sizes) or len(fields) != len(counts):
        raise ValueError(f'Invalid PCD FIELDS/SIZE/COUNT: {path}')

    offsets = []
    point_step = 0
    for size, count in zip(sizes, counts):
        offsets.append(point_step)
        point_step += size * count
    try:
        x_offset = offsets[fields.index('x')]
        y_offset = offsets[fields.index('y')]
        z_offset = offsets[fields.index('z')]
    except ValueError as exc:
        raise ValueError(f'PCD must contain x, y, z: {path}') from exc

    point_count = int(header.get('POINTS', header.get('WIDTH', '0')))
    points = []
    data_type = header.get('DATA', '').lower()
    if data_type == 'binary':
        with open(path, 'rb') as pcd:
            pcd.seek(data_offset)
            data = pcd.read(point_count * point_step)
        if len(data) < point_count * point_step:
            raise ValueError(f'PCD binary data is truncated: {path}')
        for index in range(point_count):
            base = index * point_step
            point = (
                struct.unpack_from('<f', data, base + x_offset)[0],
                struct.unpack_from('<f', data, base + y_offset)[0],
                struct.unpack_from('<f', data, base + z_offset)[0],
            )
            if all(math.isfinite(value) for value in point):
                points.append(point)
    elif data_type == 'ascii':
        with open(path, 'r', encoding='ascii', errors='replace') as pcd:
            for line in pcd:
                if line.strip().lower().startswith('data '):
                    break
            x_index, y_index, z_index = fields.index('x'), fields.index('y'), fields.index('z')
            for line in pcd:
                columns = line.split()
                if len(columns) <= max(x_index, y_index, z_index):
                    continue
                point = (float(columns[x_index]), float(columns[y_index]), float(columns[z_index]))
                if all(math.isfinite(value) for value in point):
                    points.append(point)
    else:
        raise ValueError(f'Unsupported PCD DATA type {data_type}: {path}')
    return points


def write_xyz_pcd(path, points):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\n'
        'DATA binary\n'
    )
    with open(path, 'wb') as pcd:
        pcd.write(header.encode('ascii'))
        for point in points:
            pcd.write(struct.pack('<fff', *point))


def _load_mapping_start(metadata_path):
    with open(metadata_path, 'r', encoding='utf-8') as metadata_file:
        metadata = json.load(metadata_file)
    pose = metadata.get('mapping_start_pointlio_pose', {})
    return float(pose['x']), float(pose['y']), float(pose['z'])


def recover_ground_map(
    map_dir,
    metadata_path,
    sensor_height=0.75,
    cell_size=0.20,
    ground_band=0.15,
    seed_radius=2.0,
    seed_z_tolerance=0.50,
    max_step_height=0.18,
    max_slope_deg=20.0,
    min_ground_points=500,
    min_ground_cells=50,
):
    source_path = os.path.join(map_dir, 'nonground_map.pcd')
    points = read_xyz_pcd(source_path)
    if not points:
        raise RuntimeError('Offline ground recovery failed: nonground_map.pcd is empty')

    start_x, start_y, start_z = _load_mapping_start(metadata_path)
    cell_points = {}
    cell_min_z = {}
    for index, (x, y, z) in enumerate(points):
        key = (math.floor(x / cell_size), math.floor(y / cell_size))
        cell_points.setdefault(key, []).append(index)
        if key not in cell_min_z or z < cell_min_z[key]:
            cell_min_z[key] = z

    nearby_cells = []
    for key, min_z in cell_min_z.items():
        center_x = (key[0] + 0.5) * cell_size
        center_y = (key[1] + 0.5) * cell_size
        distance = math.hypot(center_x - start_x, center_y - start_y)
        if distance <= seed_radius:
            nearby_cells.append((key, min_z, distance))
    ground_z_hypotheses = (start_z, start_z - sensor_height)
    expected_ground_z = max(
        ground_z_hypotheses,
        key=lambda hypothesis: sum(
            abs(min_z - hypothesis) <= seed_z_tolerance
            for _, min_z, _ in nearby_cells
        ),
    )
    seed_candidates = []
    for key, min_z, distance in nearby_cells:
        z_error = abs(min_z - expected_ground_z)
        if z_error <= seed_z_tolerance:
            seed_candidates.append((distance + z_error, key))
    if not seed_candidates:
        raise RuntimeError(
            'Offline ground recovery failed: no ground seed near the mapping start pose. '
            'Check lidar Z axis, sensor height, and map metadata.')

    seed = min(seed_candidates)[1]
    connected = {seed}
    queue = deque([seed])
    slope = math.tan(math.radians(max_slope_deg))
    while queue:
        current = queue.popleft()
        current_z = cell_min_z[current]
        for dx, dy in ((-1, -1), (-1, 0), (-1, 1), (0, -1),
                       (0, 1), (1, -1), (1, 0), (1, 1)):
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in connected or neighbor not in cell_min_z:
                continue
            horizontal = cell_size * math.hypot(dx, dy)
            allowed_delta = max(max_step_height, slope * horizontal)
            if abs(cell_min_z[neighbor] - current_z) <= allowed_delta:
                connected.add(neighbor)
                queue.append(neighbor)

    ground_indices = set()
    for key in connected:
        min_z = cell_min_z[key]
        for index in cell_points[key]:
            if points[index][2] <= min_z + ground_band:
                ground_indices.add(index)

    ground = [point for index, point in enumerate(points) if index in ground_indices]
    nonground = [point for index, point in enumerate(points) if index not in ground_indices]
    if len(ground) < min_ground_points or len(connected) < min_ground_cells:
        raise RuntimeError(
            'Offline ground recovery failed quality checks: '
            f'ground_points={len(ground)} required={min_ground_points}, '
            f'ground_cells={len(connected)} required={min_ground_cells}')

    ground_path = os.path.join(map_dir, 'recovered_ground_map.pcd')
    nonground_path = os.path.join(map_dir, 'recovered_nonground_map.pcd')
    report_path = os.path.join(map_dir, 'recovery_report.yaml')
    write_xyz_pcd(ground_path, ground)
    write_xyz_pcd(nonground_path, nonground)
    report = {
        'source': source_path,
        'mapping_start': {'x': start_x, 'y': start_y, 'z': start_z},
        'selected_ground_z_hypothesis': expected_ground_z,
        'parameters': {
            'sensor_height': sensor_height,
            'cell_size': cell_size,
            'ground_band': ground_band,
            'max_step_height': max_step_height,
            'max_slope_deg': max_slope_deg,
        },
        'result': {
            'input_points': len(points),
            'ground_points': len(ground),
            'nonground_points': len(nonground),
            'connected_ground_cells': len(connected),
        },
    }
    with open(report_path, 'w', encoding='utf-8') as report_file:
        json.dump(report, report_file, indent=2)
        report_file.write('\n')
    return ground_path, nonground_path, report
