#!/usr/bin/env python3

import math
import os
import struct
from typing import Dict, List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _parse_header(path: str) -> Tuple[Dict[str, str], int]:
    header = {}
    offset = 0
    with open(path, 'rb') as pcd:
        while True:
            line = pcd.readline()
            if not line:
                raise ValueError('PCD header ended before DATA line')
            offset += len(line)
            text = line.decode('utf-8', errors='replace').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.strip()
            if key.upper() == 'DATA':
                break
    return header, offset


def _field_offsets(header: Dict[str, str]) -> Tuple[List[str], List[int], int]:
    fields = header.get('FIELDS', '').split()
    sizes = [int(v) for v in header.get('SIZE', '').split()]
    counts = [int(v) for v in header.get('COUNT', '').split()]
    if not counts:
        counts = [1] * len(fields)
    if len(fields) != len(sizes) or len(fields) != len(counts):
        raise ValueError('PCD FIELDS/SIZE/COUNT lengths do not match')

    offsets = []
    offset = 0
    for size, count in zip(sizes, counts):
        offsets.append(offset)
        offset += size * count
    return fields, offsets, offset


class PcdToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pcd_to_occupancy_grid')

        self.declare_parameter('pcd_path', '')
        self.declare_parameter('free_pcd_path', '')
        self.declare_parameter('occupied_pcd_path', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('padding_cells', 5)
        self.declare_parameter('occupied_z_min', -0.50)
        self.declare_parameter('occupied_z_max', 0.20)
        self.declare_parameter('unknown_as_free', True)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('publish_period_sec', 5.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            qos,
        )

        self.map_msg = self._load_map()
        self.publisher.publish(self.map_msg)

        period = float(self.get_parameter('publish_period_sec').value)
        if period > 0.0:
            self.timer = self.create_timer(period, self._publish_map)
        else:
            self.timer = None

    def _load_map(self) -> OccupancyGrid:
        resolution = float(self.get_parameter('resolution').value)
        padding = int(self.get_parameter('padding_cells').value)
        z_min = float(self.get_parameter('occupied_z_min').value)
        z_max = float(self.get_parameter('occupied_z_max').value)
        unknown_as_free = bool(self.get_parameter('unknown_as_free').value)
        occupied_value = int(self.get_parameter('occupied_value').value)

        free_pcd_path = os.path.expanduser(str(self.get_parameter('free_pcd_path').value))
        occupied_pcd_path = os.path.expanduser(str(self.get_parameter('occupied_pcd_path').value))
        pcd_path = os.path.expanduser(str(self.get_parameter('pcd_path').value))

        if free_pcd_path or occupied_pcd_path:
            if not free_pcd_path or not occupied_pcd_path:
                raise ValueError('free_pcd_path and occupied_pcd_path must be set together')
            free_points = self._read_all_points(free_pcd_path)
            occupied_points = self._read_all_points(occupied_pcd_path)
            if not free_points and not occupied_points:
                raise ValueError('No points found in segmented PCD maps')
            if len(free_points) < 100:
                self.get_logger().warn(
                    f'Segmented free PCD has only {len(free_points)} points. '
                    'The static map may have almost no known free space.')
        else:
            if not pcd_path:
                raise ValueError('pcd_path parameter is empty')
            occupied_points = self._read_filtered_points(pcd_path, z_min, z_max)
            free_points = []
            if not occupied_points:
                raise ValueError(
                    f'No PCD points remain after z filter [{z_min}, {z_max}]')

        all_points = free_points + occupied_points
        min_x = min(p[0] for p in all_points)
        max_x = max(p[0] for p in all_points)
        min_y = min(p[1] for p in all_points)
        max_y = max(p[1] for p in all_points)

        origin_x = math.floor(min_x / resolution) * resolution - padding * resolution
        origin_y = math.floor(min_y / resolution) * resolution - padding * resolution
        width = int(math.ceil((max_x - origin_x) / resolution)) + padding + 1
        height = int(math.ceil((max_y - origin_y) / resolution)) + padding + 1

        default_value = 0 if unknown_as_free else -1
        grid = [default_value] * (width * height)

        occupied = 0
        free = 0
        for x, y, _ in free_points:
            mx = int((x - origin_x) / resolution)
            my = int((y - origin_y) / resolution)
            if 0 <= mx < width and 0 <= my < height:
                idx = my * width + mx
                if grid[idx] != 0:
                    grid[idx] = 0
                    free += 1

        occupied = 0
        for x, y, _ in occupied_points:
            mx = int((x - origin_x) / resolution)
            my = int((y - origin_y) / resolution)
            if 0 <= mx < width and 0 <= my < height:
                idx = my * width + mx
                if grid[idx] != occupied_value:
                    grid[idx] = occupied_value
                    occupied += 1

        msg = OccupancyGrid()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid

        self.get_logger().info(
            f'Loaded free_points={len(free_points)} occupied_points={len(occupied_points)}; '
            f'map={width}x{height} resolution={resolution:.3f} '
            f'origin=({origin_x:.2f}, {origin_y:.2f}) '
            f'free_cells={free} occupied_cells={occupied}')
        return msg

    def _read_all_points(self, path: str) -> List[Tuple[float, float, float]]:
        return self._read_points(path, None, None)

    def _read_filtered_points(
        self,
        path: str,
        z_min: float,
        z_max: float,
    ) -> List[Tuple[float, float, float]]:
        return self._read_points(path, z_min, z_max)

    def _read_points(
        self,
        path: str,
        z_min,
        z_max,
    ) -> List[Tuple[float, float, float]]:
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        header, data_offset = _parse_header(path)
        data_type = header.get('DATA', '').lower()
        fields, offsets, point_step = _field_offsets(header)
        point_count = int(header.get('POINTS', header.get('WIDTH', '0')))

        try:
            x_offset = offsets[fields.index('x')]
            y_offset = offsets[fields.index('y')]
            z_offset = offsets[fields.index('z')]
        except ValueError as exc:
            raise ValueError('PCD must contain x, y, z fields') from exc

        points = []
        if data_type == 'binary':
            with open(path, 'rb') as pcd:
                pcd.seek(data_offset)
                data = pcd.read(point_count * point_step)
            for i in range(point_count):
                base = i * point_step
                x = struct.unpack_from('<f', data, base + x_offset)[0]
                y = struct.unpack_from('<f', data, base + y_offset)[0]
                z = struct.unpack_from('<f', data, base + z_offset)[0]
                if (
                    math.isfinite(x)
                    and math.isfinite(y)
                    and math.isfinite(z)
                    and (z_min is None or z_min <= z <= z_max)
                ):
                    points.append((x, y, z))
        elif data_type == 'ascii':
            with open(path, 'r', encoding='utf-8', errors='replace') as pcd:
                while pcd.readline().strip().lower() != f'data {data_type}':
                    pass
                x_idx = fields.index('x')
                y_idx = fields.index('y')
                z_idx = fields.index('z')
                for line in pcd:
                    cols = line.split()
                    if len(cols) <= max(x_idx, y_idx, z_idx):
                        continue
                    x = float(cols[x_idx])
                    y = float(cols[y_idx])
                    z = float(cols[z_idx])
                    if (
                        math.isfinite(x)
                        and math.isfinite(y)
                        and math.isfinite(z)
                        and (z_min is None or z_min <= z <= z_max)
                    ):
                        points.append((x, y, z))
        else:
            raise ValueError(f'Unsupported PCD DATA type: {data_type}')

        return points

    def _publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcdToOccupancyGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
