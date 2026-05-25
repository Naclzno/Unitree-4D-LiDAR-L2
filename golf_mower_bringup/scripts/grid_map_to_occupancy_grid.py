#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def decode_multiarray_to_rows_cols(name, array_msg):
    data_np = np.asarray(array_msg.data, dtype=np.float32)
    dims = array_msg.layout.dim

    if len(dims) >= 2 and dims[0].label and dims[1].label:
        label0 = dims[0].label
        label1 = dims[1].label

        if label0 == 'row_index' and label1 == 'column_index':
            rows = dims[0].size or 1
            cols = dims[1].size or (len(data_np) // rows if rows else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order='C')

        if label0 == 'column_index' and label1 == 'row_index':
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            if rows * cols != data_np.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
            return data_np.reshape((rows, cols), order='F')

    if dims:
        cols = dims[0].size or 1
        rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else len(data_np))
    else:
        cols = int(math.sqrt(len(data_np))) if len(data_np) else 0
        rows = cols

    if rows * cols != data_np.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout metadata.")
    return data_np.reshape((rows, cols), order='C')


class GridMapToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('grid_map_to_occupancy_grid')
        self.declare_parameter('grid_map_topic', '/elevation_mapping_node/elevation_map_filter')
        self.declare_parameter('occupancy_grid_topic', '/elevation/traversability_grid')
        self.declare_parameter('layer', 'traversability')
        self.declare_parameter('free_threshold', 0.70)
        self.declare_parameter('occupied_threshold', 0.45)
        self.declare_parameter('unknown_value', -1)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('scale_intermediate_costs', True)
        self.declare_parameter('flip_rows', False)

        self.layer = self.get_parameter('layer').value
        self.free_threshold = float(self.get_parameter('free_threshold').value)
        self.occupied_threshold = float(self.get_parameter('occupied_threshold').value)
        self.unknown_value = int(self.get_parameter('unknown_value').value)
        self.occupied_value = int(self.get_parameter('occupied_value').value)
        self.free_value = int(self.get_parameter('free_value').value)
        self.scale_intermediate_costs = bool(self.get_parameter('scale_intermediate_costs').value)
        self.flip_rows = bool(self.get_parameter('flip_rows').value)

        grid_map_topic = self.get_parameter('grid_map_topic').value
        occupancy_grid_topic = self.get_parameter('occupancy_grid_topic').value

        self.publisher = self.create_publisher(OccupancyGrid, occupancy_grid_topic, 10)
        self.subscription = self.create_subscription(
            GridMap,
            grid_map_topic,
            self.grid_map_callback,
            10,
        )
        self.get_logger().info(
            f'Converting GridMap layer "{self.layer}" from {grid_map_topic} '
            f'to OccupancyGrid {occupancy_grid_topic}')

    def grid_map_callback(self, msg):
        if self.layer not in msg.layers:
            self.get_logger().warn(
                f'Layer "{self.layer}" not found. Available layers: {msg.layers}',
                throttle_duration_sec=2.0,
            )
            return

        layer_index = msg.layers.index(self.layer)
        try:
            layer = decode_multiarray_to_rows_cols(self.layer, msg.data[layer_index])
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        if self.flip_rows:
            layer = np.flipud(layer)

        rows, cols = layer.shape
        grid = np.full((rows, cols), self.unknown_value, dtype=np.int8)
        finite_mask = np.isfinite(layer)

        occupied_mask = finite_mask & (layer <= self.occupied_threshold)
        free_mask = finite_mask & (layer >= self.free_threshold)
        middle_mask = finite_mask & ~(occupied_mask | free_mask)

        grid[occupied_mask] = self.occupied_value
        grid[free_mask] = self.free_value

        if self.scale_intermediate_costs and np.any(middle_mask):
            denom = max(self.free_threshold - self.occupied_threshold, 1e-6)
            normalized_risk = (self.free_threshold - layer[middle_mask]) / denom
            grid[middle_mask] = np.clip(
                normalized_risk * self.occupied_value,
                self.free_value,
                self.occupied_value,
            ).astype(np.int8)
        else:
            grid[middle_mask] = self.occupied_value

        occupancy = OccupancyGrid()
        occupancy.header = msg.header
        occupancy.info.map_load_time = self.get_clock().now().to_msg()
        occupancy.info.resolution = float(msg.info.resolution)
        occupancy.info.width = int(cols)
        occupancy.info.height = int(rows)
        occupancy.info.origin.position.x = msg.info.pose.position.x - 0.5 * msg.info.length_x
        occupancy.info.origin.position.y = msg.info.pose.position.y - 0.5 * msg.info.length_y
        occupancy.info.origin.position.z = 0.0
        occupancy.info.origin.orientation.x = 0.0
        occupancy.info.origin.orientation.y = 0.0
        occupancy.info.origin.orientation.z = 0.0
        occupancy.info.origin.orientation.w = 1.0
        occupancy.data = grid.reshape(-1, order='C').tolist()
        self.publisher.publish(occupancy)


def main():
    rclpy.init()
    node = GridMapToOccupancyGrid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
