#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
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
            return data_np.reshape((rows, cols), order='C')
        if label0 == 'column_index' and label1 == 'row_index':
            cols = dims[0].size or 1
            rows = dims[1].size or (len(data_np) // cols if cols else 0)
            return data_np.reshape((rows, cols), order='F')

    cols = dims[0].size if dims else int(math.sqrt(len(data_np)))
    rows = dims[1].size if len(dims) > 1 else (len(data_np) // cols if cols else 0)
    return data_np.reshape((rows, cols), order='C')


class GridMapInspect(Node):
    def __init__(self):
        super().__init__('grid_map_inspect')
        self.declare_parameter('topic', '/elevation_mapping_node/elevation_map_raw')
        topic = self.get_parameter('topic').value
        self.subscription = self.create_subscription(GridMap, topic, self.callback, 10)
        self.seen = False
        self.get_logger().info(f'Waiting for GridMap on {topic}')

    def callback(self, msg):
        if self.seen:
            return
        self.seen = True
        self.get_logger().info(
            f'GridMap frame={msg.header.frame_id}, '
            f'size=({msg.info.length_x:.2f}, {msg.info.length_y:.2f}), '
            f'resolution={msg.info.resolution:.3f}, layers={list(msg.layers)}')
        for layer_name, array_msg in zip(msg.layers, msg.data):
            layer = decode_multiarray_to_rows_cols(layer_name, array_msg)
            finite = np.isfinite(layer)
            finite_count = int(np.count_nonzero(finite))
            total = int(layer.size)
            if finite_count:
                values = layer[finite]
                summary = (
                    f'min={float(np.min(values)):.3f}, '
                    f'max={float(np.max(values)):.3f}, '
                    f'mean={float(np.mean(values)):.3f}')
            else:
                summary = 'all values are NaN/invalid'
            self.get_logger().info(
                f'layer={layer_name}, shape={layer.shape}, '
                f'finite={finite_count}/{total}, {summary}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GridMapInspect()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
