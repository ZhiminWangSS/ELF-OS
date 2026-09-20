#!/usr/bin/env python3
"""Read the current Nav2 costmaps around the localized robot pose."""
import json
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry


def grid_report(grid, x, y):
    info = grid.info
    col = int(math.floor((x - info.origin.position.x) / info.resolution))
    row = int(math.floor((y - info.origin.position.y) / info.resolution))
    radius = max(1, int(math.ceil(0.5 / info.resolution)))
    values = []
    for r in range(max(0, row - radius), min(info.height, row + radius + 1)):
        for c in range(max(0, col - radius), min(info.width, col + radius + 1)):
            if (r - row) ** 2 + (c - col) ** 2 <= radius ** 2:
                values.append(grid.data[r * info.width + c])
    center = grid.data[row * info.width + col] if 0 <= row < info.height and 0 <= col < info.width else None
    return {
        'frame': grid.header.frame_id,
        'resolution_m': info.resolution,
        'center_cost': center,
        'max_cost_within_0_5m': max(values) if values else None,
        'lethal_cells_within_0_5m': sum(v >= 100 for v in values),
        'unknown_cells_within_0_5m': sum(v < 0 for v in values),
    }


def main():
    rclpy.init()
    node = rclpy.create_node('navigation_costmap_inspector')
    data = {'odom': [], 'local': [], 'global': []}
    node.create_subscription(Odometry, '/odom', data['odom'].append, 10)
    qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', data['local'].append, qos)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', data['global'].append, qos)
    until = time.monotonic() + 3.0
    while time.monotonic() < until and (not data['odom'] or not data['local'] or not data['global']):
        rclpy.spin_once(node, timeout_sec=.1)
    if not data['odom']:
        raise RuntimeError('No /odom received')
    pose = data['odom'][-1].pose.pose
    result = {'robot_xy': [pose.position.x, pose.position.y]}
    for label in ('local', 'global'):
        if data[label]:
            result[label] = grid_report(data[label][-1], pose.position.x, pose.position.y)
        else:
            result[label] = {'error': 'No costmap received'}
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
