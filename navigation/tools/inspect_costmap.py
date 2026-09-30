#!/usr/bin/env python3
"""Read the current Nav2 costmaps around the localized robot pose."""
import json
import math
import time

from pathlib import Path
import numpy as np
import yaml
import tf2_ros
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
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    received = {}
    counts = {'local': 0, 'global': 0}
    def receive(label, msg):
        data[label][:] = [msg]
        received[label] = time.monotonic()
        counts[label] += 1
    node.create_subscription(Odometry, '/odom', data['odom'].append, 10)
    qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: receive('local', m), qos)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: receive('global', m), qos)
    until = time.monotonic() + 5.0
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
    if not data['odom']:
        raise RuntimeError('No /odom received')
    pose = data['odom'][-1].pose.pose
    result = {'robot_xy': [pose.position.x, pose.position.y]}
    for label in ('local', 'global'):
        if data[label]:
            grid = data[label][-1]
            age = time.monotonic() - received[label]
            if counts[label] < 2 or age > 2.0:
                result[label] = {'error': 'No verified live map updates', 'updates': counts[label], 'receive_age_s': age}
                continue
            try:
                tf = buffer.lookup_transform(grid.header.frame_id, 'base_link', rclpy.time.Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                if node.get_clock().now().nanoseconds * 1e-9 - stamp > .5:
                    raise RuntimeError('Stale robot transform')
                t = tf.transform.translation; q = tf.transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                report = grid_report(grid, t.x, t.y)
                robot = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())
                cfg = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/nav2.yaml').read_text())
                pad = cfg[label+'_costmap'][label+'_costmap']['ros__parameters']['footprint_padding']
                corners = np.array(robot['footprint'], dtype=float)
                corners += np.sign(corners)*pad
                rot = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
                corners = corners@rot.T + [t.x,t.y]
                samples = np.concatenate([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/grid.info.resolution*4)+1)) for a,b in zip(corners,np.roll(corners,-1,axis=0))])
                cells = np.floor((samples-[grid.info.origin.position.x,grid.info.origin.position.y])/grid.info.resolution).astype(int)
                costs=[]; outside=False
                for x,y in set(map(tuple,cells)):
                    if 0<=x<grid.info.width and 0<=y<grid.info.height: costs.append(grid.data[y*grid.info.width+x])
                    else: outside=True
                report.update(receive_age_s=age, updates=counts[label], footprint_border_max=max(costs) if costs else None,
                    footprint_lethal_cells=sum(c==100 for c in costs), footprint_unknown_cells=sum(c<0 for c in costs), footprint_outside_map=outside)
                result[label]=report
            except Exception as exc:
                result[label]={'error': str(exc)}
        else:
            result[label] = {'error': 'No costmap received'}
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
