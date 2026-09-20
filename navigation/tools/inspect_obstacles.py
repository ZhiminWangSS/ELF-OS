#!/usr/bin/env python3
"""Summarize live obstacle points around the localized base."""
import json
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('navigation_obstacle_inspector')
    latest = {}
    node.create_subscription(Odometry, '/odom', lambda msg: latest.update(odom=msg), 10)
    node.create_subscription(PointCloud2, '/navigation/obstacles', lambda msg: latest.update(cloud=msg), rclpy.qos.qos_profile_sensor_data)
    end = time.monotonic() + 3
    while time.monotonic() < end and ('odom' not in latest or 'cloud' not in latest):
        rclpy.spin_once(node, timeout_sec=.1)
    if 'odom' not in latest or 'cloud' not in latest:
        raise RuntimeError('Missing odometry or obstacle cloud')
    cloud = latest['cloud']
    raw = np.frombuffer(bytes(cloud.data), dtype='<f4').reshape(-1, 3)
    pose = latest['odom'].pose.pose
    yaw = 2 * np.arctan2(pose.orientation.z, pose.orientation.w)
    delta = raw[:, :2] - np.array([pose.position.x, pose.position.y])
    local = delta @ np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    distance = np.linalg.norm(local, axis=1)
    near = (distance <= 1.0)
    sectors = {}
    for label, mask in {
        'front': near & (local[:, 0] > .15), 'rear': near & (local[:, 0] < -.15),
        'left': near & (local[:, 1] > .15), 'right': near & (local[:, 1] < -.15),
    }.items():
        sectors[label] = {'points': int(mask.sum()), 'nearest_m': round(float(distance[mask].min()), 3) if mask.any() else None}
    print(json.dumps({'points_total': int(len(raw)), 'points_within_1m': int(near.sum()), 'nearest_m': round(float(distance.min()), 3), 'sectors': sectors, 'height_range_m': [round(float(raw[:,2].min()),3), round(float(raw[:,2].max()),3)]}, indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
