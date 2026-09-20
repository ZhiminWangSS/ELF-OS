#!/usr/bin/env python3
"""Measure raw delivery and validate samples without slowing point reception."""
import argparse
import math
import time

import rclpy
from rclpy.serialization import deserialize_message
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 3:
        parser.error('--seconds must be finite and at least 3')
    rclpy.init()
    node = rclpy.create_node('mid360_data_check')
    stats = {kind: {'count': 0, 'first': None, 'last': None, 'gap': 0.0, 'samples': {}}
             for kind in ('lidar', 'imu')}
    started = time.monotonic()

    def record(kind, payload):
        now = time.monotonic()
        stat = stats[kind]
        if stat['first'] is None:
            stat['first'] = now
        if stat['last'] is not None:
            stat['gap'] = max(stat['gap'], now - stat['last'])
        stat['last'] = now
        stat['count'] += 1
        # Bound memory to three payloads/topic; decode after timing completes.
        bucket = min(2, int((now - started) / (args.seconds / 3)))
        stat['samples'][bucket] = payload

    for kind, cls in (('lidar', CustomMsg), ('imu', Imu)):
        node.create_subscription(cls, '/livox/' + kind,
                                 lambda payload, kind=kind: record(kind, payload),
                                 20, raw=True)
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        finished = time.monotonic()
        failures = []
        point_counts = []
        for kind, cls, minimum_rate in (('lidar', CustomMsg, 8), ('imu', Imu, 160)):
            stat = stats[kind]
            duration = (stat['last'] - stat['first']) if stat['count'] > 1 else 0
            rate = (stat['count'] - 1) / duration if duration > 0 else 0
            stamps = []
            for payload in stat['samples'].values():
                msg = deserialize_message(payload, cls)
                stamps.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
                if kind == 'lidar':
                    point_counts.append(msg.point_num)
                    if not msg.points or msg.point_num != len(msg.points):
                        failures.append('点云为空或点数不一致')
                    if not all(math.isfinite(v) for p in msg.points for v in (p.x, p.y, p.z)):
                        failures.append('点云含非有限坐标')
                else:
                    values = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                              msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
                    if not all(math.isfinite(value) for value in values):
                        failures.append('IMU 含非有限数值')
            print(f"/livox/{kind}: {stat['count']} 条，{rate:.2f} Hz，最大接收间隔 {stat['gap']:.3f}s，"
                  f"内容抽检 {len(stamps)} 帧")
            if duration < args.seconds * 0.7 or rate < minimum_rate:
                failures.append(f'{kind} 数据持续时间或频率不足')
            if stat['last'] is None or finished - stat['last'] > 1 or stat['gap'] > 1:
                failures.append(f'{kind} 接收中断超过 1 秒')
            if len(stamps) < 2 or any(b <= a for a, b in zip(stamps, stamps[1:])):
                failures.append(f'{kind} 抽检时间戳未持续递增')
        if point_counts:
            print(f'抽检每帧点数：{min(point_counts)}–{max(point_counts)}')
        print('FAIL：' + '；'.join(sorted(set(failures))) if failures else
              'PASS：点云与 IMU 持续到达，频率和抽检内容正常。')
        return 1 if failures else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
