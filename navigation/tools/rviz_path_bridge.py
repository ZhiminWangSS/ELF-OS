#!/usr/bin/env python3
"""Cache visualization paths for late RViz subscribers; never send motion commands."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Path, OccupancyGrid


class PathDisplayBridge(Node):
    def __init__(self):
        super().__init__('rviz_path_bridge')
        self.ttl = float(self.declare_parameter('preview_timeout_s', 60.0).value)
        self.cached = {}
        self.active = set()
        self.map_stamp = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.outputs = {name: self.create_publisher(Path, '/navigation/display/' + name, qos)
                        for name in ('global_plan', 'local_plan')}
        self.create_subscription(Path, '/plan', lambda m: self.receive('global_plan', m), 10)
        self.create_subscription(Path, '/local_plan', lambda m: self.receive('local_plan', m), 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_changed, qos)
        self.create_timer(.25, self.expire)

    def receive(self, name, msg):
        self.outputs[name].publish(msg)
        self.cached[name] = (time.monotonic(), msg.header.frame_id)

    def clear(self):
        for name in list(self.cached):
            self.clear_one(name)

    def clear_one(self, name):
        _, frame = self.cached.pop(name)
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        self.outputs[name].publish(msg)

    def status(self, msg):
        # Accepted/executing/canceling goals are active. Clear when they finish.
        active = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list if s.status in (1, 2, 3)}
        if self.active - active:
            self.clear()
        self.active = active

    def map_changed(self, msg):
        stamp = (msg.info.map_load_time.sec, msg.info.map_load_time.nanosec)
        if self.map_stamp is not None and stamp != self.map_stamp:
            self.clear()
        self.map_stamp = stamp

    def expire(self):
        now = time.monotonic()
        for name, (received, _) in list(self.cached.items()):
            # Local trajectories become obsolete quickly; previews have a longer lifetime.
            limit = 2.0 if name == 'local_plan' else self.ttl
            if now - received > limit:
                self.clear_one(name)


def main():
    rclpy.init()
    node = PathDisplayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
