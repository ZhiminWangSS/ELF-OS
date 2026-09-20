#!/usr/bin/env python3
"""Preview by default. Unix datagrams to an explicitly started SDK worker."""
import argparse
import json
import math
import socket
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from velocity_guard import VelocityGuard

class Bridge(Node):
    def __init__(self,socket_path=None):
        super().__init__('navigation_velocity_bridge');self.guard=VelocityGuard();self.sequence=0
        self.socket_path=socket_path;self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) if socket_path else None
        self.preview=self.create_publisher(Twist,'/navigation/guarded_cmd_vel',1)
        self.status=self.create_publisher(String,'/navigation/bridge_status',1)
        self.create_subscription(Twist,'/navigation/cmd_vel',self.command,1)
        self.create_subscription(Bool,'/navigation/localization_healthy',lambda m:self.guard.healthy(m.data,time.monotonic()),1)
        self.create_timer(.05,self.tick)
    def command(self,m):self.guard.receive([m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z],time.monotonic())
    def tick(self):
        vx,wz=self.guard.output(time.monotonic());m=Twist();m.linear.x=vx;m.angular.z=wz;self.preview.publish(m)
        self.sequence+=1;healthy=self.guard.health and not self.guard.latched and time.monotonic()-self.guard.health_time<self.guard.health_timeout
        # Timestamp is the ORIGINAL command reception, not this timer heartbeat.
        stamp=self.guard.received if math.isfinite(self.guard.received) else 0.
        packet=f'{self.sequence} {stamp:.9f} {vx:.6f} {wz:.6f} {int(healthy)}\n'
        if self.sock:
            try:self.sock.sendto(packet.encode(),self.socket_path)
            except OSError as e:self.get_logger().error(str(e));self.guard.latched=True
        self.status.publish(String(data=json.dumps({'mode':'sdk_transport' if self.sock else 'preview','vx':vx,'yaw_rate':wz,'latched':self.guard.latched,'healthy':bool(healthy)})))

def main():
    p=argparse.ArgumentParser();p.add_argument('--socket');args,ros=p.parse_known_args();rclpy.init(args=ros);node=Bridge(args.socket)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.guard.latched=True;node.tick();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
