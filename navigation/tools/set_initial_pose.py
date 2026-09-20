#!/usr/bin/env python3
"""Set a map-frame BASE pose guess; mounting transform supplies the IMU pose."""
import argparse,time
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_adapter import set_pose
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--x',type=float,required=True);p.add_argument('--y',type=float,required=True);p.add_argument('--yaw-deg',type=float,required=True);args=p.parse_args()
if not np.isfinite([args.x,args.y,args.yaw_deg]).all():raise SystemExit('Pose must be finite')
robot=yaml.safe_load((ROOT/'config/robot.yaml').read_text());base=np.eye(4);base[:3,:3]=Rotation.from_euler('z',args.yaw_deg,degrees=True).as_dcm();base[:2,3]=[args.x,args.y]
mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('xyz',robot['base_from_imu_rpy']).as_dcm();mount[:3,3]=robot['base_from_imu_xyz']
rclpy.init();n=rclpy.create_node('set_navigation_initial_pose');pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
end=time.monotonic()+3
while pub.get_subscription_count()==0 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
if not pub.get_subscription_count():raise SystemExit('Start localization preview first')
msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=n.get_clock().now().to_msg();set_pose(msg.pose.pose,base@mount);pub.publish(msg)
for _ in range(5):rclpy.spin_once(n,timeout_sec=.1)
print('Initial pose guess sent; wait for matching and healthy localization. No motion requested.')
n.destroy_node();rclpy.shutdown()
