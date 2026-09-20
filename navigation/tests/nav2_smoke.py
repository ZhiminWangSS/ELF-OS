#!/usr/bin/env python3
"""Real installed Nav2 servers on saved map, simulated stationary TF, no SDK.
Run ONLY in dedicated ROS_DOMAIN_ID=44. Tests planning and action cancellation.
"""
import json
import math
import os
import time
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped,TransformStamped,Twist
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
ROOT=Path(__file__).resolve().parents[1]

def wait(node,future,timeout=30):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    if not future.done():raise RuntimeError('ROS action timed out')
    return future.result()

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='44','Never inject simulated TF into robot domain'
    rclpy.init();n=Node('offline_nav2_smoke');tf=TransformBroadcaster(n);static=StaticTransformBroadcaster(n)
    health=n.create_publisher(Bool,'/navigation/localization_healthy',1)
    obstacle=n.create_publisher(PointCloud2,'/navigation/obstacles',10)
    root=ROOT/'maps/floor_1789552084236';m=yaml.safe_load((root/'map.yaml').read_text());grid=np.flipud(np.array(Image.open(root/'map.pgm')))
    manifest=json.loads((root/'manifest.json').read_text());path=np.loadtxt(root/'trajectory_xyz.csv',delimiter=',',skiprows=1)
    origin=np.array(m['origin'][:2]);res=m['resolution'];clearance=distance_transform_edt(grid==254)*res
    def clear(p):
        x,y=np.floor((p[:2]-origin)/res).astype(int);return clearance[y,x]
    pair=None
    for i in range(5,len(path)-8):
        if clear(path[i])>.65 and clear(path[i+5])>.65 and 1.<np.linalg.norm(path[i+5,:2]-path[i,:2])<4.:
            pair=(path[i],path[i+5]);break
    if pair is None:raise RuntimeError('No verified free start/goal pair')
    start,goal=pair;yaw=math.atan2(goal[1]-start[1],goal[0]-start[0])
    def transform(parent,child,x=0.,y=0.,z=0.,yaw=0.):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=n.get_clock().now().to_msg();t.transform.translation.x=float(x);t.transform.translation.y=float(y);t.transform.translation.z=float(z);t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);return t
    static.sendTransform([transform('map','odom'),transform('base_link','lidar_imu',z=.46)])
    def tick():
        tf.sendTransform(transform('odom','base_link',start[0],start[1],yaw=yaw));health.publish(Bool(data=True))
        cloud=PointCloud2();cloud.header.frame_id='lidar_imu';cloud.header.stamp=n.get_clock().now().to_msg();cloud.height=1;cloud.width=0;cloud.point_step=12;cloud.row_step=0
        cloud.fields=[PointField(name=k,offset=j*4,datatype=7,count=1) for j,k in enumerate(['x','y','z'])];obstacle.publish(cloud)
    n.create_timer(.05,tick)
    raw=[];guarded=[]
    n.create_subscription(Twist,'/navigation/cmd_vel',lambda m:raw.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    n.create_subscription(Twist,'/navigation/guarded_cmd_vel',lambda m:guarded.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def pose(p):
        s=PoseStamped();s.header.frame_id='map';s.header.stamp=n.get_clock().now().to_msg();s.pose.position.x=float(p[0]);s.pose.position.y=float(p[1]);s.pose.orientation.z=math.sin(yaw/2);s.pose.orientation.w=math.cos(yaw/2);return s
    planner=ActionClient(n,ComputePathToPose,'compute_path_to_pose');navigator=ActionClient(n,NavigateToPose,'navigate_to_pose')
    deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.05)
        if planner.server_is_ready() and navigator.server_is_ready():break
    ready_until=time.monotonic()+4
    while time.monotonic()<ready_until:rclpy.spin_once(n,timeout_sec=.05)
    request=ComputePathToPose.Goal();request.pose=pose(goal);request.planner_id='GridBased'
    handle=wait(n,planner.send_goal_async(request));assert handle.accepted
    result=wait(n,handle.get_result_async());assert result.status==4 and len(result.result.path.poses)>2,str(result)
    minimum=100.
    for p in result.result.path.poses:
        x,y=np.floor((np.array([p.pose.position.x,p.pose.position.y])-origin)/res).astype(int)
        assert grid[y,x]==254,'Path enters occupied or unknown cell';minimum=min(minimum,clearance[y,x])
    for value,label in [(0,'occupied'),(205,'unknown')]:
        ys,xs=np.where(grid==value)
        if value==205:
            blocked=distance_transform_edt(grid==205)*res
            ys,xs=np.where(blocked>.5)
        # A goal well inside unknown space or an occupied cell must be rejected.
        x,y=int(xs[len(xs)//2]),int(ys[len(ys)//2])
        invalid=ComputePathToPose.Goal();invalid.pose=pose(origin+np.array([x+.5,y+.5])*res);invalid.planner_id='GridBased'
        bad_handle=wait(n,planner.send_goal_async(invalid));bad_result=wait(n,bad_handle.get_result_async())
        assert bad_result.status==6,f'{label} goal was not aborted'
    nav=NavigateToPose.Goal();nav.pose=pose(goal);h=wait(n,navigator.send_goal_async(nav));assert h.accepted
    deadline=time.monotonic()+4
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
    assert any(abs(v)+abs(w)>0.001 for _,v,w in raw),'Controller produced no motion proposal'
    assert all(0<=v<=.20001 and abs(w)<=.35001 for _,v,w in guarded),'Guard limit exceeded'
    cancelled=time.monotonic();cancel=wait(n,h.cancel_goal_async());assert cancel.goals_canceling
    terminal=wait(n,h.get_result_async());assert terminal.status==5,f'Expected cancelled, got {terminal.status}'
    deadline=time.monotonic()+.6
    while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
    assert guarded[-1][1:]==(0.,0.),'Guard did not stop after cancellation'
    report={'planning':'passed','occupied_and_unknown_goals':'rejected','path_points':len(result.result.path.poses),'minimum_path_clearance_m':minimum,'start':start.tolist(),'goal':goal.tolist(),'controller_proposals':len(raw),'cancellation':'passed','guarded_stop':'passed','sdk_used':False}
    (ROOT/'log/nav2_smoke.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));planner.destroy();navigator.destroy();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
