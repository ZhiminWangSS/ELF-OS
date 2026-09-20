#!/usr/bin/env python3
"""Stationary synthetic LiDAR+IMU replay through actual LIO and frame adapter."""
import json,os,sys,time,subprocess,signal
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from livox_ros_driver2.msg import CustomMsg,CustomPoint
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'));from pcd import read_pcd

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='46';rclpy.init();n=Node('stationary_lio_replay')
    folder=ROOT/'maps/floor_1789552084236';manifest=json.loads((folder/'manifest.json').read_text());session=Path(manifest['source_session']);index=20
    rows=np.genfromtxt(session/'trajectory.csv',delimiter=',',names=True);row=rows[index]
    saved=np.eye(4);saved[:3,:3]=Rotation.from_quat([row[k] for k in ['qx','qy','qz','qw']]).as_dcm();saved[:3,3]=[row[k] for k in ['x','y','z']]
    expected=np.array(manifest['map_from_saved'])@saved
    durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    init=n.create_publisher(PoseWithCovarianceStamped,'/icp_result',durable);lidar=n.create_publisher(CustomMsg,'/livox/lidar',10);imu=n.create_publisher(Imu,'/livox/imu',100)
    health=[];poses=[]
    n.create_subscription(Bool,'/navigation/localization_healthy',lambda m:health.append((time.monotonic(),m.data)),10)
    n.create_subscription(Odometry,'/odom',lambda m:poses.append(m),10)
    p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.pose.position.x,p.pose.pose.position.y,p.pose.pose.position.z=map(float,expected[:3,3]);q=Rotation.from_dcm(expected[:3,:3]).as_quat();p.pose.pose.orientation.x,p.pose.pose.orientation.y,p.pose.pose.orientation.z,p.pose.pose.orientation.w=map(float,q)
    init.publish(p)
    points=read_pcd(session/'keyframes'/f'{index}.pcd')[:,:3];ext=Rotation.from_euler('y',-np.pi/6).as_dcm();points=(points-np.array([-.011,-.02329,.04412]))@ext
    scan=CustomMsg();scan.header.frame_id='livox_frame';scan.point_num=len(points)
    for i,xyz in enumerate(points):
        point=CustomPoint();point.x,point.y,point.z=map(float,xyz);point.reflectivity=30;point.tag=16;point.line=i%4;point.offset_time=int(i/max(1,len(points)-1)*90000000);scan.points.append(point)
    acceleration=expected[:3,:3].T@np.array([0.,0.,9.81]);im=Imu();im.header.frame_id='livox_frame';im.linear_acceleration.x,im.linear_acceleration.y,im.linear_acceleration.z=map(float,acceleration)
    logs=[];processes=[]
    commands=[['ros2','run','fast_lio','fastlio_mapping','--ros-args','--params-file',str(ROOT/'config/localization.yaml'),'-r','/Odometry:=/lio/odom'],['python3',str(ROOT/'tools/localization_adapter.py'),'--robot',str(ROOT/'config/robot.yaml')]]
    try:
        for i,command in enumerate(commands):
            log=(ROOT/f'log/localization_replay_{i}.log').open('w');logs.append(log);processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        start=time.monotonic();next_scan=start+1;next_imu=start
        while time.monotonic()-start<20:
            assert all(p.poll() is None for p in processes),'localization child crashed'
            now=time.monotonic()
            if now>=next_imu:im.header.stamp=n.get_clock().now().to_msg();imu.publish(im);next_imu=now+.005
            if now>=next_scan:
                scan.header.stamp=n.get_clock().now().to_msg();lidar.publish(scan);next_scan=now+.1
            rclpy.spin_once(n,timeout_sec=.001)
        assert len(poses)>30,f'Only {len(poses)} odometry samples'
        assert sum(v for _,v in health[-50:])>=40,'Localization never became consistently healthy'
        # State is the IMU pose; compare base XY using configured mount offset.
        robot=yaml.safe_load((ROOT/'config/robot.yaml').read_text());mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('xyz',robot['base_from_imu_rpy']).as_dcm();mount[:3,3]=robot['base_from_imu_xyz'];base=expected@np.linalg.inv(mount)
        xy=np.array([[m.pose.pose.position.x,m.pose.pose.position.y] for m in poses[-50:]])
        error=float(np.max(np.linalg.norm(xy-base[:2,3],axis=1)));assert error<.15,error
        stopped=time.monotonic()
        while time.monotonic()-stopped<1:rclpy.spin_once(n,timeout_sec=.02)
        assert health[-1][1] is False,'Health did not fail after sensor loss'
        report={'odometry_samples':len(poses),'max_stationary_xy_error_m':error,'healthy_tracking':True,'sensor_loss_rejected':True,'synthetic_imu':True,'hardware_motion':False}
        (ROOT/'log/localization_smoke.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        for process in processes:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=7)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for log in logs:log.close()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
