#!/usr/bin/env python3
"""Replay saved body scan through real ICP node, isolated domain 45, no SDK."""
import json,os,sys,time,subprocess,signal
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from livox_ros_driver2.msg import CustomMsg,CustomPoint
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'tools'));from pcd import read_pcd

def trial(n,index,bad=False):
    folder=ROOT/'maps/floor_1789552084236';manifest=json.loads((folder/'manifest.json').read_text());session=Path(manifest['source_session'])
    rows=np.genfromtxt(session/'trajectory.csv',delimiter=',',names=True);row=rows[index]
    saved=np.eye(4);saved[:3,:3]=Rotation.from_quat([row[k] for k in ['qx','qy','qz','qw']]).as_dcm();saved[:3,3]=[row[k] for k in ['x','y','z']]
    expected=np.array(manifest['map_from_saved'])@saved
    xyz=expected[:3,3]+([100,100,0] if bad else [.15,-.15,.05]);rpy=Rotation.from_dcm(expected[:3,:3]).as_euler('xyz');rpy[2]+=.08
    config={'prior_initializer':{'ros__parameters':{'map_path':str(folder/'map.pcd'),'initial_xyz':xyz.tolist(),'initial_rpy':rpy.tolist()}}}
    label='reject' if bad else 'accept';params=ROOT/f'log/icp_{label}_params.yaml';params.write_text(yaml.safe_dump(config))
    result=[];qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub=n.create_subscription(PoseWithCovarianceStamped,'/icp_result',lambda m:result.append(m),qos)
    pub=n.create_publisher(CustomMsg,'/livox/lidar',10)
    points=read_pcd(session/'keyframes'/f'{index}.pcd')[:,:3]
    # Undo LiDAR->IMU transform because input node expects driver's rotated points.
    ext=Rotation.from_euler('y',-np.pi/6).as_dcm();points=(points-np.array([-.011,-.02329,.04412]))@ext
    msg=CustomMsg();msg.header.frame_id='livox_frame';msg.point_num=len(points)
    for xyz in points:
        p=CustomPoint();p.x,p.y,p.z=map(float,xyz);p.reflectivity=30;p.tag=16;p.line=0;msg.points.append(p)
    log=(ROOT/f'log/icp_{label}.log').open('w');process=subprocess.Popen([str(ROOT/'ws/install/elf_navigation/lib/elf_navigation/prior_initializer'),'--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT)
    try:
        end=time.monotonic()+(10 if bad else 25);last=0
        while time.monotonic()<end and not result:
            if process.poll() is not None:raise RuntimeError('ICP node exited unexpectedly')
            if time.monotonic()-last>.1:
                msg.header.stamp=n.get_clock().now().to_msg();pub.publish(msg);last=time.monotonic()
            rclpy.spin_once(n,timeout_sec=.03)
        if bad:
            assert not result,'ICP accepted an impossible initial pose';return {'bad_initial_pose_rejected':True}
        assert result,'ICP failed to initialize on saved scan'
        p=result[0].pose.pose;actual=np.array([p.position.x,p.position.y,p.position.z]);q=p.orientation
        R=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_dcm();distance=float(np.linalg.norm(actual-expected[:3,3]));angle=float(np.linalg.norm(Rotation.from_dcm(R@expected[:3,:3].T).as_rotvec()))
        assert distance<.15 and angle<.1,(distance,angle)
        return {'translation_error_m':distance,'rotation_error_rad':angle,'saved_keyframe':index}
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=5)
        log.close();n.destroy_subscription(sub);n.destroy_publisher(pub)

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='45';rclpy.init();n=Node('initializer_replay_test')
    # Reject first to avoid transient-local accepted pose from an earlier publisher.
    report=trial(n,20,True);report.update(trial(n,20,False));print(json.dumps(report));(ROOT/'log/initializer_smoke.json').write_text(json.dumps(report,indent=2)+'\n');n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
