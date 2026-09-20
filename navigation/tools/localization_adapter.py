#!/usr/bin/env python3
"""Rebase prior-map LIO into level Nav2 frames; publish observed obstacles.

map==odom is intentional: this localizer continuously matches a fixed prior map.
A lost/reinitialized localizer requires cancellation and a full navigation restart.
"""
import argparse
import math
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import Float64MultiArray,Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster

def matrix(p):
    q=p.orientation;t=p.position
    T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_dcm();T[:3,3]=[t.x,t.y,t.z];return T

def set_pose(p,T):
    p.position.x,p.position.y,p.position.z=map(float,T[:3,3])
    q=Rotation.from_dcm(T[:3,:3]).as_quat()
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,q)

def tf_msg(parent,child,T,stamp):
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.header.stamp=stamp
    t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,T[:3,3])
    q=Rotation.from_dcm(T[:3,:3]).as_quat()
    t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q);return t

class Adapter(Node):
    def __init__(self,config):
        super().__init__('localization_adapter');self.config=config
        self.base_from_imu=np.eye(4);self.base_from_imu[:3,:3]=Rotation.from_euler('xyz',config['base_from_imu_rpy']).as_dcm()
        self.base_from_imu[:3,3]=config['base_from_imu_xyz'];self.initial=None;self.current=None;self.prev=None
        self.quality=None;self.quality_received=0.;self.odom_received=0.;self.cloud_received=0.;self.valid_pose=False
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        self.static.sendTransform(tf_msg('map','odom',np.eye(4),self.get_clock().now().to_msg()))
        self.odom_pub=self.create_publisher(Odometry,'/odom',10)
        self.cloud_pub=self.create_publisher(PointCloud2,'/navigation/obstacles',qos_profile_sensor_data)
        self.health_pub=self.create_publisher(Bool,'/navigation/localization_healthy',1)
        durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.guess_pub=self.create_publisher(PoseWithCovarianceStamped,'/navigation/initial_imu_pose',10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.guess_cb,10)
        self.create_subscription(PoseWithCovarianceStamped,'/icp_result',self.initial_cb,durable)
        self.create_subscription(Odometry,'/lio/odom',self.odom_cb,10)
        self.create_subscription(Float64MultiArray,'/localization/quality',self.quality_cb,10)
        self.create_subscription(PointCloud2,'/cloud_registered_body',self.cloud_cb,qos_profile_sensor_data)
        self.create_timer(.05,self.health)
    def guess_cb(self,msg):
        if self.initial is not None:
            self.get_logger().warning('Already initialized: stop navigation and restart preview before reinitializing')
            return
        if msg.header.frame_id!='map':return
        try:
            base=matrix(msg.pose.pose)
            if not np.isfinite(base).all():return
            base[2,3]=0.
            pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
            set_pose(pose.pose.pose,base@self.base_from_imu);self.guess_pub.publish(pose)
        except ValueError:self.get_logger().warning('Rejected invalid initial pose')

    def initial_cb(self,msg):
        if self.initial is None and msg.header.frame_id=='map':self.initial=matrix(msg.pose.pose)
    def quality_cb(self,msg):
        self.quality=list(msg.data);self.quality_received=time.monotonic()
    def stamp_seconds(self,stamp):return stamp.sec+stamp.nanosec*1e-9
    def fresh(self,stamp):
        age=self.get_clock().now().nanoseconds*1e-9-self.stamp_seconds(stamp)
        return -.1<=age<=self.config['sensor_timeout_s']
    def odom_cb(self,msg):
        if self.initial is None or not self.fresh(msg.header.stamp):return
        imu=self.initial@matrix(msg.pose.pose);base=imu@np.linalg.inv(self.base_from_imu)
        if not np.isfinite(base).all():self.valid_pose=False;return
        yaw=math.atan2(base[1,0],base[0,0]);flat=np.eye(4);flat[:3,:3]=Rotation.from_euler('z',yaw).as_dcm();flat[:2,3]=base[:2,3]
        stamp=self.stamp_seconds(msg.header.stamp)
        out=Odometry();out.header.frame_id='odom';out.child_frame_id='base_link';out.header.stamp=msg.header.stamp;set_pose(out.pose.pose,flat)
        self.valid_pose=True
        if self.prev:
            old,oldyaw,oldstamp=self.prev;dt=stamp-oldstamp
            if 0<dt<.5:
                vel=flat[:3,:3].T@(flat[:3,3]-old[:3,3])/dt
                wz=math.atan2(math.sin(yaw-oldyaw),math.cos(yaw-oldyaw))/dt
                if np.linalg.norm(vel[:2])>1. or abs(wz)>1.5:self.valid_pose=False
                out.twist.twist.linear.x=float(vel[0]);out.twist.twist.linear.y=float(vel[1]);out.twist.twist.angular.z=float(wz)
            else:self.valid_pose=False
        self.prev=(flat,yaw,stamp);self.current=(imu,flat,stamp);self.odom_received=time.monotonic()
        self.tf.sendTransform([tf_msg('odom','base_link',flat,msg.header.stamp),tf_msg('base_link','lidar_imu',np.linalg.inv(flat)@imu,msg.header.stamp)])
        self.odom_pub.publish(out)
    def cloud_cb(self,msg):
        if self.current is None or msg.header.frame_id!='lidar_imu' or not self.fresh(msg.header.stamp):return
        imu,base,stamp=self.current
        if abs(self.stamp_seconds(msg.header.stamp)-stamp)>.02:return
        offsets={f.name:f.offset for f in msg.fields if f.datatype==PointField.FLOAT32}
        if not all(k in offsets for k in ['x','y','z']):return
        # Respect both point and row strides, including padded organized clouds.
        pts=np.stack([np.ndarray((msg.height,msg.width),dtype=('>f4' if msg.is_bigendian else '<f4'),buffer=bytes(msg.data),offset=offsets[k],strides=(msg.row_step,msg.point_step)).ravel() for k in ['x','y','z']],axis=1)
        world=pts@imu[:3,:3].T+imu[:3,3];local=(world-base[:3,3])@base[:3,:3]
        footprint=np.asarray(self.config['footprint']);half=np.max(np.abs(footprint),axis=0)
        valid=np.isfinite(world).all(1)&(world[:,2]>=self.config['obstacle_min_height'])&(world[:,2]<=self.config['obstacle_max_height'])
        valid&=~((np.abs(local[:,0])<half[0])&(np.abs(local[:,1])<half[1]))
        points=np.asarray(world[valid],dtype='<f4')
        out=PointCloud2();out.header.frame_id='map';out.header.stamp=msg.header.stamp;out.height=1;out.width=len(points)
        out.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,k in enumerate(['x','y','z'])]
        out.point_step=12;out.row_step=12*len(points);out.is_dense=True;out.data=points.tobytes();self.cloud_pub.publish(out);self.cloud_received=time.monotonic()
    def health(self):
        now=time.monotonic();q=self.quality;limit=self.config['sensor_timeout_s']
        healthy=bool(self.valid_pose and q and len(q)==4 and np.isfinite(q).all() and q[1]>=100 and q[2]>=80 and q[2]/q[1]>=.25 and q[3]<.12 and all(0<=now-t<limit for t in [self.quality_received,self.odom_received,self.cloud_received]))
        if healthy:healthy= -.1<=self.get_clock().now().nanoseconds*1e-9-q[0]<=limit
        self.health_pub.publish(Bool(data=healthy))

def main():
    p=argparse.ArgumentParser();p.add_argument('--robot',type=Path,required=True);args,ros=p.parse_known_args()
    rclpy.init(args=ros);node=Adapter(yaml.safe_load(args.robot.read_text()))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
