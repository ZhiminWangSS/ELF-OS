"""Run with an isolated ROS_DOMAIN_ID; no navigation or hardware commands."""
import sys,time
from pathlib import Path as FilePath
sys.path.insert(0,str(FilePath(__file__).resolve().parents[1]/'tools'))
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray,GoalStatus
from rviz_path_bridge import PathDisplayBridge

rclpy.init();bridge=PathDisplayBridge();test=rclpy.create_node('rviz_path_smoke')
def spin(seconds=.6):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(bridge,timeout_sec=.01);rclpy.spin_once(test,timeout_sec=.01)
try:
    pub=test.create_publisher(Path,'/plan',10)
    spin()
    msg=Path();msg.header.frame_id='map';msg.poses=[PoseStamped()]
    pub.publish(msg);spin()
    received=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub=test.create_subscription(Path,'/navigation/display/global_plan',received.append,qos)
    spin();assert received and len(received[-1].poses)==1,'late subscriber missed cached plan'
    status=GoalStatusArray();s=GoalStatus();s.goal_info.goal_id.uuid[0]=1;s.status=2;status.status_list=[s]
    bridge.status(status);s.status=5;bridge.status(status);spin()
    assert not received[-1].poses,'canceled goal retained visible path'
    pub.publish(msg);spin();assert received[-1].poses
    bridge.ttl=.1;spin();assert not received[-1].poses,'expired preview retained path'
    bridge.ttl=60;pub.publish(msg);spin()
    m=OccupancyGrid();m.info.map_load_time.sec=1;bridge.map_changed(m)
    m.info.map_load_time.sec=2;bridge.map_changed(m);spin()
    assert not received[-1].poses,'map reload retained old path'
    local=[];test.create_subscription(Path,'/navigation/display/local_plan',local.append,qos)
    bridge.receive('local_plan',msg);spin();assert local[-1].poses
    spin(2);assert not local[-1].poses,'stale local trajectory retained path'
    print('PASS: late join, cancellation, preview expiry, map reload, local expiry')
finally:
    test.destroy_node();bridge.destroy_node();rclpy.shutdown()
