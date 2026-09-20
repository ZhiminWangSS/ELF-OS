#!/usr/bin/env python3
"""Plan by default; --navigate sends a bounded short test goal. --cancel cancels Nav2."""
import argparse,math,time,json,os,stat
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from std_msgs.msg import Bool
ROOT=Path(__file__).resolve().parents[1]

def wait(n,future,seconds):
    end=time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    if not future.done():raise RuntimeError('Action response timeout')
    return future.result()

def stop_worker():
    directory=Path(f'/tmp/elf-go2-{os.getuid()}')
    if not directory.exists():return
    info=directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o077:
        raise RuntimeError('Unsafe motion runtime directory')
    fd=os.open(str(directory/'cancel'),os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
    with os.fdopen(fd,'w') as f:f.write('navigation test ended\n')

def cancel(n):
    client=n.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
    if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Nav2 cancel service unavailable')
    response=wait(n,client.call_async(CancelGoal.Request()),3)
    print(json.dumps({'cancel_response':response.return_code,'goals_canceling':len(response.goals_canceling)}))

def main():
    p=argparse.ArgumentParser();p.add_argument('--x',type=float);p.add_argument('--y',type=float);p.add_argument('--yaw-deg',type=float,default=0.)
    p.add_argument('--navigate',action='store_true');p.add_argument('--cancel',action='store_true');p.add_argument('--timeout',type=float,default=15.)
    p.add_argument('--max-distance',type=float,default=1.,help='Explicit hardware-test goal radius; capped at 3.5 m')
    p.add_argument('--allow-long-distance',action='store_true',help='Allow a goal beyond the short supervised-test radius')
    a=p.parse_args();rclpy.init();n=rclpy.create_node('navigation_goal_client');client=None;active=False
    try:
        if a.cancel:stop_worker();cancel(n);return
        if a.x is None or a.y is None or not np.isfinite([a.x,a.y,a.yaw_deg]).all():raise ValueError('Provide finite --x, --y, --yaw-deg')
        if not 0<=a.timeout<=60 or (a.timeout==0 and not a.allow_long_distance):raise ValueError('Timeout must be 0..60 seconds; 0 is reserved for long-distance navigation')
        if not (0<a.max_distance<=3.5 or (a.allow_long_distance and 0<a.max_distance<=1000)):raise ValueError('Max distance must be 0..3.5 metres, or 0..1000 metres with --allow-long-distance')
        health=[];poses=[]
        n.create_subscription(Bool,'/navigation/localization_healthy',lambda m:health.append((time.monotonic(),m.data)),10)
        n.create_subscription(Odometry,'/odom',lambda m:poses.append(m),10)
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        if len(health)<20 or not all(v for _,v in health[-20:]) or time.monotonic()-health[-1][0]>.25 or not poses:raise RuntimeError('Localization is not healthy; no goal sent')
        robot=yaml.safe_load((ROOT/'config/robot.yaml').read_text());folder=ROOT/'maps/floor_1789552084236';config=yaml.safe_load((folder/'map.yaml').read_text());grid=np.flipud(np.array(Image.open(folder/config['image'])))
        x,y=np.floor((np.array([a.x,a.y])-config['origin'][:2])/config['resolution']).astype(int)
        radius=max(np.linalg.norm(v) for v in robot['footprint'])+.05
        clearance=distance_transform_edt(grid==254)*config['resolution']
        if not (0<=y<grid.shape[0] and 0<=x<grid.shape[1]) or clearance[y,x]<radius:raise ValueError('Goal lacks verified free space for the whole footprint')
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=n.get_clock().now().to_msg();pose.pose.position.x=a.x;pose.pose.position.y=a.y;pose.pose.orientation.z=math.sin(math.radians(a.yaw_deg)/2);pose.pose.orientation.w=math.cos(math.radians(a.yaw_deg)/2)
        if a.navigate:
            here=poses[-1].pose.pose.position
            if math.hypot(a.x-here.x,a.y-here.y)>a.max_distance:raise ValueError('Goal exceeds the explicit hardware-test distance limit')
            client=ActionClient(n,NavigateToPose,'navigate_to_pose');request=NavigateToPose.Goal();request.pose=pose
        else:
            client=ActionClient(n,ComputePathToPose,'compute_path_to_pose');request=ComputePathToPose.Goal();request.pose=pose;request.planner_id='GridBased'
        if not client.wait_for_server(timeout_sec=3):raise RuntimeError('Nav2 action unavailable')
        handle=wait(n,client.send_goal_async(request),3)
        if not handle.accepted:raise RuntimeError('Goal rejected')
        active=a.navigate;future=handle.get_result_async();end=time.monotonic()+a.timeout
        while not future.done() and (a.timeout==0 or time.monotonic()<end):
            rclpy.spin_once(n,timeout_sec=.05)
            if a.navigate and (not health[-1][1] or time.monotonic()-health[-1][0]>.25):raise RuntimeError('Localization lost; cancelling')
        if not future.done():raise RuntimeError('Goal timed out; cancelling')
        result=future.result();active=False
        print(json.dumps({'status':result.status,'mode':'navigate' if a.navigate else 'plan_only','path_points':len(result.result.path.poses) if not a.navigate else None}))
        if result.status!=4:raise RuntimeError('Nav2 did not succeed')
    except KeyboardInterrupt:pass
    finally:
        if a.navigate:stop_worker()
        if active:
            try:cancel(n)
            except Exception as e:print(str(e))
        if client:client.destroy()
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
