"""Read live pose and request a path only; export an annotated map."""
import json,math,time
from pathlib import Path
import numpy as np,yaml
from PIL import Image,ImageDraw
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
ROOT=Path(__file__).resolve().parents[1]
rclpy.init();n=rclpy.create_node('inspect_local_plan');live={}
n.create_subscription(Odometry,'/odom',lambda m:live.update(odom=m),10)
n.create_subscription(Bool,'/navigation/localization_healthy',lambda m:live.update(healthy=m.data),10)
def wait(f,seconds=10):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise RuntimeError('Planning timeout')
 return f.result()
end=time.monotonic()+2
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
assert live.get('healthy') and 'odom' in live,'Localization not healthy'
p=live['odom'].pose.pose;start=np.array([p.position.x,p.position.y]);yaw=2*math.atan2(p.orientation.z,p.orientation.w)
folder=ROOT/'maps/floor_1789552084236';cfg=yaml.safe_load((folder/'map.yaml').read_text());origin=np.array(cfg['origin'][:2]);res=cfg['resolution'];grid=np.flipud(np.array(Image.open(folder/'map.pgm')));clear=distance_transform_edt(grid==254)*res
xy=np.floor((start-origin)/res).astype(int)
ys,xs=np.where(clear>.65);points=origin+(np.c_[xs,ys]+.5)*res;delta=points-start;dist=np.linalg.norm(delta,axis=1);angles=np.arctan2(delta[:,1],delta[:,0]);turn=np.abs(np.arctan2(np.sin(angles-yaw),np.cos(angles-yaw)))
valid=(dist>.6)&(dist<1.5);ids=np.flatnonzero(valid);ids=ids[np.argsort(turn[ids]+abs(dist[ids]-1))]
assert len(ids),'No nearby goal has adequate known free clearance'
client=ActionClient(n,ComputePathToPose,'compute_path_to_pose');assert client.wait_for_server(timeout_sec=3)
result=None;goal=None
for i in ids[::max(1,len(ids)//15)][:15]:
 goal=points[i];req=ComputePathToPose.Goal();req.pose.header.frame_id='map';req.pose.header.stamp=n.get_clock().now().to_msg();req.pose.pose.position.x=float(goal[0]);req.pose.pose.position.y=float(goal[1]);req.pose.pose.orientation.w=1.;req.planner_id='GridBased'
 h=wait(client.send_goal_async(req))
 if not h.accepted:continue
 answer=wait(h.get_result_async())
 if answer.status==4 and len(answer.result.path.poses)>1:result=answer.result.path;break
path=np.array([[s.pose.position.x,s.pose.position.y] for s in result.poses]) if result else np.empty((0,2))
report={'mode':'plan_only','robot_xy':start.tolist(),'robot_yaw_deg':math.degrees(yaw),'start_static_clearance_m':float(clear[xy[1],xy[0]]),'goal_xy':goal.tolist(),'planning_succeeded':result is not None,'path_points':len(path),'motion_sent':False,'footprint_verified':False}
if len(path):
 cells=np.floor((path-origin)/res).astype(int);report.update(path_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),all_path_centers_known_free=bool((grid[cells[:,1],cells[:,0]]==254).all()),min_static_center_clearance_m=float(clear[cells[:,1],cells[:,0]].min()))
canvas=Image.fromarray(np.flipud(grid)).convert('RGB');draw=ImageDraw.Draw(canvas)
def pix(v):return (float((v[0]-origin[0])/res),float(grid.shape[0]-1-(v[1]-origin[1])/res))
if len(path):draw.line([pix(v) for v in path],fill=(0,180,0),width=4)
x,y=pix(start);draw.ellipse((x-6,y-6,x+6,y+6),fill=(0,100,255));draw.line([pix(start),pix(start+.7*np.array([math.cos(yaw),math.sin(yaw)]))],fill=(0,100,255),width=3)
gx,gy=pix(goal);draw.ellipse((gx-5,gy-5,gx+5,gy+5),outline=(255,50,0),width=3)
canvas.save(ROOT/'log/live_plan_full.png');canvas.crop((int(x-100),int(y-100),int(x+100),int(y+100))).resize((800,800)).save(ROOT/'log/live_plan_detail.png')
(ROOT/'log/live_plan_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));client.destroy();n.destroy_node();rclpy.shutdown()
