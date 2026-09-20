#!/usr/bin/env python3
"""Read-only readiness report. Does not send goals or SDK movement commands."""
import argparse,json,os,platform,subprocess,time
from pathlib import Path
import numpy as np,yaml
import rclpy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
ROOT=Path(__file__).resolve().parents[1]

def check(seconds=3):
    robot=yaml.safe_load((ROOT/'config/robot.yaml').read_text());report={'geometry_verified':robot.get('verified') is True,'ros_domain':os.environ.get('ROS_DOMAIN_ID'),'motion_sent':False}
    rclpy.init();node=rclpy.create_node('navigation_preflight');health=[];odom=[];commands=[]
    node.create_subscription(Bool,'/navigation/localization_healthy',lambda m:health.append((time.monotonic(),m.data)),10)
    node.create_subscription(Odometry,'/odom',lambda m:odom.append(m),10)
    node.create_subscription(Twist,'/navigation/cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),10)
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    now=time.monotonic();report['localization_ready']=len(health)>=20 and all(v for _,v in health[-20:]) and now-health[-1][0]<.25 and len(odom)>=10
    report['command_quiet']=not any(abs(vx)+abs(vy)+abs(wz)>1e-6 for t,vx,vy,wz in commands if now-t<2)
    if odom:
        p=odom[-1].pose.pose.position;report['base_xy']=[p.x,p.y]
    node.destroy_node();rclpy.shutdown()
    binary=ROOT/'sdk/build/nav_worker';sdk=Path.home()/'unitree_sdk2-2.0.2/thirdparty/lib'/platform.machine()
    env={'HOME':str(Path.home()),'PATH':'/usr/bin:/bin','LD_LIBRARY_PATH':str(sdk)}
    try:
        result=subprocess.run([str(binary),'eth0','--inspect'],env=env,text=True,capture_output=True,timeout=8,check=True)
        state=json.loads(result.stdout.strip().splitlines()[-1]);report['robot']=state
        report['robot_ready']=state['state_age_s']<.5 and state['mode']==1 and state['error_code']==0 and all(np.isfinite(state[k]) and abs(state[k])<.35 for k in ['roll','pitch'])
        report['remote_ready']=state['remote_age_s']<.5 and state['remote_keys']==0 and all(np.isfinite(v) and abs(v)<=.1 for v in state['remote_axes'])
    except Exception as e:report['sdk_error']=str(e);report['robot_ready']=False;report['remote_ready']=False
    report['ready_for_controlled_motion_test']=all(report[k] for k in ['geometry_verified','localization_ready','robot_ready','remote_ready','command_quiet']) and report['ros_domain']=='42'
    (ROOT/'log/preflight.json').write_text(json.dumps(report,indent=2)+'\n');return report

if __name__=='__main__':
    print(json.dumps(check(),indent=2))
