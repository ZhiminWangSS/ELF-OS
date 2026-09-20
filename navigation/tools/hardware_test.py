#!/usr/bin/env python3
"""Explicit, time-limited hardware bridge supervisor; never auto-stand or navigate."""
import argparse,os,platform,signal,subprocess,time,json
from pathlib import Path
from preflight import check,ROOT

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--seconds',type=float,default=30)
    p.add_argument('--goal-x',type=float);p.add_argument('--goal-y',type=float);p.add_argument('--goal-yaw-deg',type=float,default=0.)
    p.add_argument('--max-distance',type=float,default=1.);p.add_argument('--goal-timeout',type=float,default=15.)
    a=p.parse_args()
    if not 0<a.seconds<=60:raise SystemExit('Test session duration must be 0..60 seconds')
    requested_goal=(a.goal_x is not None or a.goal_y is not None)
    if requested_goal and (a.goal_x is None or a.goal_y is None):raise SystemExit('Provide both --goal-x and --goal-y')
    report=check();print(json.dumps(report,indent=2),flush=True)
    if not a.execute:print('Read-only check complete. --execute is required to start the hardware bridge.');return
    if not report['ready_for_controlled_motion_test']:raise SystemExit('Preflight failed; hardware bridge not started')
    worker_env={'HOME':str(Path.home()),'PATH':'/usr/bin:/bin','LD_LIBRARY_PATH':str(Path.home()/'unitree_sdk2-2.0.2/thirdparty/lib'/platform.machine())}
    # The SDK worker uses Unitree DDS domain 0 explicitly. The ROS bridge must
    # retain the caller's ROS domain so it can receive Nav2 and localization.
    bridge_env=os.environ.copy()
    bridge_env['PATH']='/usr/bin:/bin'
    worker=None;bridge=None;path=f'/tmp/elf-go2-{os.getuid()}/nav.sock'
    log_dir=ROOT/'log'/time.strftime('hardware_%Y%m%d_%H%M%S');log_dir.mkdir()
    try:
        with (log_dir/'sdk.jsonl').open('w') as sdk_log,(log_dir/'ros.log').open('w') as ros_log:
            worker=subprocess.Popen([str(ROOT/'sdk/build/nav_worker'),'eth0','--execute'],env=worker_env,stdout=sdk_log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+3
            while not Path(path).exists() and time.monotonic()<deadline:
                if worker.poll() is not None:raise RuntimeError('SDK worker exited; see log')
                time.sleep(.02)
            if worker.poll() is not None or not Path(path).exists():raise RuntimeError('SDK worker socket unavailable')
            bridge=subprocess.Popen(['/usr/bin/python3',str(ROOT/'tools/velocity_bridge.py'),'--socket',path,'--ros-args','-r','__node:=hardware_velocity_bridge','-r','/navigation/guarded_cmd_vel:=/navigation/sdk_guarded_cmd_vel','-r','/navigation/bridge_status:=/navigation/sdk_bridge_status'],env=bridge_env,stdout=ros_log,stderr=subprocess.STDOUT)
            print(f'Test bridge enabled for at most {a.seconds}s. Ctrl+C stops. Logs: {log_dir}',flush=True)
            if requested_goal:
                command=['/usr/bin/python3',str(ROOT/'tools/goal.py'),'--x',str(a.goal_x),'--y',str(a.goal_y),'--yaw-deg',str(a.goal_yaw_deg),'--navigate','--max-distance',str(a.max_distance),'--timeout',str(a.goal_timeout)]
                result=subprocess.run(command,env=bridge_env,text=True,capture_output=True,timeout=a.goal_timeout+8)
                (log_dir/'goal.log').write_text(result.stdout+result.stderr)
                print(result.stdout,end='',flush=True)
                if result.returncode!=0:raise RuntimeError('Navigation goal did not complete; see goal.log')
            end=time.monotonic()+a.seconds
            while time.monotonic()<end:
                if worker.poll() is not None or bridge.poll() is not None:break
                time.sleep(.05)
    except KeyboardInterrupt:pass
    finally:
        # The worker stops independently on stale input even if the ROS bridge is dead.
        for child in [bridge,worker]:
            if child and child.poll() is None:child.send_signal(signal.SIGINT)
        for child in [bridge,worker]:
            if child:
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
        subprocess.run(['/usr/bin/python3',str(ROOT/'tools/goal.py'),'--cancel'],timeout=8,check=False)
        print('Hardware test bridge closed. Check stop acknowledgements and physical standstill.',flush=True)
if __name__=='__main__':main()
