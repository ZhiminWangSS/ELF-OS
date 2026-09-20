"""Fail-closed command policy, independent of ROS and SDK."""
import math

class VelocityGuard:
    def __init__(self,timeout=.25,health_timeout=.5):
        self.timeout=timeout;self.health_timeout=health_timeout;self.command=None;self.received=-math.inf
        self.health=False;self.health_time=-math.inf;self.latched=False
    def receive(self,values,now):
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            self.latched=True;return
        vx,vy,vz,wx,wy,wz=values
        if any(abs(v)>1e-6 for v in (vy,vz,wx,wy)) or vx<0:
            self.latched=True;return
        self.command=(min(vx,.20),max(-.35,min(.35,wz)));self.received=now
    def healthy(self,value,now):
        self.health=bool(value);self.health_time=now
    def output(self,now):
        if self.latched or not self.health or not 0<=now-self.health_time<self.health_timeout or not 0<=now-self.received<self.timeout:return (0.,0.)
        return self.command or (0.,0.)
