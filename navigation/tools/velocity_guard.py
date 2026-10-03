"""Fail-closed command policy, independent of ROS and SDK."""
import math

class VelocityGuard:
    def __init__(self,timeout=.25,health_timeout=.5,health_grace=1.5):
        self.timeout=timeout;self.health_timeout=health_timeout;self.health_grace=health_grace
        self.command=None;self.received=-math.inf
        self.health=False;self.health_time=-math.inf;self.bad_since=None;self.latched=False
    def receive(self,values,now):
        if len(values)!=6 or not all(math.isfinite(v) for v in values):
            self.latched=True;return
        vx,vy,vz,wx,wy,wz=values
        if any(abs(v)>1e-6 for v in (vy,vz,wx,wy)) or vx<0:
            self.latched=True;return
        self.command=(min(vx,.60),max(-.35,min(.35,wz)));self.received=now
    def healthy(self,value,now):
        if value:self.bad_since=None
        elif self.bad_since is None:self.bad_since=now
        self.health=bool(value);self.health_time=now
    def health_ok(self,now):
        # In-place rotation transiently depresses localization quality; ride out
        # dips shorter than health_grace instead of zeroing every command. The
        # flag, freshness windows and latch remain fail-closed beyond that.
        return self.health or (self.bad_since is not None and 0<=now-self.bad_since<self.health_grace)
    def output(self,now):
        if self.latched or not self.health_ok(now) or not 0<=now-self.health_time<self.health_timeout or not 0<=now-self.received<self.timeout:return (0.,0.)
        return self.command or (0.,0.)
