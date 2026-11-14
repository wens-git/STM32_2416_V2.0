"""Adapters for the board and an explicit, session-only simulator."""
import copy
import time
import serial
from .protocol import ModbusMaster, ModbusExceptionResp

class SerialDevice(ModbusMaster):
    def __init__(self, port, baud=115200, unit=1):
        super().__init__(slave_addr=unit, timeout=.2)
        self.set_serial(serial.Serial(port, baudrate=baud, timeout=.02, write_timeout=.3))

    def _transact(self, func, payload, retries=1, timeout_s=None):
        # An ambiguous write response must never repeat a relative move.
        return super()._transact(func, payload, retries=retries if func in (1,2,3,4) else 0, timeout_s=timeout_s)

    def close(self):
        if self.serial: self.serial.close()

    def read_all_config(self):
        config = super().read_all_config()
        try:
            value = self.read_holding(222, 1)[0]
            if value not in (0, 1): raise ValueError('设备回原方向值无效')
            config['system']['home_direction'] = value
        except ModbusExceptionResp as exc:
            if exc.exc != 2: raise
            config['system']['home_direction'] = None
        return config

    def set_home_direction(self, value):
        self.write_single(222, value)
        if self.read_holding(222, 1)[0] != value: raise ValueError('回原方向读回不一致')

class SimDevice:
    def __init__(self):
        self.axes = [dict(position=0,state=0,homed=False,speed=1250,target=0) for _ in range(8)]
        self.mode = 0
        self.outputs = [False]*16
        self.last = time.monotonic()
        self.home_end = {}
        self.frames = 0
        self.cfg = dict(system=dict(axis_count=8,axis_capacity=8,home_fast=1250,home_slow=300,home_backoff_hz=300,home_mode=0,home_direction=0,subdivision=5000,jog_step=100,comm_wdt_ms=2000),
            axes=[dict(pulse_mode=int(i>=4),pwm=i if i<4 else None,soft=i-4 if i>=4 else None,dir=[0,2,5,15,7,9,11,13][i],home=[None,None,None,None,15,18,9,12][i],limp=None,limn=None,inv=False,home_dir=False) for i in range(8)])
        self.points = [dict(positions=[0,10000,25000,40000,0,0,0,0],names=['初始点','取料点','放料点','等待点','','','',''],speeds=[1250]*8) for _ in range(8)]
        self.limits = [dict(enable=False,min=-100000,max=100000,valid=False) for _ in range(8)]

    def tick(self):
        now=time.monotonic(); dt=min(now-self.last,.5); self.last=now; self.frames+=1
        for i,a in enumerate(self.axes):
            if a['state']==3 and now>=self.home_end.get(i,now+1): a.update(position=0,target=0,state=0,homed=True)
            elif a['state'] in (1,2):
                step=max(1,int(a['speed']*dt))
                if a['state']==1: a['position']+=step*a['direction']
                else:
                    delta=a['target']-a['position']
                    if abs(delta)<=step: a.update(position=a['target'],state=0)
                    else: a['position']+=step if delta>0 else -step
                lim=self.limits[i]
                if lim['enable'] and not lim['min']<=a['position']<=lim['max']:
                    a['position']=max(lim['min'],min(lim['max'],a['position'])); a['state']=0

    def get_version(self): return 0x2416
    def read_all_config(self): return copy.deepcopy(self.cfg)
    def read_op_mode(self): return self.mode
    def write_op_mode(self,mode): self.emergency_stop(); self.mode=mode
    def read_axis_status_range(self,count): self.tick(); return copy.deepcopy(self.axes[:count])
    def read_io(self): return ([i in (3,6,15) for i in range(24)],self.outputs[:])
    def read_diag(self): return dict(wdt_ms=self.cfg['system']['comm_wdt_ms'],wdt_triggered=0,frame_count=self.frames,err_count=0)
    def heartbeat(self): pass
    def read_position_point_data(self,axis): return copy.deepcopy(self.points[axis])
    def read_soft_limit(self,axis): return copy.deepcopy(self.limits[axis])
    def axis_stop(self,axis): self.axes[axis]['state']=0
    def emergency_stop(self):
        for a in self.axes: a['state']=0
    def axis_jog(self,axis,speed,positive): self.axes[axis].update(state=1,speed=speed,direction=1 if positive else -1)
    def axis_move_abs(self,axis,speed,target): self.axes[axis].update(state=2,speed=speed,target=target)
    def axis_move_rel(self,axis,speed,distance): self.axis_move_abs(axis,speed,self.axes[axis]['position']+distance)
    def axis_home(self,axis):
        self.axes[axis].update(state=3,homed=False,direction=1 if self.cfg['system']['home_direction'] else -1); self.home_end[axis]=time.monotonic()+1.8
    def axis_set_position(self,axis,pos): self.axes[axis]['position']=pos
    def axis_resume_remaining(self,axis): self.axes[axis]['state']=2
    def write_position_point_data(self,axis,positions,names,speeds): self.points[axis]=dict(positions=positions[:],names=names[:],speeds=speeds[:])
    def write_soft_limit(self,axis,enable,min_pos,max_pos,**kwargs): self.limits[axis]=dict(enable=enable,min=min_pos,max=max_pos,valid=enable and min_pos<max_pos)
    def set_home_params(self,fast,slow,backoff,mode): self.cfg['system'].update(home_fast=fast,home_slow=slow,home_backoff_hz=backoff,home_mode=mode)
    def set_home_direction(self,value): self.cfg['system']['home_direction']=value
    def set_comm_wdt_ms(self,timeout): self.cfg['system']['comm_wdt_ms']=timeout
    def write_coil(self,index,value): self.outputs[index]=value
    def read_coils(self,start,count): return self.outputs[start:start+count]
    def save_points_to_flash(self): pass
    def close(self): self.emergency_stop()
