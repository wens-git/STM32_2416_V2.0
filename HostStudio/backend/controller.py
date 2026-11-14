"""One worker owns the transport; Qt never waits for serial I/O."""
from dataclasses import asdict, is_dataclass
import json
import queue
import threading
import time
from PySide6.QtCore import QThread, Signal
from serial.tools import list_ports
from .device import SerialDevice, SimDevice

def integer(value, low, high, name):
    if isinstance(value,bool): raise ValueError(f'{name}必须是整数')
    result=int(value)
    if str(result)!=str(value) and not isinstance(value,int): raise ValueError(f'{name}必须是整数')
    if not low<=result<=high: raise ValueError(f'{name}范围 {low}..{high}')
    return result

class Controller(QThread):
    snapshot=Signal(str)
    reply=Signal(str)

    def __init__(self):
        super().__init__()
        self.jobs=queue.Queue(); self.priority=queue.Queue()
        self.finish=threading.Event(); self.guard=threading.Lock(); self.epoch=0
        self.device=None; self.jog=None; self.failures=0
        self.state=dict(connected=False,mode='offline',op_mode=0,axes=[],di=[False]*24,
            do=[False]*16,config={},points={},limits={},diag={},sequence=0,error='')

    def submit(self,request):
        action=request.get('action')
        with self.guard:
            if action in ('stop','estop','disconnect','connect','mode','suspend'): self.epoch+=1
            item=(self.epoch,request)
            (self.priority if action in ('stop','estop','disconnect','suspend') else self.jobs).put(item)

    def suspend(self): self.submit(dict(id='',action='suspend'))
    def close(self): self.suspend(); self.finish.set()
    def emit_state(self):
        self.state['sequence']+=1
        self.snapshot.emit(json.dumps(self.state,ensure_ascii=False))

    def run(self):
        last_poll=last_hb=0
        try:
            while not self.finish.is_set():
                item=None
                try: item=self.priority.get_nowait()
                except queue.Empty:
                    try: item=self.jobs.get_nowait()
                    except queue.Empty: pass
                if item:
                    epoch,req=item
                    try:
                        if epoch!=self.epoch and req['action'] not in ('stop','estop','disconnect','suspend'):
                            raise ValueError('操作已取消，请重新执行')
                        result=self.execute(req)
                        self.reply.emit(json.dumps(dict(id=req.get('id',''),ok=True,result=result),ensure_ascii=False))
                    except Exception as exc:
                        self.reply.emit(json.dumps(dict(id=req.get('id',''),ok=False,error=str(exc)),ensure_ascii=False))
                now=time.monotonic()
                if self.jog and now>self.jog[1]: self.stop_jog()
                if self.device and now-last_poll>=.15:
                    last_poll=now
                    try:
                        self.poll(); self.failures=0; self.state['error']=''
                    except Exception as exc:
                        self.failures+=1; self.state['error']=str(exc)
                        if self.failures>=3: self.disconnect()
                    self.emit_state()
                if self.device and now-last_hb>=.25:
                    last_hb=now
                    try: self.device.heartbeat()
                    except Exception: pass
                self.msleep(5)
        finally: self.disconnect()

    def poll(self):
        raw=self.device.read_axis_status_range(self.state['config']['system']['axis_count'])
        self.state['axes']=[asdict(a) if is_dataclass(a) else a for a in raw]
        self.state['di'],self.state['do']=self.device.read_io()
        diag=self.device.read_diag()
        self.state['diag']=asdict(diag) if is_dataclass(diag) else diag
        self.state['op_mode']=self.device.read_op_mode()

    def stop_jog(self):
        if self.jog and self.device:
            axis=self.jog[0]; self.jog=None
            try: self.device.axis_stop(axis)
            except Exception:
                try: self.device.emergency_stop()
                except Exception: self.disconnect()

    def disconnect(self):
        if self.device:
            self.stop_jog(); self.device.close()
        self.device=None; self.jog=None
        self.state.update(connected=False,mode='offline',axes=[],points={},limits={},config={})
        self.emit_state()

    def load_axis(self,axis):
        self.state['points'][str(axis)]=self.device.read_position_point_data(axis)
        self.state['limits'][str(axis)]=self.device.read_soft_limit(axis)

    def execute(self,req):
        action=req['action']; p=req.get('payload',{})
        if action=='ports': return [dict(port=x.device,description=x.description) for x in list_ports.comports()]
        if action=='snapshot': self.emit_state(); return None
        if action=='connect':
            self.disconnect(); mode=p.get('mode')
            if mode not in ('mock','serial'): raise ValueError('请选择串口或模拟设备')
            unit=integer(p.get('unit',1),1,247,'从站地址')
            dev=SimDevice() if mode=='mock' else SerialDevice(p['port'],integer(p.get('baud',115200),1200,921600,'波特率'),unit)
            try:
                if dev.get_version()!=0x2416: raise ValueError('设备版本不匹配，需要 0x2416 固件')
                cfg=dev.read_all_config()
                if not 0<=cfg['system']['axis_count']<=8: raise ValueError('轴数量无效')
                self.device=dev
                self.state.update(connected=True,mode=mode,config=cfg,error='',endpoint='模拟设备' if mode=='mock' else p['port'],version='0x2416')
                for axis in range(cfg['system']['axis_count']): self.load_axis(axis)
                self.poll()
            except Exception:
                dev.close(); self.device=None
                self.state.update(connected=False,mode='offline',axes=[],points={},limits={},config={})
                raise
            self.failures=0; self.emit_state()
            return '模拟设备已连接' if mode=='mock' else '设备握手成功'
        if action=='disconnect': self.disconnect(); return '连接已断开'
        if action=='suspend': self.stop_jog(); return None
        if action=='stop' and not self.device: return None
        if not self.device: raise ValueError('请先连接设备')
        dev=self.device
        if action in ('stop','estop'):
            self.jog=None
            try:
                if action=='estop' or 'axis' not in p: dev.emergency_stop()
                else: dev.axis_stop(integer(p['axis'],0,len(self.state['axes'])-1,'轴号'))
            except Exception:
                self.disconnect()
                raise
            return '停止指令已确认'
        if action=='renew':
            if self.jog and p.get('token')==self.jog[2]: self.jog=(self.jog[0],time.monotonic()+.65,self.jog[2])
            return None
        if action=='refresh':
            self.state['config']=dev.read_all_config()
            for axis in range(len(self.state['axes'])): self.load_axis(axis)
            self.emit_state(); return '设备数据已重新读取'
        if action=='mode':
            self.stop_jog(); dev.write_op_mode(integer(p['value'],0,1,'运行模式'))
            self.state['op_mode']=dev.read_op_mode(); self.emit_state(); return '运行模式已切换'
        if self.state['op_mode']!=1: raise ValueError('请先切换到调试模式')
        if action=='home_params':
            self.poll()
            if any(a['state'] for a in self.state['axes']): raise ValueError('所有轴空闲时才能修改全局回原参数')
            fast=integer(p['fast'],100,100000,'快找频率'); slow=integer(p['slow'],100,100000,'慢找频率')
            back=integer(p['backoff'],100,65535,'回退频率')
            direction=p.get('direction')
            if direction is not None:
                direction=integer(direction,0,1,'回原方向')
                if self.state['config']['system'].get('home_direction') is None:
                    raise ValueError('当前固件不支持全局回原方向，请先烧录新版固件')
            dev.set_home_params(fast,slow,back,integer(p['mode'],0,1,'回原模式'))
            if direction is not None: dev.set_home_direction(direction)
            self.state['config']=dev.read_all_config(); self.emit_state(); return '回原参数已写入并读回'
        if action=='watchdog':
            dev.set_comm_wdt_ms(integer(p['value'],1000,60000,'通信看门狗'))
            self.state['config']=dev.read_all_config(); self.emit_state(); return '看门狗参数已写入'
        if action=='flash':
            if any(a['state'] for a in self.state['axes']): raise ValueError('所有轴空闲时才能保存 Flash')
            dev.save_points_to_flash()
            return '模拟保存完成（仅本次会话）' if self.state['mode']=='mock' else '设备 Flash 保存已确认'
        if action=='output':
            index=integer(p['index'],0,15,'输出编号')
            if not isinstance(p.get('value'),bool): raise ValueError('输出状态必须为 ON 或 OFF')
            self.poll()
            if self.state['op_mode']!=1: raise ValueError('请先切换到调试模式')
            pulse_outputs=[1,3,4,14,6,8,10,12]
            for i,a in enumerate(self.state['axes']):
                binding=self.state['config']['axes'][i]
                if index in (pulse_outputs[i],binding['dir']) and a['state']!=0:
                    raise ValueError('该 Y 点对应的轴正在运动，请先停止该轴')
            dev.write_coil(index,p['value'])
            actual=bool(dev.read_coils(index,1)[0])
            self.state['do'][index]=actual; self.emit_state()
            if actual!=p['value']: raise ValueError('输出状态读回不一致')
            return f'Y{index:o} 已置为 {"ON" if actual else "OFF"}，设备已读回'
        axis=integer(p.get('axis',-1),0,len(self.state['axes'])-1,'轴号')
        self.poll()
        if action=='axis_data': self.load_axis(axis); self.emit_state(); return None
        if self.state['axes'][axis]['state']!=0: raise ValueError('轴正在运动，请先停止')
        if action=='home':
            if self.state['config']['axes'][axis]['home'] is None: raise ValueError('该轴未绑定原点传感器，请在固件轴模板中配置')
            dev.axis_home(axis)
        elif action=='jog':
            jog_speed=integer(p['speed'],100,100000,'速度')
            self.jog=(axis,time.monotonic()+.65,str(p['token']))
            try: dev.axis_jog(axis,jog_speed,p['direction']==1)
            except Exception:
                self.stop_jog()
                raise
        elif action in ('move_abs','move_rel'):
            fn=dev.axis_move_abs if action=='move_abs' else dev.axis_move_rel
            fn(axis,integer(p['speed'],100,100000,'速度'),integer(p['target'],-2147483648,2147483647,'位置'))
        elif action=='zero': dev.axis_set_position(axis,0)
        elif action=='resume': dev.axis_resume_remaining(axis)
        elif action=='point_run':
            index=integer(p['index'],0,7,'点位号'); data=dev.read_position_point_data(axis)
            dev.axis_move_abs(axis,data['speeds'][index],data['positions'][index])
        elif action=='point_write':
            index=integer(p['index'],0,7,'点位号'); data=dev.read_position_point_data(axis)
            data['positions'][index]=self.state['axes'][axis]['position'] if p.get('teach') else integer(p['position'],-2147483648,2147483647,'点位位置')
            data['speeds'][index]=integer(p['speed'],100,100000,'点位速度'); name=str(p['name'])
            if len(name.encode('utf-16-le'))>22: raise ValueError('名称最多 11 个 UTF-16 单元')
            data['names'][index]=name
            dev.write_position_point_data(axis,data['positions'],data['names'],data['speeds'])
            self.load_axis(axis); self.emit_state(); return '点位已写入 RAM 并读回；可保存到 Flash'
        elif action=='limit':
            low=integer(p['min'],-2147483648,2147483647,'负向软限位'); high=integer(p['max'],-2147483648,2147483647,'正向软限位')
            if p['enable'] and low>=high: raise ValueError('负向软限位必须小于正向软限位')
            dev.write_soft_limit(axis,bool(p['enable']),low,high,save=True)
            self.load_axis(axis); self.emit_state(); return '软限位已写入并读回'
        else: raise ValueError('未知操作')
        return '指令已确认，运动结果请查看轴状态'
