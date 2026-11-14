import sys
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from PySide6.QtCore import QCoreApplication
from backend.controller import Controller
from backend.device import SerialDevice
from unittest.mock import patch
app=QCoreApplication.instance() or QCoreApplication([])

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.c=Controller(); self.call('connect',mode='mock')
    def tearDown(self): self.c.disconnect()
    def call(self,action,**payload): return self.c.execute(dict(action=action,payload=payload))
    def test_read_only_connect(self):
        self.assertEqual(self.c.state['op_mode'],0); self.assertEqual(len(self.c.state['axes']),8)
        with self.assertRaisesRegex(ValueError,'调试模式'): self.call('jog',axis=4,speed=1250,direction=1,token='x')
    def test_unbound_home(self):
        self.call('mode',value=1)
        with self.assertRaisesRegex(ValueError,'原点传感器'): self.call('home',axis=0)
        self.assertEqual(self.c.device.axes[0]['state'],0)
    def test_homing_waits_for_completion(self):
        self.call('mode',value=1); self.call('home',axis=4); self.c.poll()
        self.assertEqual(self.c.state['axes'][4]['state'],3); self.assertFalse(self.c.state['axes'][4]['homed'])
        self.c.device.home_end[4]=time.monotonic()-1; self.c.poll()
        self.assertTrue(self.c.state['axes'][4]['homed'])
    def test_jog_and_suspend(self):
        self.call('mode',value=1); self.call('jog',axis=4,speed=1250,direction=1,token='x')
        self.assertIsNotNone(self.c.jog); self.call('suspend'); self.c.poll()
        self.assertIsNone(self.c.jog); self.assertEqual(self.c.state['axes'][4]['state'],0)
    def test_ambiguous_jog_stops(self):
        self.call('mode',value=1)
        original=self.c.device.axis_jog
        def lost_response(*args):
            original(*args)
            raise TimeoutError('lost response')
        with patch.object(self.c.device,'axis_jog',side_effect=lost_response):
            with self.assertRaises(TimeoutError): self.call('jog',axis=4,speed=1250,direction=1,token='x')
        self.assertEqual(self.c.device.axes[4]['state'],0); self.assertIsNone(self.c.jog)
    def test_suspend_does_not_stop_position_move(self):
        self.call('mode',value=1); self.call('move_abs',axis=4,speed=100,target=100000)
        self.call('suspend'); self.c.poll(); self.assertEqual(self.c.state['axes'][4]['state'],2)
    def test_point_round_trip(self):
        self.call('mode',value=1); self.call('point_write',axis=4,index=2,name='放料位置',position=-123456,speed=88000)
        p=self.c.state['points']['4']; self.assertEqual(p['positions'][2],-123456)
        self.assertEqual(p['names'][2],'放料位置'); self.assertEqual(p['speeds'][2],88000)
        self.call('point_run',axis=4,index=2); self.c.poll(); self.assertEqual(self.c.state['axes'][4]['target'],-123456)
    def test_range_rejected_before_writes(self):
        self.call('mode',value=1)
        with self.assertRaises(ValueError): self.call('move_rel',axis=4,speed=100,target=2147483648)
        with self.assertRaises(ValueError): self.call('home_params',fast=5000,slow=300,backoff=70000,mode=0)
        with self.assertRaises(ValueError): self.call('limit',axis=4,enable=True,min=100,max=-100)
        self.assertEqual(self.c.device.axes[4]['state'],0)
    def test_stop_invalidates_queued_commands(self):
        self.c.submit(dict(action='jog',payload={}))
        old,_=self.c.jobs.get_nowait(); self.c.submit(dict(action='stop')); self.assertNotEqual(old,self.c.epoch)
    def test_output_owned_by_axis(self):
        self.call('mode',value=1)
        self.call('output',index=3,value=True)
        self.assertTrue(self.c.state['do'][3])
        self.call('output',index=3,value=False)
        self.assertFalse(self.c.state['do'][3])
        self.call('move_abs',axis=1,speed=100,target=100000)
        with self.assertRaisesRegex(ValueError,'正在运动'): self.call('output',index=3,value=True)
        with self.assertRaisesRegex(ValueError,'正在运动'): self.call('output',index=2,value=True)
        self.call('output',index=8,value=True)
        self.assertTrue(self.c.state['do'][8])
    def test_output_requires_debug_and_boolean(self):
        with self.assertRaisesRegex(ValueError,'调试模式'): self.call('output',index=3,value=True)
        self.call('mode',value=1)
        with self.assertRaises(ValueError): self.call('output',index=16,value=True)
        with self.assertRaises(ValueError): self.call('output',index=3,value='false')
    def test_output_readback_mismatch(self):
        self.call('mode',value=1)
        with patch.object(self.c.device,'read_coils',return_value=[False]):
            with self.assertRaisesRegex(ValueError,'读回不一致'): self.call('output',index=8,value=True)
    def test_global_home_direction(self):
        self.call('mode',value=1)
        self.call('home_params',fast=5000,slow=800,backoff=300,mode=0,direction=1)
        self.assertEqual(self.c.state['config']['system']['home_direction'],1)
        self.call('home',axis=4)
        self.assertEqual(self.c.device.axes[4]['direction'],1)
        self.call('stop',axis=4)
        self.call('home_params',fast=5000,slow=800,backoff=300,mode=0,direction=0)
        self.call('home',axis=5)
        self.assertEqual(self.c.device.axes[5]['direction'],-1)
    def test_old_firmware_direction_rejected_before_parameters(self):
        self.call('mode',value=1)
        self.c.state['config']['system']['home_direction']=None
        with patch.object(self.c.device,'set_home_params') as write:
            with self.assertRaisesRegex(ValueError,'新版固件'):
                self.call('home_params',fast=5000,slow=800,backoff=300,mode=0,direction=1)
            write.assert_not_called()
    def test_direction_adapter_register_and_legacy(self):
        from backend.protocol import ModbusExceptionResp
        dev=object.__new__(SerialDevice)
        with patch('backend.protocol.ModbusMaster.read_all_config',return_value={'system':{}}):
            with patch.object(dev,'read_holding',side_effect=ModbusExceptionResp(3,2)):
                self.assertIsNone(dev.read_all_config()['system']['home_direction'])
            with patch.object(dev,'read_holding',side_effect=TimeoutError()):
                with self.assertRaises(TimeoutError): dev.read_all_config()
        with patch.object(dev,'write_single') as write, patch.object(dev,'read_holding',return_value=[1]):
            dev.set_home_direction(1); write.assert_called_once_with(222,1)
    def test_read_retries_write_no_retry(self):
        dev=object.__new__(SerialDevice)
        with patch('backend.protocol.ModbusMaster._transact',return_value=b'') as transact:
            dev._transact(6,b''); self.assertEqual(transact.call_args.kwargs['retries'],0)
            dev._transact(3,b''); self.assertEqual(transact.call_args.kwargs['retries'],1)

if __name__=='__main__': unittest.main()
