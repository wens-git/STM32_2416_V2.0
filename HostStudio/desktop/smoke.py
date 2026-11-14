"""Native integration exercise, simulator only. Produces screenshots and exits."""
import json
import time
from pathlib import Path
from PySide6.QtCore import QTimer

SCRIPT = r"""
(async()=>{
  const wait=ms=>new Promise(r=>setTimeout(r,ms));
  const check=(ok,message)=>{if(!ok)throw Error(message);window.smokeChecks.push(message)};
  const capture=async name=>{studio.hmi.toasts=[];await wait(150);window.smokeCapture=name;await wait(700)};
  window.smokeChecks=[];
  try {
    check(!studio.hmi.state.connected,'offline startup');
    await capture('01-overview-offline');
    check(await studio.run('connect',{mode:'mock'}),'QWebChannel mock connection');
    check(studio.hmi.state.axes.length===8,'eight axes');
    await studio.run('mode',{value:1});
    studio.navigate('overview');await wait(300);await capture('02-overview');
    studio.select(4);studio.navigate('axes');await wait(300);
    const edit=document.querySelector('[aria-label="P0 名称"]');
    edit.value='草稿保留';edit.dispatchEvent(new Event('input',{bubbles:true}));
    await wait(700);
    check(edit.value==='草稿保留','polling preserves point draft');
    check(await studio.run('point_write',{axis:4,index:0,position:4321,speed:2500,name:'桌面验证点'}),'point write');
    check(studio.hmi.state.points['4'].positions[0]===4321,'point readback');
    await studio.run('move_abs',{axis:4,target:500,speed:10000});await wait(700);
    check(studio.hmi.state.axes[4].position===500,'absolute motion completion');
    await capture('03-axes');
    studio.navigate('home');await wait(300);
    const direction=document.querySelector('[aria-label="全局回原方向"]');
    check(direction && !direction.disabled,'global home direction selector available');
    direction.value='1';direction.dispatchEvent(new Event('change',{bubbles:true}));
    document.querySelector('.parameter-panel').dispatchEvent(new Event('submit',{bubbles:true,cancelable:true}));
    await wait(400);
    check(studio.hmi.state.config.system.home_direction===1,'direction selector writes positive direction');
    check(await studio.run('home_params',{fast:5000,backoff:300,slow:800,mode:0,direction:0}),'home parameter write/readback');
    check(studio.hmi.state.config.system.home_direction===0,'negative direction readback');
    check(await studio.run('home',{axis:4}),'home accepted');await wait(2200);
    check(studio.hmi.state.axes[4].homed,'home completes asynchronously');
    await capture('04-homing');
    studio.select(0);await wait(100);
    check([...document.querySelectorAll('button')].find(b=>b.textContent.trim()==='回原').disabled,'unbound home button disabled');
    studio.select(4);studio.navigate('axes');await wait(250);
    const jog=document.querySelectorAll('.jog-button')[1];
    jog.dispatchEvent(new PointerEvent('pointerdown',{bubbles:true,pointerId:1,button:0}));
    await wait(450);
    check(studio.hmi.state.axes[4].state===1,'pointer starts jog');
    jog.dispatchEvent(new PointerEvent('pointerup',{bubbles:true,pointerId:1}));await wait(350);
    check(studio.hmi.state.axes[4].state===0,'pointer release stops jog');
    studio.navigate('io');await wait(300);await capture('05-io');
    check(document.querySelectorAll('.studio-io').length===40,'24 inputs and 16 outputs');
    document.querySelector('[aria-label="Y10 置ON"]').click();await wait(400);
    check(studio.hmi.state.do[8]===true,'Y10 ON via UI and device readback');
    document.querySelector('[aria-label="Y10 置OFF"]').click();await wait(400);
    check(studio.hmi.state.do[8]===false,'Y10 OFF via UI and device readback');
    await studio.run('move_abs',{axis:4,target:100000,speed:100});await wait(300);
    check(document.querySelector('[aria-label="Y6 置ON"]').disabled,'moving axis output disabled');
    check(!document.querySelector('[aria-label="Y10 置ON"]').disabled,'unrelated idle output remains available');
    await studio.run('stop',{axis:4});await capture('08-manual-outputs');
    studio.navigate('diagnostics');await wait(300);await capture('06-diagnostics');
    check(studio.hmi.history.length>0,'command history present');
    studio.navigate('axes');await wait(200);await capture('07-axes-1180x760');
    check(document.documentElement.scrollWidth===document.documentElement.clientWidth,'compact window has no horizontal page overflow');
    await studio.run('disconnect');await wait(250);check(!studio.hmi.state.connected,'disconnect');
    window.smokeDone={ok:true,checks:window.smokeChecks};
  } catch(e) {window.smokeDone={ok:false,error:e.message,checks:window.smokeChecks}}
})();
"""

class Smoke:
    def __init__(self,window,report):
        self.window=window; self.report=Path(report); self.report.parent.mkdir(parents=True,exist_ok=True)
        self.start=time.monotonic(); self.launched=False; self.captured=set(); self.errors=[]
        self.timer=QTimer(window); self.timer.timeout.connect(self.tick); self.timer.start(200)
    def tick(self):
        if time.monotonic()-self.start>60:
            self.finish(dict(ok=False,error='native smoke timeout')); return
        if not self.window.loaded: return
        if not self.launched:
            self.window.page.runJavaScript('Boolean(window.studio && window.studio.hmi.native)',self.launch)
        else:
            self.window.page.runJavaScript('JSON.stringify({capture:window.smokeCapture,done:window.smokeDone})',self.observe)
    def launch(self,ready):
        if ready and not self.launched:
            self.launched=True; self.window.activateWindow(); self.window.raise_()
            self.window.page.runJavaScript(SCRIPT)
    def observe(self,raw):
        if not raw: return
        data=json.loads(raw); name=data.get('capture')
        if name and name not in self.captured:
            self.captured.add(name)
            if name=='07-axes-1180x760':
                self.window.resize(1180,760)
                QTimer.singleShot(350,lambda:self.window.grab().save(str(self.report.parent/(name+'.png'))))
            else: self.window.grab().save(str(self.report.parent/(name+'.png')))
        if data.get('done'): self.finish(data['done'])
    def finish(self,result):
        self.timer.stop()
        result['screenshots']=sorted(self.captured)
        self.report.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        self.window.close()
