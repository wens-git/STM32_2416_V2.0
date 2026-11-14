import { reactive, computed } from 'vue'
export const hmi=reactive({ state:{connected:false,mode:'offline',op_mode:0,axes:[],config:{},points:{},limits:{},di:Array(24).fill(false),do:Array(16).fill(false),diag:{},sequence:0}, pending:{},toasts:[],history:[],ports:[],jog:null, native:false })
export const ready=computed(()=>hmi.state.connected)
export const manual=computed(()=>ready.value && hmi.state.op_mode===1)
let bridge, counter=0, lease, resolveReady
const waiting=new Map()
const initialized=new Promise(r=>resolveReady=r)
function receive(raw) {
  const r=JSON.parse(raw), entry=waiting.get(r.id)
  if(!entry) return
  waiting.delete(r.id); clearTimeout(entry.timer); delete hmi.pending[entry.action];
  r.ok ? entry.resolve(r.result) : entry.reject(Error(r.error))
}
export async function start() {
  if(!window.qt?.webChannelTransport) { hmi.native=false; resolveReady(); return }
  await new Promise((resolve,reject)=>{ const script=document.createElement('script'); script.src='qrc:///qtwebchannel/qwebchannel.js'; script.onload=resolve; script.onerror=reject; document.head.appendChild(script) })
  new window.QWebChannel(window.qt.webChannelTransport,channel=>{
    bridge=channel.objects.hmi; hmi.native=true
    bridge.snapshot.connect(raw=>{hmi.state=JSON.parse(raw)})
    bridge.reply.connect(receive); resolveReady(); request('snapshot')
  })
}
export async function request(action,payload={}) {
  await initialized
  if(!bridge) throw Error('请通过“启动上位机.bat”打开桌面程序')
  const id=String(++counter)
  if(!['snapshot','renew','stop'].includes(action)) hmi.pending[action]=true
  return new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{waiting.delete(id); delete hmi.pending[action]; reject(Error('响应超时，结果未知，请读取设备状态后再操作'))},30000)
    waiting.set(id,{resolve,reject,timer,action}); bridge.request(JSON.stringify({id,action,payload}))
  })
}
export function notice(message,good=true,detail='') {
  const id=++counter
  hmi.toasts.unshift({id,message,good,detail}); hmi.toasts.splice(4)
  hmi.history.unshift({id,time:new Date().toLocaleTimeString('zh-CN',{hour12:false}),message,good,detail})
  hmi.history.splice(150); setTimeout(()=>hmi.toasts=hmi.toasts.filter(t=>t.id!==id),4500)
}
export async function run(action,payload={},quiet=false) {
  try { const result=await request(action,payload); if(!quiet && result) notice(result); return true }
  catch(error) { if(!quiet) notice(error.message,false); return false }
}
export async function scanPorts() { try {hmi.ports=await request('ports')} catch(e) {notice(e.message,false)} }
export function stopJog() {
  if(!hmi.jog) return
  hmi.jog=null; clearInterval(lease); run('stop',{},true)
}
export async function startJog(event,axis,direction,speed) {
  if(!manual.value || hmi.jog || hmi.pending.jog) return
  event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId)
  const token=String(++counter); hmi.jog={axis,direction,token}
  const ok=await run('jog',{axis,direction,speed,token},true)
  if(!ok) notice('点动未能启动，已请求停止，请查看设备状态',false)
  if(!hmi.jog || hmi.jog.token!==token) return
  if(!ok) { stopJog(); return }
  lease=setInterval(()=>{if(hmi.jog) run('renew',{token},true)},180)
}
window.addEventListener('blur',stopJog)
document.addEventListener('visibilitychange',()=>{if(document.hidden) stopJog()})
window.addEventListener('pointerup',stopJog)
window.addEventListener('pointercancel',stopJog)
window.addEventListener('keydown',e=>{if(e.key==='Escape'){stopJog(); if(ready.value) run('estop')}})
