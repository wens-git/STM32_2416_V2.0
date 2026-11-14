<script setup>
import { computed } from 'vue'
import { hmi, ready, manual, run } from './store'
const props=defineProps({index:Number})
const pulseOutputs=[1,3,4,14,6,8,10,12]
const owner=computed(()=>hmi.state.axes.findIndex((a,i)=>pulseOutputs[i]===props.index||hmi.state.config.axes?.[i]?.dir===props.index))
const busy=computed(()=>owner.value>=0 && hmi.state.axes[owner.value]?.state!==0)
const value=computed(()=>ready.value && hmi.state.do[props.index])
const disabled=computed(()=>!manual.value||busy.value||hmi.pending.output)
const reason=computed(()=>!ready.value?'请先连接设备':!manual.value?'请切换调试模式':busy.value?'对应轴运动中，停止后可操作':'点击置 ON 或 OFF，写入后读取设备状态')
</script>
<template>
  <div class="io-cell studio-io output-card" :class="{on:value}" :title="reason">
    <span class="mono">Y{{index.toString(8)}}</span><i class="status-dot"/>
    <small>{{ready?(value?'ON':'OFF'):'—'}}<span v-if="owner>=0"> · {{String.fromCharCode(65+owner)}} 轴{{busy?'运动中':'空闲'}}</span></small>
    <div class="output-actions"><button class="button" :class="{selected:value}" :disabled="disabled" :aria-label="'Y'+index.toString(8)+' 置ON'" @click="run('output',{index,value:true})">置 ON</button><button class="button" :class="{selected:ready&&!value}" :disabled="disabled" :aria-label="'Y'+index.toString(8)+' 置OFF'" @click="run('output',{index,value:false})">置 OFF</button></div>
  </div>
</template>
