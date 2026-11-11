// SPDX-License-Identifier: Apache-2.0
import {LatestNativeClient,sameIdentity} from './transport.mjs';
import {preparePresentation} from './presentation.mjs';
import {nativeCellMap} from './native_wire_receiver.mjs';
// Only this dedicated module Worker owns HTTP, native verification and RGBA work.
if(typeof WorkerGlobalScope!=='undefined'&&globalThis instanceof WorkerGlobalScope){
  let palette='gray';
  const send=(kind,value,transfer=[])=>postMessage({kind,...value},transfer);
  const requestedPoll=new URL(import.meta.url).searchParams.get('pollMs');
  if(requestedPoll!==null&&!/^[0-9]{3,4}$/.test(requestedPoll))throw Error('client_budget');
  const client=new LatestNativeClient({
    ...(requestedPoll===null?{}:{pollMs:Number(requestedPoll)}),
    prepareView:async(view,active)=>{
      let result;do{const mode=palette;result=await preparePresentation(view,active,mode);active();}while(result.palette!==palette);return result;
    },
    onState:state=>send('state',{state}),
    onCommit:event=>{if(event.prepared.palette!==palette)throw Error('palette_superseded');send('commit',{event},event.prepared.models.map(m=>m.rgba));}
  });
  onmessage=async({data})=>{
    try {
      if(data?.kind==='start')await client.start();
      else if(data?.kind==='refresh')await client.refresh();
      else if(data?.kind==='stop')client.stop('client_stopped');
      else if(data?.kind==='palette'){
        if(!['gray','positive'].includes(data.palette))throw Error('palette_mode');palette=data.palette;
        await client.repaint();
      } else if(data?.kind==='inspect'){
        const view=client.visibleView(),state=client.state();let cell=null;
        if(view&&sameIdentity(data.identity,state.latestComplete)&&Array.isArray(data.candidates)&&data.candidates.length<=256){
          for(const p of data.candidates){const e=view.submaps.find(v=>v.trajectory_id===p.trajectory_id&&v.submap_index===p.submap_index),i=p.index;
            if(e&&Number.isSafeInteger(i)&&i>=0&&i<e.cells&&e.body.knownAt(i)){cell={probability:e.body.probabilityAt(i),map_center:nativeCellMap(e,i%e.crop_dimensions_n0_n1[0],Math.floor(i/e.crop_dimensions_n0_n1[0])),trajectory_id:e.trajectory_id,submap_index:e.submap_index};break;}}
        }send('inspect',{request:data.request,cell});
      }
    } catch(error){send('error',{reason:String(error.message||error).slice(0,512)});}
  };
}
