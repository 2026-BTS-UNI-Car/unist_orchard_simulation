// Producer control only. Display Start/Stop remains in sim-native-view.mjs.
const PREFIX='/api/sim/native-observer';
const states={idle:'관찰 대기',starting:'관찰 준비 중',running:'추가 지도 관찰 중',stopping:'관찰 정리 중',stopped:'관찰 멈춤',failed:'관찰 오류'};
export function ownerLabel(value){return states[value?.state]||'관찰 서비스 연결 대기';}
const token=value=>typeof value==='string'&&/^[0-9a-f]{32}$/.test(value);
const integer=(value,max)=>Number.isSafeInteger(value)&&value>=0&&value<=max;
function available(value,field){
  const flags=value?.engine_diagnostic_fields_available;
  return flags&&typeof flags==='object'&&!Array.isArray(flags)&&Object.keys(flags).sort().join(',')==='first_failure_reason,queue'&&
    typeof flags.first_failure_reason==='boolean'&&typeof flags.queue==='boolean'&&flags[field]===true;
}
function queueLabel(value){
  const q=value?.engine_queue;
  const valid=available(value,'queue')&&q&&typeof q==='object'&&!Array.isArray(q)&&
    Object.keys(q).sort().join(',')==='closed,encoded_bytes,max_bytes,max_packets,packets'&&
    integer(q.packets,64)&&integer(q.encoded_bytes,8388608)&&typeof q.closed==='boolean'&&q.max_packets===64&&q.max_bytes===8388608;
  return valid?`최근 처리 대기 ${q.packets}/${q.max_packets}건`:'처리 대기 확인 불가';
}
function failureLabel(value){
  const reason=value?.engine_first_failure_reason;
  const known=available(value,'first_failure_reason')&&(reason===null||typeof reason==='string'&&reason.length>0&&reason.length<=256);
  if(known&&reason==='native_packet_queue_overflow')return ' · 지도 처리 대기 초과';
  return (known&&reason!==null)||value?.fault!=null||value?.state==='failed'?' · 진단 확인 필요':'';
}

export function attachNativeOwner({document:doc=globalThis.document,active=()=>!doc.hidden,
  request,interval=globalThis.setInterval,clear=globalThis.clearInterval}={}){
  if(doc.documentElement.dataset.nativeOwnerEnabled!=='true')return null;
  if(typeof request!=='function')throw Error('native_owner_request_required');
  const element=id=>doc.getElementById(id),panel=element('simNativeOwnerPanel');
  let current=null,busy=false,polling=false,destroyed=false,paused=false,generation=0,lastProgress={};
  const listeners=[];let timer=null;
  const shown=()=>!destroyed&&!paused&&active()&&panel.open!==false;
  function buttons(){
    const known=token(current?.control_epoch)&&token(current?.source_session);
    element('simNativeOwnerStart').disabled=busy||!known||current?.enabled!==true||!current?.source_context_fresh||current.active||current.priority_active||['starting','running','stopping'].includes(current.state);
    // A stale source never prevents an explicit Stop targeting known ownership.
    element('simNativeOwnerStop').disabled=busy||!known||!(current.active||current.priority_active||['starting','running','stopping'].includes(current.state));
  }
  function renderProgress(value){
    const progress=value?(value.progress||{}):lastProgress;if(value)lastProgress=progress;
    element('simNativeOwnerProgress').textContent=`센서 수신 ${Number(progress.source_callbacks||0).toLocaleString()}건 · 완성 지도 ${Number(progress.complete_batches||0).toLocaleString()}회 · ${queueLabel(value)} · 주행 제어에 사용하지 않음`;
  }
  function render(value){
    current=value;
    element('simNativeOwnerStatus').textContent=ownerLabel(value)+failureLabel(value);
    renderProgress(value);
    buttons();
  }
  async function poll(){
    if(!shown()||polling||busy)return;
    const expected=generation;polling=true;
    try{const value=await request(PREFIX+'/status');if(expected===generation&&shown()&&!busy)render(value);}
    catch{if(expected===generation&&shown()&&!busy){current=null;element('simNativeOwnerStatus').textContent='관찰 서비스 연결 대기';renderProgress(null);buttons();}}
    finally{polling=false;}
  }
  async function action(op){
    if(op!=='start'&&op!=='stop')return;
    if(!shown()||busy||!current||element(op==='start'?'simNativeOwnerStart':'simNativeOwnerStop').disabled)return;
    const expected=++generation;busy=true;buttons();
    try{
      const value=await request(PREFIX+'/'+op,{expected_control_epoch:current.control_epoch,source_session:current.source_session});
      if(expected===generation&&!destroyed)render(value);
    }catch{if(expected===generation&&!destroyed){current=null;element('simNativeOwnerStatus').textContent='요청 결과 확인 필요 · 상태 새로고침';renderProgress(null);}}
    finally{busy=false;if(!destroyed){buttons();void poll();}}
  }
  function routeChanged(){generation++;if(shown())void poll();}
  function listen(target,name,handler){if(target){target.addEventListener(name,handler);listeners.push([target,name,handler]);}}
  panel.hidden=false;
  listen(element('simNativeOwnerStart'),'click',()=>void action('start'));
  listen(element('simNativeOwnerStop'),'click',()=>void action('stop'));
  listen(panel,'toggle',routeChanged);listen(doc,'visibilitychange',routeChanged);
  listen(globalThis.window,'pagehide',event=>{generation++;paused=true;if(!event.persisted)destroy();});
  listen(globalThis.window,'pageshow',()=>{paused=false;routeChanged();});
  function destroy(){if(destroyed)return;destroyed=true;generation++;if(timer!==null)clear(timer);for(const [target,name,handler]of listeners)target.removeEventListener(name,handler);listeners.length=0;}
  buttons();timer=interval(()=>void poll(),1000);void poll();
  return {poll,action,routeChanged,destroy,state:()=>({busy,polling,destroyed,current})};
}
