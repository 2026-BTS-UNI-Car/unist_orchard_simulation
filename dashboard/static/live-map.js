import {MetricMap} from './metric-map.mjs';
const $=id=>document.getElementById(id),text=(id,value)=>{if($(id))$(id).textContent=value;};
const live=new MetricMap('liveMapCanvas'),archive=new MetricMap('liveArchiveCanvas');
const active=()=>location.pathname.replace(/\/$/,'')==='/rover-control'&&!document.hidden;
let polling=false,timer=null,selected='',archiveEpoch=0,speedBusy=false;
let archiveTimer=null,archivePending=null,archiveTimeS=0,archivePlaying=false,followGeneration=false,archiveGenerating=false;
async function request(path,body,signal){
  const response=await fetch(path,{cache:'no-store',signal:signal?AbortSignal.any([signal,AbortSignal.timeout(8000)]):AbortSignal.timeout(8000),...(body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})});
  const value=await response.json();if(!response.ok||value.ok===false)throw new Error(value.error||value.message||`HTTP ${response.status}`);return value;
}
function paint(value){
  const waiting=value.state==='waiting_for_sensors';
  const restored=value.persistence?.restored&&value.state==='needs_reset';
  live.set(value);text('liveMapState',restored?'저장된 지도':waiting?'센서 동기화 대기':value.active?'그리는 중':value.state==='needs_reset'?'위치 추정 중단':'멈춤');
  $('liveMapStart').disabled=value.active||value.state==='needs_reset';$('liveMapStop').disabled=!value.active;
  text('liveMapStatus',restored?'저장된 지도 유지 · 새 지도를 눌러 그리기를 시작하세요.':`${waiting?'LiDAR·IMU 동기화 대기 · 마지막 지도 유지':value.active?'지도 누적 중':value.state==='needs_reset'?'새 지도 필요':'마지막 지도 유지'}${value.reason?` · ${value.reason}`:''}${value.persistence?.saved===false?' · 원본 셀 저장 실패 · 표시 지도 유지':''}`);
}
for(const operation of ['start','stop','reset'])$('liveMap'+operation[0].toUpperCase()+operation.slice(1)).addEventListener('click',async()=>{
  try{paint(await request('/api/live/map/'+operation,{}));}catch(error){text('liveMapStatus',error.message);}
});
for(const button of document.querySelectorAll('[data-live-zoom]'))button.addEventListener('click',()=>live.zoom(button.dataset.liveZoom));
for(const button of document.querySelectorAll('[data-archive-zoom]'))button.addEventListener('click',()=>archive.zoom(button.dataset.archiveZoom));
async function selectArchive(id){
  if(id===selected)return;selected=id;const epoch=++archiveEpoch;
  clearTimeout(archiveTimer);archiveTimer=null;archivePending?.controller.abort();
  archiveTimeS=0;archivePlaying=false;followGeneration=false;archiveGenerating=false;archive.set({});$('liveArchiveRegenerate').disabled=!id;
  if(!id){text('liveArchiveStatus','기록을 선택하세요.');return;}
  try{await pollArchive(id,epoch);}catch(error){if(epoch===archiveEpoch)text('liveArchiveStatus',error.message);}
}
async function pollArchive(id,epoch){
  if(epoch!==archiveEpoch||archivePending?.epoch===epoch)return;
  const pending={epoch,controller:new AbortController()},target=followGeneration?null:archiveTimeS;
  archivePending=pending;const query=new URLSearchParams();
  if(target!==null)query.set('t_s',(Math.floor(target*1000)/1000).toFixed(3));
  if(archive.imageVersion!==undefined)query.set('since',`${archive.snapshot.map_epoch}:${archive.imageVersion}`);
  let value;
  try{
    value=await request(`/api/live/recordings/${encodeURIComponent(id)}/map?${query}`,undefined,pending.controller.signal);
    if(epoch!==archiveEpoch||(!followGeneration&&(target===null||target>archiveTimeS+.001)))return;
    if(value.grid||value.scan_count===0)archive.set(value);
    const generation=value.generation||{},duration=Number(generation.duration_s)||0;
    archiveGenerating=value.state==='regenerating';$('liveArchiveRegenerate').disabled=archiveGenerating;
    text('liveArchiveStatus',value.regeneration_error?`재생성 실패 · ${value.regeneration_error}`:
      value.state==='regenerating'?(generation.stage==='localizing'?'LiDAR·IMU 위치 추정 계산 중…':`지도 생성 ${Math.round((generation.progress||0)*100)}% · ${(generation.processed_t_s||0).toFixed(1)} / ${duration.toFixed(1)}초`):
      value.needs_regeneration?'시각별 지도 재생성이 필요합니다. 기록으로 재생성을 누르세요.':
      value.state==='interrupted'?'중단된 지도 생성 · 부분 결과 유지 · 다시 재생성할 수 있습니다.':
      value.grid?`${followGeneration?'생성된 전체 지도':'재생 시각의 지도'} · ${(value.frame_t_s||0).toFixed(1)}초 · 표시용 위치 추정`:
      value.history_available?'첫 지도 관측 전 · 재생하면 지도가 쌓입니다.':'저장된 지도 없음 · 기록으로 재생성을 누르세요.');
  }finally{
    if(archivePending===pending)archivePending=null;
    // A paused seek can occur while the previous request is in flight. Its
    // timer may have found that request busy; always service the newest cursor
    // after completion instead of leaving the canvas empty until another seek.
    const cursorChanged=!followGeneration&&(target===null||Math.abs(target-archiveTimeS)>.05);
    if(epoch===archiveEpoch&&(value?.state==='regenerating'||archivePlaying||cursorChanged))scheduleArchive();
  }
}
function scheduleArchive(){
  if(archiveTimer!==null||!selected||!active())return;
  const id=selected,epoch=archiveEpoch;
  archiveTimer=setTimeout(()=>{archiveTimer=null;pollArchive(id,epoch).catch(error=>{if(epoch===archiveEpoch&&error.name!=='AbortError')text('liveArchiveStatus',error.message);});},800);
}
function archiveTime(id,timeS,playing){
  if(id!==selected)return;
  if(playing||Math.abs(timeS-archiveTimeS)>.05)followGeneration=false;
  if(!followGeneration&&Math.max(Number(archive.snapshot.frame_t_s)||0,Number(archive.pendingImage?.snapshot.frame_t_s)||0)>timeS){
    // Cancel a future PNG decoder immediately on backward seek/play-from-zero.
    // The same epoch keeps the user's metric zoom/pan while an earlier frame loads.
    archive.set({map_epoch:archive.snapshot.map_epoch,preserve_view_on_seek:true,scan_count:0,version:0});
  }
  const changed=Math.abs(timeS-archiveTimeS)>.05||Boolean(playing)!==archivePlaying;
  archiveTimeS=Math.max(0,Number(timeS)||0);archivePlaying=Boolean(playing);
  if(changed)scheduleArchive();
}
$('liveArchiveRegenerate').addEventListener('click',async()=>{
  const id=selected,epoch=archiveEpoch;if(!id)return;$('liveArchiveRegenerate').disabled=true;
  try{await request(`/api/live/recordings/${encodeURIComponent(id)}/regenerate`,{});if(epoch!==archiveEpoch)return;archiveGenerating=true;followGeneration=true;await pollArchive(id,epoch);scheduleArchive();}
  catch(error){if(epoch===archiveEpoch)text('liveArchiveStatus',error.message);}
  finally{if(epoch===archiveEpoch)$('liveArchiveRegenerate').disabled=archiveGenerating;}
});
$('hardwareAutonomySpeed').addEventListener('input',()=>text('hardwareAutonomySpeedValue',`${$('hardwareAutonomySpeed').value}%`));
$('hardwareAutonomySpeed').addEventListener('change',async()=>{
  if(speedBusy)return;speedBusy=true;$('hardwareAutonomySpeed').disabled=true;
  try{const value=await request('/api/autonomy/speed',{speed_percent:Number($('hardwareAutonomySpeed').value)});text('hardwareAutonomyStatus',`${value.message} · 속도 ${value.speed_percent}% 저장됨`);}
  catch(error){text('hardwareAutonomyStatus',error.message);}
  finally{speedBusy=false;$('hardwareAutonomySpeed').disabled=false;}
});
async function poll(){
  clearTimeout(timer);if(polling||!active())return;polling=true;
  const results=await Promise.allSettled([request('/api/live/map'+(live.imageVersion!==undefined?`?since=${live.imageVersion}`:'')),request('/api/autonomy')]);
  if(results[0].status==='fulfilled')paint(results[0].value);else text('liveMapStatus',`지도 연결 대기 · 마지막 지도 유지 · ${results[0].reason.message}`);
  if(results[1].status==='fulfilled'&&!speedBusy&&document.activeElement!==$('hardwareAutonomySpeed')){
    $('hardwareAutonomySpeed').value=results[1].value.speed_percent;text('hardwareAutonomySpeedValue',`${results[1].value.speed_percent}%`);
    text('hardwareAutonomyStatus',`${results[1].value.message} · 현재 순항속도 설정 ${results[1].value.speed_percent}%`);
  }
  polling=false;if(active())timer=setTimeout(poll,1000);
}
function routeChanged(){live.draw();archive.draw();if(active()){poll();if(selected)scheduleArchive();}}
window.LiveMap={selectArchive,archiveTime,routeChanged};
document.addEventListener('visibilitychange',routeChanged);window.addEventListener('popstate',routeChanged);
// The main application exposes route changes through the GPS hook.
const gps=window.RoverGps;if(gps){const original=gps.routeChanged;gps.routeChanged=(...args)=>{original?.(...args);routeChanged();};}
setInterval(()=>{if(active()&&!polling)poll();},3000);
routeChanged();
