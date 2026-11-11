import {MetricMap} from './metric-map.mjs';
import {virtualGpsPoint} from './sim-map-model.mjs';
import {renderJobs} from './mapping-jobs.mjs';
import {attachExternalNativePC} from './external-native-pc.mjs';

// This module is imported only for the isolated simulation dashboard.
for (const element of document.querySelectorAll('[data-simulation-only]')) element.hidden=false;
const $ = id => document.getElementById(id);
const text = (id, value) => { if ($(id)) $(id).textContent = value; };
const active = () => location.pathname.replace(/\/$/, '') === '/rover-control' && !document.hidden;
const externalNativePC=document.documentElement.dataset.externalNativePc==='true';
const externalNativeView=externalNativePC?attachExternalNativePC({active}):null;
let nativeViewController=null;
if(document.documentElement.dataset.nativeViewEnabled==='true'){
  import('./sim-native-view.mjs').then(({attachNativeView})=>{nativeViewController=attachNativeView({active});})
    .catch(()=>{text('simNativeMapStatus','선택형 네이티브 표시를 불러오지 못했습니다. 기존 지도 유지');});
}
if(document.documentElement.dataset.nativeOwnerEnabled==='true'){
  import('./sim-native-owner.mjs').then(({attachNativeOwner})=>{globalThis.SimNativeOwner=attachNativeOwner({active,request});})
    .catch(()=>{text('simNativeOwnerStatus','관찰 제어 화면 연결 대기');});
}
window.addEventListener('pagehide',event=>{if(event.persisted)nativeViewController?.stop();else nativeViewController?.destroy();});
const format = (value, digits=1) => Number.isFinite(value) ? value.toFixed(digits) : '—';
async function request(url, body) {
  if(externalNativePC&&url.startsWith('/api/sim/'))throw new Error('externally_managed_native_mapping');
  const response = await fetch(url, {cache:'no-store', signal:AbortSignal.timeout(8000),
    ...(body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)})});
  const payload = await response.json();
  if (!response.ok || payload.ok === false) throw new Error(payload.error || payload.message || `HTTP ${response.status}`);
  return payload;
}


const live=new MetricMap('simMapCanvas'), archived=new MetricMap('simArchiveMapCanvas');
const historical=$('simHistoryMapCanvas')?new MetricMap('simHistoryMapCanvas'):null;
function viewButtons(kind,map){
  for(const button of document.querySelectorAll(`[data-map-view="${kind}"][data-map-zoom]`)){
    const mode=button.dataset.mapZoom;
    if(mode==='fit'||mode==='full'){
      const selected=map.auto && map.fitMode===(mode==='fit'?'nearby':'full');
      button.classList.toggle('active',selected);button.setAttribute('aria-pressed',String(selected));
    }
  }
}
live.afterDraw=()=>viewButtons('live',live);archived.afterDraw=()=>viewButtons('archive',archived);
if(historical)historical.afterDraw=()=>viewButtons('history',historical);
const overview=$('roverArchiveOverviewToggle');
if(overview){
  try{overview.checked=localStorage.getItem('ubts-archive-overview-v1')==='1';}catch{}
  const updateOverview=()=>{
    document.querySelector('.archive-preview')?.classList.toggle('sensor-overview',overview.checked);
    try{localStorage.setItem('ubts-archive-overview-v1',overview.checked?'1':'0');}catch{}
  };
  overview.addEventListener('change',updateOverview);updateOverview();
}
for(const button of document.querySelectorAll('[data-map-zoom]'))button.addEventListener('click',()=> {if(!externalNativePC)(button.dataset.mapView==='archive'?archived:button.dataset.mapView==='history'?historical:live)?.zoom(button.dataset.mapZoom);});
let historyTicket=0;
function historyDate(value){const date=new Date(value);return Number.isFinite(date.getTime())?date.toLocaleString('ko-KR',{month:'numeric',day:'numeric',hour:'2-digit',minute:'2-digit'}):'시각 미상';}
function historyLocalization(value){return value==='lidar_scanmatch_imu_heading_without_loop_closure'?'LiDAR·IMU 위치 추정':value==='wheel_distance_imu_heading_without_loop_closure'?'휠·IMU 위치 추정':'이전 위치 추정';}
async function loadHistoryList(){
  if(externalNativePC)return;
  const select=$('simMapHistorySelect');if(!select)return;
  try{
    const result=await request('/api/sim/map-history');
    for(const item of result.items){const option=document.createElement('option');option.value=item.id;option.textContent=`${historyDate(item.captured_at)} · ${item.scan_count||0} scans`;select.append(option);}
    if(!result.items.length)text('simMapHistoryStatus','아직 보존된 지도가 없습니다.');
  }catch(error){text('simMapHistoryStatus',error.message);}
}
$('simMapHistorySelect')?.addEventListener('change',async event=>{
  const ticket=++historyTicket,id=event.target.value;if(!id){historical.set({});return;}
  try{
    const result=await request(`/api/sim/map-history/${encodeURIComponent(id)}`);
    if(ticket!==historyTicket)return;
    historical.set({...result.snapshot,map_epoch:`history:${result.id}`});
    text('simMapHistoryStatus',`${historyDate(result.captured_at)} · ${historyLocalization(result.snapshot.localization)} · 보존된 화면·경로 · 이어 그리기·재생성 불가`);
  }catch(error){if(ticket===historyTicket)text('simMapHistoryStatus',error.message);}
});
loadHistoryList();
let snapshot={},busy=false,mapError='',pollTimer=null,polling=false,selectedId='',archiveRequest=0;
let lastStatusPoll=0,gpsPolling=false,autonomyPolling=false,pollGeneration=0;
function renderMapping(payload){
  if(externalNativePC)return;
  snapshot=payload;live.set(payload);
  const n=payload.navigation||{},f=n.feedback||{},s=n.space||{},c=n.corridor||{};
  const reasons={waiting_native_mapping:'지도 시작 대기',mapping_stopped:'지도 정지',observation_expired:'최신 관측 대기',
    corridor_geometry_unavailable:'통로 모호 · 판단 보류',awaiting_persistent_corridor:'통로 연속 관측 중',
    preview_only_output_inhibited:'전진 후보 · 출력 차단',current_space_occupied:'장애물 · 정지 판단',
    current_space_unknown:'미관측 공간 · 정지 판단',body_feedback_unavailable:'이동량 관측 대기',
    measured_geometry_unavailable:'위치 정합 불확실',native_scan_receipt_expired:'센서 지연 · 판단 보류'};
  text('simNavigationStatus',`${reasons[n.reason]||n.reason||'관측 대기'} · 후보 ${format(n.proposed_linear_mps,2)} m/s / ${format(n.proposed_yaw_rate_rad_s,2)} rad/s · 실제 출력 0`);
  text('simNavigationFeedback',`LiDAR 몸체 속도 ${f.valid?format(f.forward_mps,3):'—'} m/s · 측방 ${f.valid?format(f.lateral_mps,3):'—'} m/s · 회전 ${f.valid?format(f.yaw_rate_rad_s,3):'—'} rad/s · 엔코더 미사용`);
  text('simNavigationSpace',`통로 폭 ${format(c.width_m,2)} m · 관측 나이 ${format(n.receipt_age_s,2)} s · 계산 ${format(n.total_ms)} ms · 대기 ${n.queued||0} · 최신 관측으로 교체 ${n.replaced_observations||0}회 · 자유/장애물/미관측 ${s.free_cells??'—'}/${s.occupied_cells??'—'}/${s.unknown_cells??'—'}`);
  const discontinuity=payload.state==='reset_detected';
  const restored=discontinuity&&payload.persistence?.restored;
  $('simMapStart').disabled=busy||payload.active===true||discontinuity;$('simMapStop').disabled=busy||!payload.active;$('simMapReset').disabled=busy||payload.active===true;
  $('simMapStart').textContent=payload.scan_count>0?'이어서 그리기':'그리기 시작';
  text('simMapState',restored?'저장된 지도':discontinuity?'좌표 연결 끊김 · 지도 유지':payload.active?'그리는 중':payload.scan_count>0?'지도 유지':'대기');
  const grid=payload.grid;
  text('simMapStatus',`${restored?'저장된 지도 유지 · 새 지도를 눌러 그리기를 시작하세요.':discontinuity?'센서 시간·수신 연결이 바뀌었습니다. 새 지도를 시작해 주세요.':payload.active?'실시간 작성':payload.scan_count>0?'정지 · 마지막 지도 유지':'시작 대기'}${payload.no_encoder?' · LiDAR·IMU·GPS / 엔코더 미사용':''}${grid?` · ${format(grid.width*grid.resolution)} × ${format(grid.height*grid.resolution)} m · ${format(grid.resolution,2)} m/cell`:''}${payload.limited?' · 지도 크기 제한 도달':''}${payload.message?` · ${payload.message}`:''}${payload.error?` · ${payload.error}`:''}${mapError?` · ${mapError}`:''}`);
}
async function mappingAction(action){
  if(externalNativePC)return;
  if(busy)return;busy=true;mapError='';renderMapping(snapshot);
  try{renderMapping(await request(`/api/sim/map/${action}`,{expected_epoch:snapshot.map_epoch}));}
  catch(error){mapError=error.message;}
  finally{busy=false;renderMapping(snapshot);}
}
$('simMapStart').addEventListener('click',()=>mappingAction('start'));
$('simMapStop').addEventListener('click',()=>mappingAction('stop'));
$('simMapReset').addEventListener('click',()=>mappingAction('reset'));
let mappingJobsBusy=false;
async function pollMappingJobs(){
  if(externalNativePC)return;
  if(mappingJobsBusy||!active()||!$('simMappingJobsPanel')?.open)return;
  mappingJobsBusy=true;
  try{
    const value=await request('/api/sim/mapping-jobs'),r=value.resources||{};
    text('simMappingJobsResources',`실시간 우선 · CPU ${format(r.cpu_percent_one_core)}% (1코어 기준) · ${r.scope==='native_mapper_process'?'지도 프로세스':'대시보드'} 메모리 ${format(r.rss_bytes/1048576,0)} MB · Gazebo·센서 사용량은 별도`);
    renderJobs($('simMappingJobsList'),value,async job=>{
      try{await request('/api/sim/mapping-jobs/stop',{job_id:job.id,expected_epoch:job.epoch,...(job.kind==='native-observer'?{source_session:job.source_session}:{})});if(job.kind==='live')renderMapping(await request('/api/sim/map?transport=tiles'));}
      catch(error){text('simMappingJobsResources',error.message);}
    });
  }catch(error){text('simMappingJobsResources',error.message);}
  finally{mappingJobsBusy=false;}
}
$('simMappingJobsPanel')?.addEventListener('toggle',pollMappingJobs);
setInterval(pollMappingJobs,1500);

// Keep the GPS panel usable with no country tile download and no external network.
const gpsPanel=document.querySelector('.rover-gps-panel');
gpsPanel.querySelector('.eyebrow').textContent='Simulation · Virtual GPS';
gpsPanel.querySelector('h2').textContent='가상 GPS · 과수원 지도';
gpsPanel.querySelectorAll('.gps-caption').forEach(element=>element.textContent='');
gpsPanel.querySelector('.gps-caption:last-of-type').textContent='가상 원점 (37°, 127°) · Gazebo 위치를 좌표로 변환 · 실제 GNSS·RTK 측정값 아님';
gpsPanel.querySelector('.gps-diagnostics').hidden=true;
for(const [container,id] of [['roverGpsMap','simGpsCanvas'],['roverArchiveGpsMap','simArchiveGpsCanvas']]){
  $(container).classList.add('sim-gps-map');const canvas=document.createElement('canvas');canvas.id=id;canvas.setAttribute('aria-label','실제 지리가 아닌 가상 과수원 지도');$(container).append(canvas);
}
const gpsLive=new MetricMap('simGpsCanvas',true),gpsArchive=new MetricMap('simArchiveGpsCanvas',true);
let gpsTrack=[],lastGpsKey='',lastGpsStamp=-Infinity,gpsWorldKey='',gpsArchiveId='';
function gpsReadout(prefix,sample){
  const valid=sample?.valid===true;
  text(`${prefix}Position`,valid?`${format(sample.latitude,7)}, ${format(sample.longitude,7)}`:'가상 GPS 대기');
  text(`${prefix}Quality`,valid?'SIM · 가상 위치':'SIM · 수신 대기');
  text(`${prefix}Accuracy`,'시뮬레이션 참값 · 실제 정확도 아님');
  text(`${prefix}Speed`,valid?`${format(Number(sample.speed_mps)*3.6)} km/h`:'— km/h');
  text(`${prefix}Altitude`,valid?`가상 고도 ${format(sample.altitude_msl_m)} m`:'');
  text(`${prefix}MapStatus`,'가상 과수원 · 외부 지도 연결 없음');
}
function paintGps(sample){
  const point=virtualGpsPoint(sample),key=`${sample?.received_at}:${sample?.sequence}`;
  const worldKey=JSON.stringify(sample?.world||{});
  if(point&&(point.stamp<lastGpsStamp||(gpsWorldKey&&gpsWorldKey!==worldKey))){gpsTrack=[];lastGpsKey='';gpsLive.view=null;gpsLive.auto=true;}
  if(point){lastGpsStamp=point.stamp;gpsWorldKey=worldKey;}
  if(point&&key!==lastGpsKey){gpsTrack.push(point);lastGpsKey=key;if(gpsTrack.length>12000)gpsTrack=gpsTrack.filter((_,index)=>index%2===0||index===gpsTrack.length-1);}
  gpsReadout('roverGps',sample);text('roverGpsFixState',point?'SIM GPS':'SIM 대기');
  text('roverGpsAge',point?`${format(sample.age_s||0)}초 전 가상 표본`:'시뮬레이터 GPS 대기');
  gpsLive.set({pose:point,path:gpsTrack,stamp:point?.stamp||0,world:sample?.world});
}
$('roverGpsFollow').textContent='지도 전체';$('roverGpsFollow').addEventListener('click',()=>gpsLive.zoom('fit'));
$('roverArchiveGpsFollow').textContent='지도 전체';$('roverArchiveGpsFollow').addEventListener('click',()=>gpsArchive.zoom('fit'));
// In simulation archive order is media -> grid -> IMU -> GPS.
$('roverArchiveImuPanel').after($('roverArchiveGpsPanel'));
window.RoverGps={routeChanged(){gpsLive.draw();gpsArchive.draw();},profileChanged(){},archive(frames,seconds,id,enabled=false){
  $('roverArchiveGpsPanel').hidden=!enabled&&!frames?.length;if($('roverArchiveGpsPanel').hidden)return;
  if(gpsArchiveId!==id){gpsArchiveId=id;gpsArchive.view=null;gpsArchive.auto=true;}
  const eligible=(frames||[]).filter(frame=>Number(frame.t_s)<=seconds);
  const current=eligible.at(-1), sample=current?.sample||{};
  const path=eligible.map(frame=>{const p=virtualGpsPoint(frame.sample);return p?{...p,stamp:Number(frame.t_s)}:null;}).filter(Boolean);
  gpsReadout('roverArchiveGps',sample);gpsArchive.set({pose:path.at(-1),path,stamp:seconds,world:sample.world});
  text('roverArchiveGpsTime',`가상 GPS · 재생 ${format(seconds,2)} s`);
}};

async function selectArchive(id){
  if(externalNativePC)return;
  selectedId=id;const sequence=++archiveRequest;
  archived.set({});archived.until=0;$('simArchiveMapRegenerate').disabled=!id;
  text('simArchiveMapStatus',id?'저장된 지도 불러오는 중…':'기록을 선택하세요.');
  if(!id)return;
  try{const payload=await request(`/api/sim/recordings/${encodeURIComponent(id)}/map`);if(sequence!==archiveRequest)return;archived.set(payload);text('simArchiveMapStatus',`저장된 최종 지도 · ${payload.scan_count||0} scans · 로버·경로는 재생 시각에 동기화`);}
  catch(error){if(sequence===archiveRequest)text('simArchiveMapStatus',`${error.message} · 지도 재생성으로 다시 만들 수 있습니다.`);}
}
$('simArchiveMapRegenerate').addEventListener('click',async()=>{
  const id=selectedId;if(!id)return;$('simArchiveMapRegenerate').disabled=true;text('simArchiveMapStatus','기록된 스캔으로 지도 재생성 중…');
  try{
    await request(`/api/sim/recordings/${encodeURIComponent(id)}/regenerate`,{});
    const deadline=Date.now()+180000;
    while(selectedId===id){
      const payload=await request(`/api/sim/recordings/${encodeURIComponent(id)}/map`);
      if(selectedId!==id)break;
      if(payload.state!=='regenerating'){if(payload.regeneration_error)throw new Error(payload.regeneration_error);archived.set(payload);text('simArchiveMapStatus',`지도 재생성 완료 · ${payload.scan_count||0} scans`);break;}
      if(Date.now()>deadline)throw new Error('재생성이 계속 진행 중입니다. 잠시 후 기록을 다시 선택하세요.');
      await new Promise(resolve=>setTimeout(resolve,1000));
    }
  }
  catch(error){if(selectedId===id)text('simArchiveMapStatus',`재생성 실패 · ${error.message}`);}
  finally{$('simArchiveMapRegenerate').disabled=!selectedId;}
});

let autoToken=null,autoState={},autoPending=false,autoModeChanges=0,autoError='',autoEpoch=0,heartbeatBusy=false,heartbeatTimer=null,autoRenderKey='';
function renderAutonomy(payload=autoState){
  if(externalNativePC){for(const id of ['simAutonomyStart','simAutonomyStop','simAutonomySpeed'])$(id).disabled=true;return;}
  autoState=payload;const running=payload.active===true,modePending=autoModeChanges>0;
  if(Number.isFinite(payload.speed_percent)&&document.activeElement!==$('simAutonomySpeed'))$('simAutonomySpeed').value=payload.speed_percent;
  text('simAutonomySpeedValue',`${$('simAutonomySpeed').value}%${Number.isFinite(payload.cruise_speed_m_s)?` · ${format(payload.cruise_speed_m_s,2)} m/s`:''}`);
  $('simAutonomySpeed').disabled=speedBusy||autoPending||modePending||(running&&!autoToken);
  // An externally started mission is observed, never adopted: changing the
  // selection does not dispatch an event or acquire its heartbeat token.
  if(running&&!payload.stale&&!modePending)$('simDriveMode').value='autonomy';
  const mode=$('simDriveMode').value==='autonomy';
  $('simAutonomyStart').hidden=!mode;$('simAutonomyStop').hidden=!mode&&!running;
  $('simAutonomyStart').disabled=payload.start_available===false||running||autoPending||modePending||speedBusy||Number($('simAutonomySpeed').value)===0;$('simAutonomyStop').disabled=!running&&!autoPending;
  $('roverArmButton').disabled=running||autoPending||modePending||mode;
  for(const button of document.querySelectorAll('[data-rover-drive]'))if(button.dataset.roverDrive!=='stop')button.disabled=running||modePending||mode;
  text('simAutonomyStatus',`${payload.stale?'상태 수신 대기':running?'자율주행 중':payload.state||'대기'}${payload.message?` · ${payload.message}`:''}${Number.isFinite(payload.progress)?` · ${format(payload.progress*100,0)}%`:''}${autoError?` · ${autoError}`:''}`);
  window.FieldPreparation?.render($('fieldPreparationPanel'),payload.preparation);
  const renderKey=JSON.stringify([running,mode,modePending,payload.state,payload.stale]);
  if(renderKey!==autoRenderKey){autoRenderKey=renderKey;window.renderRoverControl?.();}
}
function clearHeartbeat(){clearInterval(heartbeatTimer);heartbeatTimer=null;autoToken=null;}
async function stopAutonomy(reason='operator_stop',beacon=false){
  autoEpoch++;const token=autoToken;clearHeartbeat();autoPending=false;
  if(!token&&!autoState.active)return;
  const body={token,reason};
  if(beacon&&navigator.sendBeacon){navigator.sendBeacon('/api/sim/autonomy/stop',new Blob([JSON.stringify(body)],{type:'application/json'}));return;}
  try{renderAutonomy(await request('/api/sim/autonomy/stop',body));}catch(error){text('simAutonomyStatus',`정지 요청 · ${error.message}`);}
}
async function heartbeat(){
  if(!autoToken||heartbeatBusy)return;
  if(!active()){stopAutonomy('page_inactive');return;}
  const token=autoToken;heartbeatBusy=true;
  try{const payload=await request('/api/sim/autonomy/heartbeat',{token});if(autoToken===token){renderAutonomy(payload);if(payload.active===false)clearHeartbeat();}}
  catch(error){if(autoToken===token){stopAutonomy('heartbeat_failed');text('simAutonomyStatus',`연결 끊김 · 자율주행 정지 · ${error.message}`);}}
  finally{heartbeatBusy=false;}
}
$('simAutonomyStart').addEventListener('click',async()=>{
  if(autoPending||autoState.active||speedBusy||Number($('simAutonomySpeed').value)===0)return;autoPending=true;autoError='';const epoch=++autoEpoch;renderAutonomy();
  try{
    if(typeof window.stopRoverNow==='function')await window.stopRoverNow('autonomy_start');
    else await request('/api/rover/stop',{reason:'autonomy_start'});
    if(epoch!==autoEpoch)return;
    renderMapping(await request('/api/sim/map/start',{}));
    if(epoch!==autoEpoch)return;
    const payload=await request('/api/sim/autonomy/start',{});
    if(epoch!==autoEpoch||!active()){await request('/api/sim/autonomy/stop',{token:payload.token,reason:'cancelled_start'});return;}
    if(!payload.token)throw new Error('자율주행 제어권 응답이 없습니다');
    autoToken=payload.token;renderAutonomy(payload);heartbeatTimer=setInterval(heartbeat,250);
  }catch(error){autoError=`시작 실패 · ${error.message}`;}
  finally{autoPending=false;renderAutonomy();}
});
$('simAutonomyStop').addEventListener('click',()=>stopAutonomy());
let speedBusy=false;
$('simAutonomySpeed').addEventListener('input',()=>text('simAutonomySpeedValue',`${$('simAutonomySpeed').value}%`));
$('simAutonomySpeed').addEventListener('change',async()=>{
  if(speedBusy)return;speedBusy=true;$('simAutonomySpeed').disabled=true;
  try{
    const payload=await request('/api/sim/autonomy/speed',{speed_percent:Number($('simAutonomySpeed').value),token:autoToken});
    renderAutonomy(payload);if(!payload.active)clearHeartbeat();
  }catch(error){autoError=`속도 변경 실패 · ${error.message}`;}
  finally{speedBusy=false;renderAutonomy();}
});
$('simDriveMode').addEventListener('change',async()=>{
  autoModeChanges++;renderAutonomy();
  try{await stopAutonomy('mode_changed');if(typeof window.stopRoverNow==='function')await window.stopRoverNow('mode_changed');}
  finally{autoModeChanges--;renderAutonomy();}
});
$('roverStopButton').addEventListener('click',()=>stopAutonomy());
for(const button of document.querySelectorAll('[data-rover-drive="stop"]'))button.addEventListener('click',()=>stopAutonomy());
window.addEventListener('keydown',event=>{if(event.key==='Escape')stopAutonomy('escape');});
window.addEventListener('blur',()=>{if(autoToken||autoPending)stopAutonomy('focus_lost');});
window.addEventListener('pagehide',()=>stopAutonomy('pagehide',true));

async function pollGps(generation){
  if(gpsPolling)return;gpsPolling=true;
  try{
    const payload=await request('/api/gps');
    if(active()&&generation===pollGeneration)paintGps(payload);
  }catch{} // Retain the last received GPS view on transport failure.
  finally{gpsPolling=false;}
}
async function pollAutonomy(generation){
  if(autonomyPolling)return;autonomyPolling=true;
  const pollEpoch=autoEpoch,modeWasPending=autoModeChanges>0;
  const current=()=>active()&&generation===pollGeneration&&pollEpoch===autoEpoch&&!modeWasPending&&!autoModeChanges;
  try{
    const payload=await request('/api/sim/autonomy');
    if(current()){
      renderAutonomy(payload);
      if(autoToken&&payload.active===false&&!autoPending)clearHeartbeat();
    }
  }catch{if(current())renderAutonomy({...autoState,stale:true});}
  finally{autonomyPolling=false;}
}
async function poll(){
  clearTimeout(pollTimer);if(!active()||polling)return;polling=true;
  const generation=pollGeneration;
  const query=new URLSearchParams({transport:'tiles'});
  if(live.tileEpoch){query.set('tile_epoch',live.tileEpoch);query.set('tile_since',live.snapshot.tile_version || 0);}
  const statusDue=performance.now()-lastStatusPoll>=750;
  try{
    const pendingMap=externalNativePC?null:request('/api/sim/map?'+query);
    if(statusDue){lastStatusPoll=performance.now();pollGps(generation);if(!externalNativePC)pollAutonomy(generation);}
    if(!pendingMap)return;
    const payload=await pendingMap;
    if(active()&&generation===pollGeneration)renderMapping(payload);
  }catch(error){
    if(active()&&generation===pollGeneration)text('simMapStatus',`지도 연결 대기 · 마지막 지도 유지 · ${error.message}`);
  }finally{
    polling=false;if(active())pollTimer=setTimeout(poll,100);
  }
}
function routeChanged(){
  externalNativeView?.routeChanged();
  globalThis.SimNativeOwner?.routeChanged();
  nativeViewController?.routeChanged();
  ++pollGeneration;
  clearTimeout(pollTimer);if(active()){if(!externalNativePC){live.draw();archived.draw();historical?.draw();}gpsLive.draw();poll();}
  else if(autoToken||autoPending)stopAutonomy('page_inactive');
}
window.SimMap={routeChanged,selectArchive,autonomyStatus:()=>({...autoState,selected:$('simDriveMode').value==='autonomy'}),autonomySelected:()=>autoState.active===true||autoPending||autoModeChanges>0||$('simDriveMode').value==='autonomy',archiveTime(seconds){archived.until=seconds;archived.draw();},stopAutonomy};
document.addEventListener('visibilitychange',routeChanged);
new MutationObserver(()=>{if(!externalNativePC){live.draw();archived.draw();historical?.draw();}gpsLive.draw();gpsArchive.draw();}).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
renderAutonomy();routeChanged();
