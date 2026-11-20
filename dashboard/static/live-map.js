import {MetricMap} from './metric-map.mjs';
import {ArchiveMapBuffer} from './archive-map-buffer.mjs';
import {ArchiveSegmentGallery} from './archive-segments.mjs';
import {decodeArchiveWindow} from './archive-window-codec.mjs';
import {renderJobs} from './mapping-jobs.mjs';
import {LIVE_MAP_TIMEOUT_MS,livePollDelay,livePathQuery,acceptLiveResponse} from './live-map-delta.mjs';
const $=id=>document.getElementById(id),text=(id,value)=>{if($(id))$(id).textContent=value;};
// Correct the real-rover front marker as requested, without modifying any
// retained scan coordinates, localization yaw or simulation body-frame heading.
const live=new MetricMap('liveMapCanvas',false,Math.PI),archive=new MetricMap('liveArchiveCanvas',false,Math.PI);
// Rover Control holds the live and the recording map again (UI-16). The live map is asked for only while its box
// is on screen (without IntersectionObserver: while the page is open); the recording map follows its player.
const routeOn=()=>location.pathname.replace(/\/$/,'')==='/rover-control'&&!document.hidden;
let liveMapOnScreen=null;
const active=()=>routeOn()&&(liveMapOnScreen===true||(liveMapOnScreen===null&&typeof IntersectionObserver!=='function'));
const archiveActive=routeOn;
let polling=false,timer=null,selected='',archiveEpoch=0,speedBusy=false,lastAutonomyPoll=0;
let mapBuffer=null,nextMapBuffer=null,mapWindowPending=null,bufferPaintPending=false,windowSupported=null;
let archiveTimer=null,archivePending=null,archiveTimeS=0,archivePlaying=false,followGeneration=false,archiveGenerating=false;
let archiveRetryMs=75;
let cursorMapping=false,playbackPending=null,lastPlaybackPush=-Infinity;
let triedCausalMap=false,nextExplicitResume=false,lastJobsPoll=0,liveActionBusy=false,archiveMappingGeneration=null;
let liveMapEpoch=null,archiveMapEpoch=null,liveActionError='';
let archiveWaitReason='';
// 2026-11-21: a refused map control does not pause the recording any more (see pushPlayback). The reason stays in the
// map's status line while the video and the sensors keep playing, and the request is tried again after a pause.
const MAP_CONTROL_REFUSED={
  another_device_controls_this_recording:'다른 기기가 이 기록의 지도를 만들고 있습니다. 영상과 센서는 그대로 재생됩니다.',
  another_recording_mapping_active_stop_it_first:'다른 기록의 지도를 만드는 중입니다. 그 작업이 끝나면 이 기록의 지도를 이어서 그립니다.',
  mapping_stopped_press_play_to_resume:'이 기록의 지도 만들기가 멈춰 있습니다. 재생을 다시 누르면 이어서 그립니다.',
  mapping_control_revoked_refresh:'지도 만들기가 다른 곳에서 바뀌었습니다. 재생을 다시 누르면 이어서 그립니다.'};
const MAP_CONTROL_RETRY_MS=3000,MAP_CONTROL_ERROR_RETRY_MS=1000;
let archiveControlNote='',playbackRetryAt=-Infinity;
// S17: a poll that departed before (or during) a New map/Start/Stop action
// must not bring back the previous map or epoch.
let liveActionSeq=0,liveRttMs=0,autonomyPending=false;
let inlinePlaybackMap=false,immutableCursorMap=false,bufferedCursorMap=false,availableCursorPrefix=0;
const playbackClient=globalThis.crypto?.randomUUID?.()||`map-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;let playbackSequence=0;
const segments=new ArchiveSegmentGallery(archive,time=>window.setArchiveTime?.(time,{pause:true}));
archive.beforeCommit=(previous,next)=>segments.beforeCommit(previous,next);
archive.afterDraw=()=>{
  if(archive.snapshot.playback_driven){
    if($('liveArchiveSegments'))$('liveArchiveSegments').hidden=true;
    text('liveArchiveMapNote','새 지도 만들기를 누른 뒤 재생하세요. 일시정지해도 지도는 남고 원본 기록은 그대로 보존됩니다.');
  }else{
    segments.update(archive.snapshot,archiveTimeS);
    text('liveArchiveMapNote','이전 방식으로 만든 기록 지도입니다. 새 지도 만들기를 누른 뒤 재생하면 같은 지도에 계속 쌓입니다.');
  }
  mapControls('archive',archive);
};
live.afterDraw=()=>mapControls('live',live);
function mapControls(kind,map){
  for(const button of document.querySelectorAll(`[data-${kind}-zoom]`)){
    const mode=button.dataset[kind+'Zoom'];
    if(mode==='fit'||mode==='full'){
      const active=map.auto && map.fitMode===(mode==='fit'?'nearby':'full');
      button.classList.toggle('active',active);button.setAttribute('aria-pressed',String(active));
    }
  }
}
function archiveQuality(value){
  if(value.needs_regeneration)return ' · 이전 지도 · 재생성 필요';
  if(value.tracking==='relocalizing')return ' · 위치 다시 찾는 중 · 이전 지도 유지';
  return Number(value.segments)>1?` · 구간 ${(value.segment??0)+1} · 이전 지도 보존 · 구간 연결 미확정`:'';
}
// S22: AbortSignal.any / AbortSignal.timeout are missing in older browsers.
function requestScope(signal,timeout){
  const controller=new AbortController(),abort=()=>controller.abort();
  let expired=false;
  const timer=setTimeout(()=>{expired=true;abort();},timeout);
  if(signal){if(signal.aborted)abort();else signal.addEventListener('abort',abort,{once:true});}
  return {signal:controller.signal,expired:()=>expired,dispose(){clearTimeout(timer);signal?.removeEventListener('abort',abort);}};
}
async function request(path,body,signal,timeoutMs){
  const timeout=timeoutMs??(path.includes('/map-window')||path.includes('/map?')||path.endsWith('/map-playback')?30000:8000);
  const scope=requestScope(signal,timeout);
  try{
  const response=await fetch(path,{cache:'no-store',priority:path.includes('/map')?'high':'auto',signal:scope.signal,...(body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})});
  // S41: a proxy/tunnel HTML error page (502/524) is reported as 'HTTP <status>'.
  const type=response.headers?.get?.('content-type')||'';
  let value;
  try{
    if(type&&!type.toLowerCase().includes('json'))throw new Error('not_json');
    value=await response.json();
  }catch(cause){
    if(cause?.name==='AbortError')throw cause;
    if(scope.signal.aborted){const aborted=new Error('aborted');aborted.name='AbortError';throw aborted;}
    const error=new Error(`HTTP ${response.status}`);error.code=`HTTP ${response.status}`;error.status=response.status;throw error;
  }
  if(!response.ok||value.ok===false){
    const code=value.error||value.message||`HTTP ${response.status}`;
    const messages={another_device_controls_this_recording:'다른 기기가 이 기록의 매핑을 제어하고 있습니다.',another_recording_mapping_active_stop_it_first:'다른 기록을 매핑 중입니다. 작업 관리에서 중단한 뒤 재생하세요.',mapping_stopped_press_play_to_resume:'매핑이 중단되었습니다. 재생을 눌러 이어가세요.',mapping_epoch_changed_refresh_before_action:'다른 기기에서 지도가 변경되었습니다. 지도를 확인한 뒤 다시 시도하세요.',mapping_worker_stopping_retry:'이전 매핑 작업을 정리 중입니다. 잠시 후 다시 시도하세요.',mapping_read_capacity_busy:'지도 조회가 많습니다. 잠시 후 다시 시도하세요.'};
    messages.mapping_control_revoked_refresh='매핑 제어권이 변경되었습니다. 지도를 확인한 뒤 재생을 다시 누르세요.';
    const error=new Error(messages[code]||code);error.code=code;error.status=response.status;
    error.deferred=value.deferred===true||code==='mapping_read_capacity_busy';
    error.retryAfterMs=Math.max(500,Math.min(15000,1000*(Number(value.retry_after_s)||Number(response.headers?.get?.('Retry-After'))||1)));
    throw error;
  }
  return value;
  }catch(error){
    // Caller cancellation is intentionally silent. A deadline must remain a
    // retryable TimeoutError, including on browsers without AbortSignal.any.
    if(scope.expired()){const timeoutError=new Error('요청 시간 초과');timeoutError.name='TimeoutError';throw timeoutError;}
    throw error;
  }finally{scope.dispose();}
}
// S14/E2: deferred reads are shown as a wait with its reason, not as an error.
function deferredArchiveText(error){
  archiveWaitReason=error.code==='autonomy_priority'?'자율주행 우선 · 기록 지도 계산 대기':'지도 조회 대기 중';
  return `${archiveWaitReason} · 마지막 지도 유지`;
}
function updateArchiveWaitReason(value){
  const stage=value?.mapping_job?.stage||value?.generation?.stage;
  archiveWaitReason=value?.waiting_autonomy_priority||stage==='waiting_autonomy_priority'?
    '자율주행 우선 · 기록 지도 계산 대기':value?.waiting_live_priority||stage==='waiting_live_priority'?
    '실시간 지도 우선 · 기록 지도 계산 대기':'';
}
function paint(value){
  liveMapEpoch=value.map_epoch;
  const waiting=value.state==='waiting_for_sensors';
  const restored=value.persistence?.restored&&value.state==='needs_reset';
  live.set(value);text('liveMapState',restored?'저장된 지도':waiting?'센서 동기화 대기':value.active?'그리는 중':value.state==='needs_reset'?'위치 추정 중단':'멈춤');
  $('liveMapStart').disabled=value.active||value.state==='needs_reset';$('liveMapStop').disabled=!value.active;
  text('liveMapStatus',restored?'저장된 지도를 유지하고 있습니다. 새 지도를 눌러 그리기를 시작하세요.':`${waiting?'GPS·IMU·LiDAR 동기화 대기 · 마지막 지도 유지':value.active?'지도 누적 중':value.state==='needs_reset'?'새 지도 필요':'마지막 지도 유지'}${value.reason?` · ${value.reason}`:''}${value.persistence?.saved===false?' · 원본 셀 저장 실패 · 표시 지도 유지':''}`);
}
for(const operation of ['start','stop','reset'])$('liveMap'+operation[0].toUpperCase()+operation.slice(1)).addEventListener('click',async()=>{
  if(liveActionBusy)return;liveActionBusy=true;liveActionSeq++;
  try{const value=await request('/api/live/map/'+operation,{expected_epoch:liveMapEpoch});liveActionError='';paint(value);}
  catch(error){liveActionError=error.message;text('liveMapStatus',error.message);}
  finally{liveActionBusy=false;liveActionSeq++;}
});
for(const button of document.querySelectorAll('[data-live-zoom]'))button.addEventListener('click',()=>live.zoom(button.dataset.liveZoom));
for(const button of document.querySelectorAll('[data-archive-zoom]'))button.addEventListener('click',()=>archive.zoom(button.dataset.archiveZoom));
async function selectArchive(id){
  // S18: re-selecting the same recording keeps its epoch and generation
  // (clearing them made the next New map a 409).
  if(id===selected)return;
  archiveWaitReason='';
  archiveMappingGeneration=null;
  archiveMapEpoch=null;
  selected=id;const epoch=++archiveEpoch;
  clearTimeout(archiveTimer);archiveTimer=null;archivePending?.controller.abort();
  mapBuffer=null;nextMapBuffer=null;windowSupported=null;mapWindowPending?.controller.abort();mapWindowPending=null;
  archiveTimeS=0;archivePlaying=false;followGeneration=false;archiveGenerating=false;cursorMapping=false;lastPlaybackPush=-Infinity;
  archiveControlNote='';playbackRetryAt=-Infinity;
  triedCausalMap=false;inlinePlaybackMap=false;immutableCursorMap=false;bufferedCursorMap=false;availableCursorPrefix=0;
  playbackPending?.controller.abort();playbackPending=null;segments.reset();archive.set({});$('liveArchiveRegenerate').disabled=!id;
  if(!id){text('liveArchiveStatus','기록을 선택하세요.');return;}
  try{await pollArchive(id,epoch);}catch(error){if(epoch===archiveEpoch&&error.name!=='AbortError'){
    if(error.deferred){text('liveArchiveStatus',deferredArchiveText(error));scheduleArchive(error.retryAfterMs||1000);}
    else{text('liveArchiveStatus',error.message);scheduleArchive(500);}
  }}
}
async function pollArchive(id,epoch){
  if(epoch!==archiveEpoch||archivePending?.epoch===epoch)return;
  if(cursorMapping && !bufferedCursorMap && inlinePlaybackMap && archivePlaying){await pushPlayback();return;}
  if((!cursorMapping||bufferedCursorMap) && !followGeneration && windowSupported!==false){
    if(!mapBuffer?.contains(archiveTimeS) && nextMapBuffer?.contains(archiveTimeS)){mapBuffer=nextMapBuffer;nextMapBuffer=null;}
    if(mapBuffer?.contains(archiveTimeS)){
      if(!bufferPaintPending){
        const value=mapBuffer.at(archiveTimeS,archive.snapshot.tile_version,archive.tileEpoch);
        if(value && value.version!==archive.snapshot.version){
          bufferPaintPending=true;
          try{await archive.set(value);}finally{bufferPaintPending=false;}
        }
        text('liveArchiveStatus',`GPS·IMU·LiDAR 통합 지도 · ${(archive.snapshot.frame_t_s||0).toFixed(1)}초 · 센서 시각 동기 재생${archiveQuality(archive.snapshot)}`);
      }
      if(archivePlaying && !nextMapBuffer && !mapWindowPending && mapBuffer.end_s<mapBuffer.duration_s && mapBuffer.end_s-archiveTimeS<10){
        // Overlap the boundary so the last displayed scan is also present in
        // the new window. A late render tick then stays incremental instead
        // of repeatedly installing its full-history seed.
        loadMapWindow(id,epoch,Math.max(mapBuffer.start_s,mapBuffer.end_s-2),12,true).catch(()=>{});
      }
      if(archivePlaying)scheduleArchive();return;
    }
    // A small window can arrive after the clock has already passed its end.
    // Repeated current-frame GETs then compete with that same download forever.
    // Keep the last map while one sufficiently long window catches up.
    if(archivePlaying && mapWindowPending){scheduleArchive(150);return;}
    // A seek gets one complete map first. Bulk look-ahead must not block that
    // response or occupy the connection needed by the current playback cursor.
  }
  const pending={epoch,controller:new AbortController()},target=followGeneration?null:archiveTimeS;
  archivePending=pending;const query=new URLSearchParams({transport:'tiles',codec:'xor2-deflate-v1'});
  if(archive.tileEpoch){query.set('tile_epoch',archive.tileEpoch);query.set('tile_since',archive.snapshot.tile_version || 0);}
  if(target!==null)query.set('t_s',(Math.floor(target*1000)/1000).toFixed(3));
  if(archive.imageVersion!==undefined)query.set('since',`${archive.snapshot.map_epoch}:${archive.imageVersion}`);
  let value;
  try{
    value=await request(`/api/live/recordings/${encodeURIComponent(id)}/map?${query}`,undefined,pending.controller.signal);
    if(value.map_frame)value=(await decodeArchiveWindow(value)).seed;
    // 2026-11-21: read the control generation after unwrapping. A recording that already has map cells answers with
    // a wrapped reply whose generation sits in the seed; read from the wrapper it was never learned, so a recording
    // whose map work the server had ended (15 s after a pause or after its player left) refused every later play
    // with mapping_control_revoked_refresh until the dashboard was restarted. Only for the recording still selected.
    if(epoch===archiveEpoch&&value.mapping_generation)archiveMappingGeneration=value.mapping_generation;
    if(epoch!==archiveEpoch||(!followGeneration&&(target===null||target>archiveTimeS+.001)))return;
    if(inlinePlaybackMap&&!bufferedCursorMap&&archivePlaying)return;
    await acceptArchiveMap(value);
    archiveRetryMs=75;
    if(archivePlaying && (!cursorMapping||bufferedCursorMap) && !followGeneration && windowSupported!==false && !mapWindowPending){
      loadMapWindow(id,epoch,Math.max(0,archiveTimeS),12,false).then(()=>scheduleArchive()).catch(error=>{
        if(epoch===archiveEpoch&&error.name!=='AbortError')scheduleArchive(500);
      });
    }
  }finally{
    if(archivePending===pending)archivePending=null;
    // A paused seek can occur while the previous request is in flight. Its
    // timer may have found that request busy; always service the newest cursor
    // after completion instead of leaving the canvas empty until another seek.
    const cursorChanged=!followGeneration&&(target===null||Math.abs(target-archiveTimeS)>.05);
    const legacyGenerating=!cursorMapping&&value?.state==='regenerating';
    const catchingUp=cursorMapping&&archivePlaying&&!value?.mapping_error&&value?.generation?.waiting_for_cursor!==true;
    if(epoch===archiveEpoch&&(!inlinePlaybackMap||bufferedCursorMap)&&(legacyGenerating||archivePlaying||cursorChanged||catchingUp))scheduleArchive();
  }
}
async function acceptArchiveMap(value){
    archiveMapEpoch=value.map_epoch;
    if(archivePlaying && value.tile_epoch===archive.tileEpoch &&
       Number(value.tile_version)<Number(archive.snapshot.tile_version))return;
    updateArchiveWaitReason(value);
    cursorMapping=value.playback_driven===true;
    immutableCursorMap=cursorMapping && value.state==='saved' && value.generation?.stage==='complete' && !value.needs_regeneration;
    availableCursorPrefix=cursorMapping?Number(value.generation?.processed_t_s)||0:0;
    // Previously committed events remain immutable when seeking back into an
    // unfinished causal history. Read windows never advance the localizer.
    bufferedCursorMap=cursorMapping && !value.needs_regeneration &&
      (immutableCursorMap || availableCursorPrefix>=archiveTimeS+3);
    if(value.grid||value.scan_count===0)await archive.set(value);
    const generation=value.generation||{},duration=Number(generation.duration_s)||0;
    // A causal worker can be interrupted by New map. Only the legacy bulk
    // regeneration requires waiting; it must not lock this new workflow.
    archiveGenerating=!cursorMapping&&value.state==='regenerating';$('liveArchiveRegenerate').disabled=archiveGenerating;
    text('liveArchiveStatus',archiveControlNote?archiveControlNote:archiveWaitReason?`${archiveWaitReason} · 마지막 지도 유지`:value.mapping_error?`지도 처리 오류 · ${value.mapping_error} · 이전 지도 유지`:value.regeneration_error?`재생성 실패 · ${value.regeneration_error}`:
      cursorMapping?`${value.grid?(archivePlaying?'재생하며 지도 저장 중':'저장된 지도 유지'):'재생하면 새 지도가 쌓입니다'} · ${(value.frame_t_s||0).toFixed(1)}초${archiveQuality(value)}`:
      value.state==='regenerating'?(generation.stage==='localizing'?'LiDAR·IMU 위치 추정 계산 중…':`지도 생성 ${Math.round((generation.progress||0)*100)}% · ${(generation.processed_t_s||0).toFixed(1)} / ${duration.toFixed(1)}초`):
      value.needs_regeneration?'이전 방식의 지도입니다. 새 지도 만들기 후 재생하세요.':
      value.state==='interrupted'?'지도 생성이 중단됐습니다. 만든 부분은 남아 있고 다시 만들 수 있습니다.':
      value.grid?`${followGeneration?'생성된 전체 지도':'재생 시각의 지도'} · ${(value.frame_t_s||0).toFixed(1)}초${archiveQuality(value)}`:
      value.history_available?'아직 지도 관측이 없습니다. 재생하면 지도가 쌓입니다.':'저장된 지도가 없습니다. 새 지도 만들기를 누른 뒤 재생하세요.');
}
async function loadMapWindow(id,epoch,start,span,next){
  if(mapWindowPending)return;
  const pending={controller:new AbortController(),epoch,start,end:start+span,next};mapWindowPending=pending;
  try{
    const raw=await request(`/api/live/recordings/${encodeURIComponent(id)}/map-window?start_s=${start.toFixed(3)}&span_s=${span}&codec=xor2-deflate-v1`,undefined,pending.controller.signal);
    if(raw.mapping_generation)archiveMappingGeneration=raw.mapping_generation;
    const value=await decodeArchiveWindow(raw);
    if(epoch!==archiveEpoch)return;
    windowSupported=value.available===true;
    if(windowSupported){const window=new ArchiveMapBuffer(value);if(next)nextMapBuffer=window;else mapBuffer=window;}
  }finally{if(mapWindowPending===pending)mapWindowPending=null;}
}
function scheduleArchive(delay=75){
  if(archiveTimer!==null||!selected||!archiveActive())return;
  const id=selected,epoch=archiveEpoch;
  archiveTimer=setTimeout(()=>{archiveTimer=null;pollArchive(id,epoch).catch(error=>{
    if(epoch===archiveEpoch&&error.name!=='AbortError'){
      if(error.deferred){
        // S14/E2: a busy read slot or autonomy priority is a wait, not a failure.
        text('liveArchiveStatus',deferredArchiveText(error));
        archiveRetryMs=Math.min(15000,Math.max(error.retryAfterMs||500,archiveRetryMs*2));scheduleArchive(archiveRetryMs);
        return;
      }
      text('liveArchiveStatus',`${error.message} · 마지막 지도 유지 · 재시도 중`);
      archiveRetryMs=Math.min(2000,Math.max(500,archiveRetryMs*2));scheduleArchive(archiveRetryMs);
    }
  });},delay);
}
function archiveTime(id,timeS,playing,seekRequested=false){
  if(id!==selected)return;
  if(playing&&!archivePlaying)nextExplicitResume=true;
  // Video quality changes/buffering can report a slightly older media clock.
  // Only an explicit user seek or a paused change may rewind a running map.
  // Keep both the requested raw prefix and rendered cells monotonic otherwise.
  timeS=Math.max(0,Number(timeS)||0);
  if(playing && archivePlaying && !seekRequested)timeS=Math.max(timeS,archiveTimeS);
  if(playing||Math.abs(timeS-archiveTimeS)>.05)followGeneration=false;
  if(!followGeneration&&(seekRequested||!playing)&&Math.max(Number(archive.snapshot.frame_t_s)||0,Number(archive.pendingImage?.snapshot.frame_t_s)||0)>timeS){
    // Cancel a future PNG decoder immediately on backward seek/play-from-zero.
    // The same epoch keeps the user's metric zoom/pan while an earlier frame loads.
    archive.set({map_epoch:archive.snapshot.map_epoch,preserve_view_on_seek:true,scan_count:0,version:0});
  }
  const jump=Number(timeS)-archiveTimeS;
  // Media clocks can jump after buffering or a busy render frame. Only an
  // explicit seek (or backward/paused change) invalidates a future download.
  const seek=seekRequested || jump<-.05 || (!playing && Math.abs(jump)>.05);
  const playChanged=Boolean(playing)!==archivePlaying;
  const changed=Math.abs(jump)>.05||playChanged;
  archiveTimeS=Math.max(0,Number(timeS)||0);archivePlaying=Boolean(playing);
  if(cursorMapping)bufferedCursorMap=!archive.snapshot.needs_regeneration&&
    (immutableCursorMap||availableCursorPrefix>=archiveTimeS+3);
  if(playChanged&&!archivePlaying){
    // The note about a refused map control belongs to the playback that was refused.
    archiveControlNote='';playbackRetryAt=-Infinity;
    clearTimeout(archiveTimer);archiveTimer=null;
    // Supersede an old Play immediately. Server sequence checks prevent a
    // delayed request from reviving processing after this newer Pause.
    playbackPending?.controller.abort();playbackPending=null;
    mapWindowPending?.controller.abort();mapWindowPending=null;
  }
  if((cursorMapping || (!triedCausalMap && archivePlaying)) && (changed || archivePlaying))pushPlayback(playChanged||seekRequested);
  // A future window is intentionally ahead of the clock. Normal playback must
  // not abort it simply because the current cursor precedes its start.
  if(mapWindowPending && seek && (archiveTimeS<mapWindowPending.start || archiveTimeS>mapWindowPending.end)){
    mapWindowPending.controller.abort();mapWindowPending=null;
  }
  if(changed && !archivePlaying && archivePending){archivePending.controller.abort();archivePending=null;}
  if(changed&&(!inlinePlaybackMap||bufferedCursorMap))scheduleArchive();
}
function archiveReady(id,timeS){
  if(id!==selected||!cursorMapping||archive.snapshot.needs_regeneration||archive.snapshot.mapping_error)return true;
  const time=Math.max(0,Number(timeS)||0);
  if(bufferedCursorMap && windowSupported!==false){
    return mapBuffer?.contains(time)===true||nextMapBuffer?.contains(time)===true||
      time<=Number(archive.snapshot.frame_t_s||0)+.35;
  }
  // Rejected geometry still advances the processed source prefix. It must not
  // deadlock playback by demanding an invented new map pose in a sensor gap.
  return time<=Math.max(availableCursorPrefix,Number(archive.snapshot.frame_t_s)||0)+.35;
}
// Seconds the recorded map is behind the playback time; 0 while it keeps up or when there is no map to wait for.
// Only for the note in the player's time label (2026-11-21): the playback itself does not wait for the map.
function archiveLag(id,timeS){
  if(archiveReady(id,timeS))return 0;
  const cursor=bufferedCursorMap&&windowSupported!==false?0:availableCursorPrefix;
  return Math.max(0,Math.max(0,Number(timeS)||0)-Math.max(cursor,Number(archive.snapshot.frame_t_s)||0));
}
async function pushPlayback(force=false){
  // Complete causal histories are immutable. Timed cell windows can render
  // every retained scan locally; there is no remaining localization to drive.
  if(immutableCursorMap)return;
  if((!cursorMapping && (triedCausalMap||!archivePlaying))||!selected||playbackPending||(!force&&performance.now()-lastPlaybackPush<250))return;
  // After a refusal or an error the next try waits; an explicit play, pause or seek (force) goes out at once.
  if(!force&&performance.now()<playbackRetryAt)return;
  const id=selected,epoch=archiveEpoch,time=archiveTimeS,playing=archivePlaying;
  const pending={controller:new AbortController(),epoch};playbackPending=pending;lastPlaybackPush=performance.now();
  let failed=false;
  try{
    const value=await request(`/api/live/recordings/${encodeURIComponent(id)}/map-playback`,{
      t_s:time,playing,control:{client:playbackClient,sequence:++playbackSequence},resume_mapping:nextExplicitResume,
      // Cached cells render locally while light ordered control still freezes
      // the unfinished worker. Live generation uses inline cells.
      mapping_generation:archiveMappingGeneration,
      ...(bufferedCursorMap?{}:{transport:'tiles',codec:'xor2-deflate-v1',tile_epoch:archive.tileEpoch,tile_since:archive.snapshot.tile_version||0})
    },pending.controller.signal);
    if(epoch===archiveEpoch){
      // The reason of an earlier refusal must not stay on screen once the control is accepted.
      if(archiveControlNote&&playing)text('liveArchiveStatus','이 기록의 지도를 이어서 그립니다.');
      nextExplicitResume=false;archiveControlNote='';playbackRetryAt=-Infinity;
      triedCausalMap=true;
      // S15: remember why the recorded map is waiting, for the playback label.
      updateArchiveWaitReason(value);
      if(value.mapping_generation)archiveMappingGeneration=value.mapping_generation;
      if(value.map_snapshot && !value.stale_control_ignored && playbackPending===pending){
        let snapshot=value.map_snapshot;
        if(snapshot.map_frame)snapshot=(await decodeArchiveWindow(snapshot)).seed;
        // An explicit rewind/selection may supersede the decoder as well as
        // fetch. Never apply cells newer than the currently visible cursor.
        if(epoch===archiveEpoch && playbackPending===pending && Number(snapshot.frame_t_s||0)<=archiveTimeS+.001){
          inlinePlaybackMap=true;
          clearTimeout(archiveTimer);archiveTimer=null;
          archivePending?.controller.abort();archivePending=null;
          mapWindowPending?.controller.abort();mapWindowPending=null;
          mapBuffer=null;nextMapBuffer=null;
          await acceptArchiveMap(snapshot);
        }
      }
      if(value.playback_driven && !cursorMapping){
        cursorMapping=true;mapBuffer=null;nextMapBuffer=null;
        mapWindowPending?.controller.abort();mapWindowPending=null;scheduleArchive();
      }
      // A retained inline snapshot may carry older job metadata. The current
      // control reply is authoritative about a live/autonomy priority wait.
      updateArchiveWaitReason(value);
      if(archiveWaitReason)text('liveArchiveStatus',`${archiveWaitReason} · 마지막 지도 유지`);
    }
  }catch(error){if(epoch===archiveEpoch&&error.name!=='AbortError'){
    failed=true;
    if(Object.hasOwn(MAP_CONTROL_REFUSED,error.code)){
      // 2026-11-21: the recording keeps playing. Until now a refusal paused the player, so a recording could not be
      // played for up to 15 s after another one had been watched (the other recording's map work ends by itself
      // 15 s after its player has gone), and not at all while someone else watched another recording. The explicit
      // resume flag is kept: a play pressed with an outdated generation goes through once the reading below has
      // brought the current one. A map stopped in the job list stays stopped: without that flag the server refuses.
      archiveControlNote=MAP_CONTROL_REFUSED[error.code];playbackRetryAt=performance.now()+MAP_CONTROL_RETRY_MS;
      inlinePlaybackMap=false;
      text('liveArchiveStatus',archiveControlNote);
      scheduleArchive(500);
    }else{
      playbackRetryAt=performance.now()+MAP_CONTROL_ERROR_RETRY_MS;
      text('liveArchiveStatus',`${error.message} · 마지막 지도 유지`);
      if(!archivePlaying)scheduleArchive(500);
    }
  }}
  finally{
    if(playbackPending===pending)playbackPending=null;
    // A failed request is tried again by the next playback tick after its wait, not at once in a tight loop.
    if(!failed&&epoch===archiveEpoch&&(Math.abs(time-archiveTimeS)>.05||playing!==archivePlaying))pushPlayback(true);
  }
}
$('liveArchiveRegenerate').addEventListener('click',async()=>{
  const id=selected;if(!id)return;
  window.setArchiveTime?.(0,{pause:true});
  const epoch=++archiveEpoch;archivePending?.controller.abort();archivePending=null;
  mapWindowPending?.controller.abort();mapWindowPending=null;playbackPending?.controller.abort();playbackPending=null;
  mapBuffer=null;nextMapBuffer=null;windowSupported=null;$('liveArchiveRegenerate').disabled=true;
  try{
    const value=await request(`/api/live/recordings/${encodeURIComponent(id)}/new-map`,{expected_epoch:archiveMapEpoch,client:playbackClient});
    if(epoch!==archiveEpoch)return;
    archiveMappingGeneration=value.mapping_generation;
    archiveMapEpoch=value.map_epoch;
    archiveWaitReason='';
    cursorMapping=true;triedCausalMap=true;inlinePlaybackMap=false;immutableCursorMap=false;bufferedCursorMap=false;availableCursorPrefix=0;
    archiveTimeS=0;archivePlaying=false;archiveGenerating=false;followGeneration=false;
    segments.reset();archive.set({map_epoch:value.map_epoch,preserve_view_on_seek:true,version:0,scan_count:0,tiles:[],tiles_reset:true,playback_driven:true});
    text('liveArchiveStatus','새 지도가 준비됐습니다. 재생을 누르면 지도가 쌓이고 저장됩니다.');
  }
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
async function refreshAutonomyStatus(){
  // Independent of the map poll: a slow /api/autonomy never delays the map.
  if(autonomyPending||Date.now()-lastAutonomyPoll<=3000)return;
  autonomyPending=true;lastAutonomyPoll=Date.now();
  try{
    const value=await request('/api/autonomy');
    if(value&&!speedBusy&&document.activeElement!==$('hardwareAutonomySpeed')){
      $('hardwareAutonomySpeed').value=value.speed_percent;text('hardwareAutonomySpeedValue',`${value.speed_percent}%`);
      text('hardwareAutonomyStatus',`${value.message} · 현재 순항속도 설정 ${value.speed_percent}%`);
    }
    if(value)window.FieldPreparation?.render($('fieldPreparationPanel'),value.preparation);
  }catch(_){}
  finally{autonomyPending=false;}
}
async function poll(){
  const started=performance.now();
  clearTimeout(timer);if(polling||!active())return;polling=true;
  const actionSeq=liveActionSeq;
  try{
    const query=new URLSearchParams({transport:'tiles'});
    if(live.tileEpoch){query.set('tile_epoch',live.tileEpoch);query.set('tile_since',live.snapshot.tile_version || 0);}
    // S02: name the newest path point we hold; the server then sends only later points.
    if(live.tileEpoch)for(const [key,value] of Object.entries(livePathQuery(live.snapshot,live.pathNeedsFull)))query.set(key,value);
    refreshAutonomyStatus();
    if($('mappingJobsPanel')?.open&&Date.now()-lastJobsPoll>1500){lastJobsPoll=Date.now();pollJobs().catch(()=>{});}
    try{
      const value=await request('/api/live/map?'+query,undefined,undefined,LIVE_MAP_TIMEOUT_MS);
      liveRttMs=performance.now()-started;
      if(acceptLiveResponse({requestActionSeq:actionSeq,currentActionSeq:liveActionSeq,actionBusy:liveActionBusy,value,shown:live.snapshot})){
        paint(value);
        if(liveActionError)text('liveMapStatus',liveActionError+' · 마지막 지도 유지');
      }
    }catch(error){
      liveRttMs=Math.max(liveRttMs,performance.now()-started);
      text('liveMapStatus',`지도 연결 대기 · 마지막 지도 유지 · ${error.message}`);
    }
  }finally{
    polling=false;
    if(active()){
      const elapsed=performance.now()-started;
      timer=setTimeout(poll,livePollDelay({profile:window.RoverLink?.profile?.()||'balanced',drawing:Boolean(live.snapshot.active),
        rttMs:liveRttMs,elapsedMs:elapsed,autonomyActive:window.RoverLink?.autonomyPriority?.()==='active'}));
    }
  }
}
async function pollJobs(){
  const value=await request('/api/live/mapping-jobs');
  const resources=value.resources||{},cpu=resources.cpu_percent_one_core;
  text('mappingJobsResources',`실시간 우선 · 기록 계산 최대 1개 · 대시보드 CPU ${Number.isFinite(cpu)?cpu.toFixed(1)+'% (1코어 기준)':'—'} · 메모리 ${Number.isFinite(resources.rss_bytes)?(resources.rss_bytes/1048576).toFixed(0)+'MB':'—'} · 센서 드라이버 사용량은 별도`);
  renderJobs($('mappingJobsList'),value,async job=>{
    if(job.kind==='archive'&&job.recording_id===selected){
      window.setArchiveTime?.(archiveTimeS,{pause:true});archivePlaying=false;
      playbackPending?.controller.abort();nextExplicitResume=false;
    }
    try{
      const stopped=await request('/api/live/mapping-jobs/stop',{job_id:job.id,expected_epoch:job.epoch});
      if(job.kind==='archive'&&job.recording_id===selected)archiveMappingGeneration=stopped.mapping_generation;
      await pollJobs();
      if(job.kind==='live'){
        liveActionSeq++;
        try{paint(await request('/api/live/map?transport=tiles'));}finally{liveActionSeq++;}
      }
    }catch(error){text('mappingJobsResources',error.message);}
  });
}
$('mappingJobsPanel')?.addEventListener('toggle',()=>{if($('mappingJobsPanel').open)pollJobs().catch(error=>text('mappingJobsResources',error.message));});
function routeChanged(){live.draw();archive.draw();if(active())poll();if(archiveActive()&&selected)scheduleArchive();}
window.LiveMap={selectArchive,archiveTime,archiveReady,archiveLag,routeChanged,waitReason:()=>archiveControlNote?'지도는 잠시 멈춰 있습니다':archiveWaitReason};
document.addEventListener('visibilitychange',routeChanged);window.addEventListener('popstate',routeChanged);
// The main application exposes route changes through the GPS hook.
const gps=window.RoverGps;if(gps){const original=gps.routeChanged;gps.routeChanged=(...args)=>{original?.(...args);routeChanged();};}
if(typeof IntersectionObserver==='function'&&$('liveMapPanel'))new IntersectionObserver(entries=>{
  const was=liveMapOnScreen;liveMapOnScreen=Boolean(entries[entries.length-1].isIntersecting);
  if(liveMapOnScreen&&was!==true)routeChanged();
}).observe($('liveMapPanel'));
// The drive panel's autonomy speed and field preparation rows are refreshed with the map poll, or on their own
// while the map is off screen.
setInterval(()=>{if(routeOn())refreshAutonomyStatus();if(active()&&!polling)poll();},3000);
routeChanged();
