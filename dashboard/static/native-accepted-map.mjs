import {decodeTileCells} from './grid-cell-codec.mjs';

const FALSE_FLAGS=['runtime_authority','motor_command_sent','autonomy_output_enabled',
  'execution_eligible','physical_accuracy_verified','live_input_eligible'];
const require=(value,reason)=>{if(!value)throw new Error(reason);};
const integer=value=>Number.isSafeInteger(value)&&value>=0;
const clone=value=>JSON.parse(JSON.stringify(value));
const base='/api/autonomy/native-map/';

export class PoseHistoryNotice {
  constructor() {this.message='';}
  update(value) {
    if(value?.state==='new_map')this.message='';
    const map=value?.status?.map,missing=map?.pose_history_missing_commits;
    const knownCount=Number.isFinite(missing)&&Number.isInteger(missing)&&missing>0;
    if(knownCount||(map?.pose_history_complete===false&&map?.pose_history_warning==='retained_pose_history_capacity')) {
      const count=knownCount&&Number.isSafeInteger(missing)?`${missing.toLocaleString('ko-KR')}회`:'일부';
      this.message=`지도 재구성 이력 ${count} 누락 · 보관 한도 초과 · 전체 이력 재구성이 불완전합니다.`;
    }else if(map?.pose_history_complete===true)this.message='';
    // Display Stop or a missing transport reply carries no new history proof.
    // Retain the notice until a complete ledger or explicit New map is seen.
    return this.message;
  }
}

export class NativeAcceptedMapClient {
  constructor({request,commit,status=()=>{},pollMs=500,action=null,clear=()=>{}}) {
    require(typeof request==='function'&&typeof commit==='function'&&typeof status==='function', 'native_map_callbacks');
    require(Number.isInteger(pollMs)&&pollMs>=100&&pollMs<=5000,'native_map_poll');
    require((action===null||typeof action==='function')&&typeof clear==='function','native_map_explicit_clear');
    this.request=request;this.commit=commit;this.notify=status;this.pollMs=pollMs;
    this.action=action;this.clear=clear;this.resetting=false;
    this.running=false;this.busy=false;this.revision=0;this.last=null;this.controller=null;this.timer=null;this.lease=null;
  }
  start() {if(this.running||this.resetting)return;this.running=true;++this.revision;this.schedule(0);}
  stop() {this.running=false;++this.revision;clearTimeout(this.timer);this.timer=null;
    this.controller?.abort();this.controller=null;this.releaseLease();this.notify({state:'display_stopped',retained:!!this.last});}
  async releaseLease() {
    const token=this.lease;this.lease=null;if(!token)return;
    const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),500);
    try{await this.request(base+'release?lease='+token,abort.signal);}catch{}
    finally{clearTimeout(timeout);}
  }
  schedule(ms) {clearTimeout(this.timer);if(this.running)this.timer=setTimeout(()=>this.poll(),ms);}
  async newMap() {
    if(this.resetting||!this.action)return false;
    this.resetting=true;this.stop();const revision=this.revision;
    const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),2000);
    try {
      const status=await this.request(base+'status',abort.signal),epoch=status?.status?.map_epoch;
      require(status?.ok===true&&FALSE_FLAGS.every(k=>status[k]===false)&&
        typeof epoch==='string'&&/^[0-9a-f]{32}$/.test(epoch),'native_new_map_current_epoch');
      const answer=await this.action(base+'new',{map_epoch:epoch},abort.signal);
      require(answer?.ok===true&&answer.map_cleared===true&&FALSE_FLAGS.every(k=>answer[k]===false)&&
        typeof answer.map_epoch==='string'&&/^[0-9a-f]{32}$/.test(answer.map_epoch)&&answer.map_epoch!==epoch,
        'native_new_map_closed_owner_required');
      require(revision===this.revision&&!abort.signal.aborted,'native_new_map_cancelled');
      this.last=null;await this.clear(answer.map_epoch);this.notify({state:'new_map',retained:false});return true;
    }catch{this.notify({state:'new_map_denied',retained:!!this.last});return false;}
    finally{clearTimeout(timeout);this.resetting=false;}
  }
  async poll() {
    if(!this.running||this.busy)return;
    this.busy=true;const pollStarted=performance.now();
    const revision=this.revision,abort=new AbortController();this.controller=abort;
    // A fixed 120s server pin does not expire during ordinary slow paging.
    // Single-flight and this 60s total bound prevent accumulating generations.
    const timeout=setTimeout(()=>abort.abort(),60000);
    try {
      const first=await this.request(base+'tiles',abort.signal);
      require(first?.ok===true&&FALSE_FLAGS.every(k=>first[k]===false),'native_map_reply_contract');
      // Track a returned pin for release even when this reply was superseded
      // by display Stop/Start. No obsolete reply may update status or history.
      if(first.available&&typeof first.page?.lease_id==='string'&&/^[0-9a-f]{32}$/.test(first.page.lease_id))this.lease=first.page.lease_id;
      require(!abort.signal.aborted&&this.running&&revision===this.revision,'native_map_cancelled');
      if(!first.available){this.notify({state:'waiting',retained:!!this.last,status:first.status});return;}
      const pages=[],keys=new Set();let answer=first,head=null,total=0;
      for(let index=0;index<64;index++) {
        require(!abort.signal.aborted&&this.running&&revision===this.revision,'native_map_cancelled');
        const page=answer.page;
        if(index===0&&typeof page?.lease_id==='string'&&/^[0-9a-f]{32}$/.test(page.lease_id))this.lease=page.lease_id;
        require(answer.ok===true&&answer.available===true&&FALSE_FLAGS.every(k=>answer[k]===false)&&
          page?.schema==='ubts.native.accepted_map.tiles.v1'&&FALSE_FLAGS.every(k=>page[k]===false)&&
          page.storage==='ternary_native_evidence_v1'&&page.retained_historical_generation===true&&
          page.tile_size===64&&page.tiles_reset===true&&integer(page.tile_version)&&page.tile_version>0&&
          typeof page.tile_epoch==='string'&&/^[0-9a-f]{64}$/.test(page.tile_epoch)&&
          typeof page.generation_id==='string'&&/^[0-9a-f]{64}$/.test(page.generation_id)&&
          typeof page.lease_id==='string'&&/^[0-9a-f]{32}$/.test(page.lease_id)&&
          integer(page.page_index)&&page.page_index===index&&integer(page.page_count)&&
          page.page_count>0&&page.page_count<=64&&integer(page.tile_count)&&page.tile_count<=512&&
          Array.isArray(page.tiles)&&page.tiles.length>0&&page.tiles.length<=8,'native_map_page_contract');
        const metadata={...page};delete metadata.tiles;delete metadata.page_index;delete metadata.next_cursor;
        if(!head)head=metadata;else require(JSON.stringify(metadata)===JSON.stringify(head),'native_map_mixed_generation');
        require(page.grid?.encoding==='tiles-cells-v1'&&page.grid.row_order==='min_y_first'&&
          Number.isFinite(page.grid.resolution)&&page.grid.resolution>=.02&&page.grid.resolution<=1&&
          integer(page.grid.width)&&page.grid.width>0&&integer(page.grid.height)&&page.grid.height>0&&
          Number.isFinite(page.grid.origin?.x)&&Number.isFinite(page.grid.origin?.y)&&
          Number.isFinite(page.pose?.x)&&Number.isFinite(page.pose?.y)&&Number.isFinite(page.pose?.yaw)&&
          integer(page.cell_count)&&page.cell_count<=250000&&typeof page.context==='object'&&page.context!==null,
          'native_map_geometry_contract');
        require(Array.isArray(page.path)&&page.path.length<=256&&page.path_vertex_limit===256&&
          page.path_representation==='accepted_render_pose_vertices_uniform_thinning'&&integer(page.path_compressions)&&
          page.path.every(p=>p&&Number.isFinite(p.x)&&Number.isFinite(p.y)&&Number.isFinite(p.yaw)&&
            Number.isFinite(p.stamp)&&typeof p.break_before==='boolean'),'native_map_path_contract');
        for(const tile of page.tiles) {
          const key=`${tile.x},${tile.y}`;require(!keys.has(key),'native_map_duplicate_tile');keys.add(key);
          decodeTileCells(tile,64); // original codec validates coordinates and every cell
        }
        if(index===0&&this.last?.generation_id===page.generation_id) {
          const old={...this.last},current={...metadata};
          delete old.tiles;delete old.lease_id;delete current.lease_id;
          require(JSON.stringify(old)===JSON.stringify(current),'native_map_cached_generation_changed');
          const map=answer.status?.map;
          this.notify({state:map?.active===true&&map?.pose_continuity===true?'observing':'retained',
            retained:true,map_version:head.tile_version,cell_count:head.cell_count,status:answer.status});
          return; // already validated complete generation; finally releases the new pin
        }
        total+=new TextEncoder().encode(JSON.stringify(page)).length;
        require(total<=4*1024*1024,'native_map_generation_capacity');pages.push(...page.tiles);
        const next=index+1===page.page_count?null:`${page.lease_id}:${index+1}`;
        require(page.next_cursor===next,'native_map_cursor_contract');
        if(next===null)break;
        answer=await this.request(base+'tiles?cursor='+encodeURIComponent(next),abort.signal);
      }
      require(pages.length===head.tile_count&&pages.length<=512,'native_map_incomplete_generation');
      if(this.last?.tile_epoch===head.tile_epoch)require(head.tile_version>=this.last.tile_version,'native_map_rewind');
      require(!abort.signal.aborted&&this.running&&revision===this.revision,'native_map_cancelled');
      const snapshot={...head,tiles:pages};
      if(this.last?.generation_id!==head.generation_id){
        await this.commit(clone(snapshot));
        require(!abort.signal.aborted&&this.running&&revision===this.revision,'native_map_cancelled');
        this.last=clone(snapshot);
      }
      const map=answer.status?.map;
      this.notify({state:map?.active===true&&map?.pose_continuity===true?'observing':'retained',
        retained:true,map_version:head.tile_version,cell_count:head.cell_count,status:answer.status});
    } catch(error) {if(revision===this.revision&&this.running)this.notify({state:'unavailable',retained:!!this.last,reason:'native_map_transport_unavailable'});}
    finally {clearTimeout(timeout);if(this.controller===abort)this.controller=null;await this.releaseLease();this.busy=false;
      // A Stop/Start during an old in-flight request schedules only this new owner.
      if(this.running)this.schedule(Math.max(0,this.pollMs-(performance.now()-pollStarted)));}
  }
}

export async function boundedMapRequest(path,signal) {
  const response=await fetch(path,{method:'GET',credentials:'same-origin',redirect:'error',cache:'no-store',signal});
  return boundedJsonResponse(response);
}

async function boundedMapAction(path,payload,signal) {
  const response=await fetch(path,{method:'POST',credentials:'same-origin',redirect:'error',cache:'no-store',signal,
    headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)});
  return boundedJsonResponse(response);
}

async function boundedJsonResponse(response) {
  require(response.ok&&response.headers.get('Content-Type')?.includes('application/json'),'native_map_http');
  const length=response.headers.get('Content-Length');if(length!==null)require(/^\d+$/.test(length)&&Number(length)<=131072,'native_map_http_capacity');
  const reader=response.body.getReader(),parts=[];let size=0;
  try {while(true){const {done,value}=await reader.read();if(done)break;
    size+=value.byteLength;require(size<=131072,'native_map_http_capacity');parts.push(value);}}
  finally {try{await reader.cancel();}catch{}reader.releaseLock();}
  const raw=new Uint8Array(size);let offset=0;for(const part of parts){raw.set(part,offset);offset+=part.length;}
  return JSON.parse(new TextDecoder('utf-8',{fatal:true}).decode(raw));
}

export async function mountNativeAcceptedMap(container) {
  if(!container||container._nativeMapView||container._nativeMapMounting)return container?._nativeMapView;
  container._nativeMapMounting=true;
  try {
    const {MetricMap}=await import('./metric-map.mjs');
    const document=container.ownerDocument,section=document.createElement('section');
    const title=document.createElement('h3');title.textContent='센서 누적 지도 · 관찰용';section.append(title);
    const note=document.createElement('p');note.textContent='기존 지도와 별도로 표시 · 주행 제어에 사용하지 않음';section.append(note);
    const start=document.createElement('button'),stop=document.createElement('button'),fresh=document.createElement('button');
    start.type=stop.type=fresh.type='button';start.textContent='지도 표시';stop.textContent='표시 멈춤';fresh.textContent='새 지도';
    // Same button style and spacing as the observation controls above (they had the browser default look).
    for(const button of [start,stop,fresh])button.className='action-button secondary';
    const actions=document.createElement('div');actions.className='field-observation-actions';actions.append(start,stop,fresh);
    fresh.title='생산자 관찰을 먼저 멈춘 뒤에만 기존 지도를 지웁니다';section.append(actions);
    const canvas=document.createElement('canvas');canvas.id='nativeAcceptedMapCanvas';
    canvas.style.cssText='width:100%;height:320px;touch-action:none';section.append(canvas);
    const status=document.createElement('p');status.setAttribute('role','status');status.textContent='표시 OFF · 기존 지도 유지';
    section.append(status);container.append(section);
    const historyWarning=document.createElement('p'),historyNotice=new PoseHistoryNotice();
    historyWarning.setAttribute('role','status');historyWarning.hidden=true;section.append(historyWarning);
    const map=new MetricMap(canvas.id,false,Math.PI),client=new NativeAcceptedMapClient({request:boundedMapRequest,
      action:boundedMapAction,clear:epoch=>map.set({map_epoch:epoch,scan_count:0}),
      commit:snapshot=>map.setTiles(snapshot),status:value=>{
        const labels={display_stopped:'표시 멈춤 · 마지막 지도 유지',waiting:'관측 지도 준비 중',
          observing:'센서 지도 관찰 중',retained:'관찰 종료·대기 · 마지막 지도 유지',
          unavailable:'연결 확인 대기 · 마지막 지도 유지',new_map:'새 지도 준비 · 관찰 시작 필요',
          new_map_denied:'생산자 관찰 멈춤 확인 필요 · 기존 지도 유지'};
        status.textContent=labels[value.state]||labels.waiting;
        if(value.map_version!==undefined)status.textContent+=` · 완료 ${value.map_version} · 관측 셀 ${value.cell_count}`;
        if(value.status?.map?.pose_continuity===false&&value.retained)status.textContent+=' · 연결 정합 대기';
        historyWarning.textContent=historyNotice.update(value);historyWarning.hidden=!historyWarning.textContent;
      }});
    container._nativeMapView=client;
    start.addEventListener('click',()=>client.start());stop.addEventListener('click',()=>client.stop());
    fresh.addEventListener('click',()=>client.newMap());
    window.addEventListener('pagehide',()=>client.stop()); // bfcache Back requires explicit display Start
    return client;
  } finally {container._nativeMapMounting=false;}
}
