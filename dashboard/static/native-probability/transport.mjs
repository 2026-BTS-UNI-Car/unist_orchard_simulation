// SPDX-License-Identifier: Apache-2.0
// Optional display transport. Instantiate in a Worker; default has no requests.
import {NativeWireReceiver,parseNativePacket,SCHEMA} from './native_wire_receiver.mjs';
export const API='/api/sim/native-map';
const check=(v,why)=>{if(!v)throw Error(why);};
const integer=(v,a,b)=>Number.isSafeInteger(v)&&v>=a&&v<=b;
const shaText=v=>typeof v==='string'&&/^[0-9a-f]{64}$/.test(v);
const identityKeys=['publisher_epoch','generation','sequence','transform_revision','map_frame','snapshot_sha256'];
export const sameIdentity=(a,b)=>!!a&&!!b&&identityKeys.every(k=>a[k]===b[k]);
const canonical=v=>JSON.stringify(v,(_,x)=>x&&typeof x==='object'&&!Array.isArray(x)?Object.fromEntries(Object.keys(x).sort().map(k=>[k,x[k]])):x);
export const digest=async bytes=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',bytes)),x=>x.toString(16).padStart(2,'0')).join('');
export function identity(v,withSHA=true) {
  check(v&&typeof v.publisher_epoch==='string'&&/^[0-9a-f]{32}$/.test(v.publisher_epoch)&&integer(v.generation,1,Number.MAX_SAFE_INTEGER)&&
    integer(v.sequence,1,Number.MAX_SAFE_INTEGER)&&integer(v.transform_revision,0,Number.MAX_SAFE_INTEGER)&&typeof v.map_frame==='string'&&v.map_frame.length>0&&v.map_frame.length<=256,'snapshot_identity');
  if(withSHA)check(shaText(v.snapshot_sha256),'snapshot_SHA');
  return Object.freeze(Object.fromEntries(identityKeys.filter(k=>withSHA||k!=='snapshot_sha256').map(k=>[k,v[k]])));
}
function scope(v) {
  check(v.runtime_authority===false&&v.physical_outputs===false&&v.live_sensor_admission_qualified===false&&v.global_posterior_fused===false,'display_only_scope');
}
function status(raw) {
  const s=parseNativePacket(raw);scope(s);
  check(s.schema==='ubts.cartographer.native_view.status.v1'&&['enabled','available','stale','stopped'].every(k=>typeof s[k]==='boolean')&&
    (s.reason===null||(typeof s.reason==='string'&&s.reason.length<=512)),'native_status');
  if(s.available)identity(s.latest_complete);else check(s.latest_complete===null,'unavailable_identity');return s;
}
class HTTPFailure extends Error {
  constructor(status){super('HTTP_'+status);this.status=status;}
}
class LatestRace extends Error {
  constructor(reason,target){super(reason);this.target=target;}
}
function sameContextForward(next,previous) {
  check(next.publisher_epoch===previous.publisher_epoch&&next.generation===previous.generation&&next.map_frame===previous.map_frame,'snapshot_race_context_changed');
  check(next.sequence>=previous.sequence&&next.transform_revision>=previous.transform_revision,'snapshot_race_rewind');
  if(next.sequence===previous.sequence)check(sameIdentity(next,previous),'changed_immutable_snapshot');
}
export async function fetchBounded(fetcher,url,signal,cap,expected=null,immutable=false) {
  const response=await fetcher(url,{method:'GET',credentials:'same-origin',cache:immutable?'force-cache':'no-store',redirect:'error',signal});
  if(!response.ok)throw new HTTPFailure(response.status);check(response.body&&typeof response.body.getReader==='function','bounded_stream_required');
  check(expected===null||integer(expected,0,cap),'body_size_bound');
  const length=response.headers?.get('content-length'),encoding=response.headers?.get('content-encoding');
  if(length&&!encoding)check(/^[0-9]+$/.test(length)&&Number(length)<=cap&&(expected===null||Number(length)===expected),'HTTP_length');
  const reader=response.body.getReader(),target=expected===null?null:new Uint8Array(expected),parts=[];let size=0;
  try {
    for(;;){check(!signal.aborted,'request_cancelled');const {done,value}=await reader.read();if(done)break;
      check(value instanceof Uint8Array&&size+value.length<=(expected??cap),'HTTP_decoded_byte_cap');
      if(target)target.set(value,size);else parts.push(value);size+=value.length;
    }
    check(expected===null||size===expected,'HTTP_decoded_length');
    if(target)return target;const result=new Uint8Array(size);let at=0;for(const part of parts){result.set(part,at);at+=part.length;}return result;
  } finally {try{await reader.cancel();}catch{}reader.releaseLock();}
}
function base64(bytes) {
  const strings=[];for(let i=0;i<bytes.length;i+=8192)strings.push(String.fromCharCode(...bytes.subarray(i,i+8192)));
  return btoa(strings.join(''));
}
export const yieldTask=()=>new Promise(resolve=>{
  const c=new MessageChannel();c.port1.onmessage=()=>{c.port1.close();c.port2.close();resolve();};c.port2.postMessage(null);
});

export class LatestNativeClient {
  #fetch;#sha;#prepare;#commit;#notify;#yield;#limits;#receiver;#cache=new Map();#visible=null;#latest=null;#decoded=null;
  #accepted=null;#retired=new Set();
  #enabled=false;#busy=null;#token=0;#abort=null;#timer=null;#poll;#timeout;#stale=true;#reason='inactive';#repaint=false;#producerStopped=false;
  constructor({fetchImpl=globalThis.fetch,sha256=digest,prepareView=async v=>v,onCommit=()=>{},onState=()=>{},yieldFn=yieldTask,
    maxCells=4194304,pollMs=500,timeoutMs=15000}={}) {
    check(integer(maxCells,1,4194304)&&integer(pollMs,100,5000)&&integer(timeoutMs,100,30000),'client_budget');
    this.#fetch=fetchImpl;this.#sha=sha256;this.#prepare=prepareView;this.#commit=onCommit;this.#notify=onState;this.#yield=yieldFn;
    this.#limits={maxBatchCells:maxCells,maxCacheCells:maxCells};this.#receiver=new NativeWireReceiver(this.#limits);this.#poll=pollMs;this.#timeout=timeoutMs;
  }
  state(){return Object.freeze({enabled:this.#enabled,inFlight:!!this.#busy,stale:this.#stale,reason:this.#reason,producerStopped:this.#producerStopped,latestComplete:this.#latest});}
  visibleView(){return this.#visible;}
  repaint(){this.#repaint=true;return this.refresh();}
  #state(stale,reason,producerStopped=false){this.#stale=stale;this.#reason=reason;this.#producerStopped=producerStopped;this.#notify(this.state());}
  start({poll=true}={}){if(this.#enabled)return this.#busy;this.#enabled=true;return this.refresh().finally(()=>{if(poll)this.#schedule();});}
  #schedule(){if(this.#enabled&&!this.#timer)this.#timer=setTimeout(()=>{this.#timer=null;this.refresh().finally(()=>this.#schedule());},this.#poll);}
  stop(reason='client_stopped') {
    this.#enabled=false;++this.#token;clearTimeout(this.#timer);this.#timer=null;this.#abort?.abort();
    // An interrupted decoder staging batch never becomes the next start's cache.
    this.#receiver=new NativeWireReceiver(this.#limits);this.#cache=new Map();this.#decoded=null;this.#state(true,reason);
  }
  refresh(){
    if(!this.#enabled)return Promise.resolve({kind:'inactive'});if(this.#busy)return this.#busy;
    const token=this.#token,abort=this.#abort=new AbortController();const timer=setTimeout(()=>abort.abort(),this.#timeout);
    this.#busy=this.#recover(token,abort.signal).catch(error=>{
      if(token===this.#token)this.#state(true,String(error.message||error).slice(0,512));return {kind:'retained',reason:String(error.message||error).slice(0,512)};
    }).finally(()=>{clearTimeout(timer);this.#busy=null;});return this.#busy;
  }
  #active(token,signal){check(this.#enabled&&token===this.#token&&!signal.aborted,'request_cancelled');}
  async #getStatus(signal){return status(new TextDecoder('utf-8',{fatal:true}).decode(await fetchBounded(this.#fetch,API+'/status',signal,1048576)));}
  async #recover(token,signal) {
    // At most two restarts share refresh's controller, deadline and single flight.
    let previous=null;
    for(let restarts=0;;++restarts){this.#active(token,signal);
      try{return await this.#run(token,signal,previous);}
      catch(error){
        if(!(error instanceof LatestRace))throw error;
        this.#active(token,signal);
        check(restarts<2,'latest_recovery_restart_cap:'+error.message);
        previous=error.target;await this.#yield();this.#active(token,signal);
      }
    }
  }
  async #run(token,signal,previous) {
    const first=await this.#getStatus(signal);this.#active(token,signal);
    if(!first.available){this.#state(true,first.reason??'no_complete_snapshot');return {kind:'retained'};}
    const wanted=identity(first.latest_complete);
    if(previous)sameContextForward(wanted,previous);
    check(!this.#retired.has(wanted.publisher_epoch),'retired_HTTP_epoch');
    if(this.#accepted){const a=this.#accepted;
      if(a.publisher_epoch===wanted.publisher_epoch){
        check(wanted.map_frame===a.map_frame,'HTTP_map_frame_changed');
        check(wanted.generation>=a.generation,'HTTP_generation_rewind');
        if(wanted.generation===a.generation){
          check(wanted.sequence>=a.sequence&&wanted.transform_revision>=a.transform_revision,'HTTP_sequence_or_pose_rewind');
          if(wanted.sequence===a.sequence)check(sameIdentity(wanted,a),'changed_immutable_snapshot');
        }
      }else check(this.#retired.size<16,'HTTP_epoch_history_cap');
    }
    if(!this.#repaint&&sameIdentity(wanted,this.#latest)&&sameIdentity(wanted,this.#decoded?.identity)){this.#state(first.stale||first.stopped,first.reason,first.stopped);return {kind:'unchanged'};}
    this.#state(true,'loading_complete_snapshot');
    if(sameIdentity(wanted,this.#decoded?.identity))return this.#present(this.#decoded,token,signal);
    const raw=await fetchBounded(this.#fetch,API+'/snapshot',signal,1048576);this.#active(token,signal);
    const rawSHA=await this.#sha(raw);this.#active(token,signal);
    const snapshot=parseNativePacket(new TextDecoder('utf-8',{fatal:true}).decode(raw));scope(snapshot);
    check(snapshot.schema==='ubts.cartographer.native_view.snapshot.v1'&&snapshot.transport_reconstructed_full_refresh===true,'snapshot_schema');
    const observed=identity({...snapshot,snapshot_sha256:rawSHA});
    sameContextForward(observed,wanted);
    const manifestRaw=snapshot.producer_manifest_raw,doneRaw=snapshot.producer_done_raw;
    check(typeof manifestRaw==='string'&&typeof doneRaw==='string'&&shaText(snapshot.producer_manifest_sha256)&&shaText(snapshot.producer_done_sha256),'producer_raw_required');
    check(await this.#sha(new TextEncoder().encode(manifestRaw))===snapshot.producer_manifest_sha256&&await this.#sha(new TextEncoder().encode(doneRaw))===snapshot.producer_done_sha256,'producer_raw_SHA');
    const m=parseNativePacket(manifestRaw),done=parseNativePacket(doneRaw);this.#active(token,signal);
    check(m.schema===SCHEMA&&done.schema===SCHEMA&&m.kind==='manifest'&&done.kind==='done'&&identityKeys.slice(0,5).every(k=>m[k]===observed[k]&&done[k]===observed[k]),'producer_identity');
    if(snapshot.producer_manifest!==undefined)check(canonical(snapshot.producer_manifest)===canonical(m),'manifest_mirror_changed');
    if(snapshot.producer_done!==undefined)check(canonical(snapshot.producer_done)===canonical(done),'DONE_mirror_changed');
    check(Array.isArray(m.submaps)&&Array.isArray(snapshot.submaps)&&m.submaps.length===snapshot.submaps.length&&m.submaps.length<=256,'snapshot_membership');
    let total=0;const entries=[],oldState=this.#receiver.state(),sameContext=oldState.epoch===wanted.publisher_epoch&&oldState.generation===wanted.generation;
    for(let i=0;i<m.submaps.length;++i){
      const {body,...metadata}=snapshot.submaps[i];check(canonical(metadata)===canonical(m.submaps[i]),'body_metadata_changed');
      check(body&&integer(body.cells,1,4194304)&&body.cells===m.submaps[i].cells&&body.layout==='probability_f32le_then_known_u8'&&body.bytes===body.cells*5&&
        body.probability_bytes===body.cells*4&&body.known_bytes===body.cells&&shaText(body.sha256),'body_descriptor');
      total+=body.cells;check(total<=this.#limits.maxCacheCells,'snapshot_total_cell_cap');
      const url=API+'/body/'+wanted.publisher_epoch+'/'+wanted.generation+'/'+m.submaps[i].body_id+'/'+body.sha256;
      check(body.url===url,'same_origin_content_addressed_body');
      const cached=sameContext&&this.#cache.get(m.submaps[i].body_id)?.sha256===body.sha256;
      entries.push({metadata:m.submaps[i],descriptor:body,included:!cached,bytes:null});
    }
    // Never adopt the unbound newer snapshot. A fresh status must pin its SHA
    // on the next attempt; malformed metadata/caps/context already failed.
    if(!sameIdentity(observed,wanted))throw new LatestRace('snapshot_advanced_before_GET',wanted);
    // Fetch and SHA all missing bodies BEFORE touching the receiver's staging.
    for(const e of entries)if(e.included){
      try{e.bytes=await fetchBounded(this.#fetch,e.descriptor.url,signal,e.descriptor.bytes,e.descriptor.bytes,true);}
      catch(error){if(error instanceof HTTPFailure&&error.status===410)throw new LatestRace('body_evicted_before_fetch',wanted);throw error;}
      this.#active(token,signal);
      check(await this.#sha(e.bytes)===e.descriptor.sha256,'body_SHA_changed');this.#active(token,signal);
    }
    const synthetic={...m,full_refresh:false,submaps:entries.map(e=>({...e.metadata,body_included:e.included}))};
    const receiver=this.#receiver;let completed;
    try {
    check(receiver.accept(JSON.stringify(synthetic)).kind==='pending','display_manifest_rejected');let chunks=0;
    for(const e of entries)if(e.included){
      const n=e.descriptor.cells;
      for(let at=0;at<n;at+=16384){this.#active(token,signal);const count=Math.min(16384,n-at);
        const packet={...identity(m,false),schema:m.schema,kind:'cells',values_semantics:m.values_semantics,source_kind:m.source_kind,source_session:null,
          live_sensor_admission_qualified:false,runtime_authority:false,physical_outputs:false,body_id:e.metadata.body_id,offset_cells:at,count_cells:count,
          probability_f32le_base64:base64(e.bytes.subarray(at*4,(at+count)*4)),known_u8_base64:base64(e.bytes.subarray(n*4+at,n*4+at+count))};
        check(receiver.accept(JSON.stringify(packet)).kind==='pending','display_cells_rejected');
        if(++chunks%4===0){await this.#yield();this.#active(token,signal);}
      }e.bytes=null;
    }
    completed=receiver.accept(doneRaw);check(completed.kind==='commit','display_DONE_rejected');this.#active(token,signal);
    }catch(error){
      // Interrupted staging can advance the codec high-water sequence. Discard
      // only that decoder/cache; immutable completed UI/body references survive.
      // Next retry fetches omitted bodies in full. No cached input is made fresh.
      if(receiver===this.#receiver){this.#receiver=new NativeWireReceiver(this.#limits);this.#cache=new Map();}
      throw error;
    }
    if(this.#accepted&&this.#accepted.publisher_epoch!==wanted.publisher_epoch)this.#retired.add(this.#accepted.publisher_epoch);
    this.#accepted=wanted;
    this.#cache=new Map(entries.map(e=>[e.metadata.body_id,{sha256:e.descriptor.sha256}]));
    this.#decoded={view:completed.view,identity:wanted,manifestSHA:snapshot.producer_manifest_sha256,doneSHA:snapshot.producer_done_sha256};
    return this.#present(this.#decoded,token,signal);
  }
  async #present(decoded,token,signal) {
    const prepared=await this.#prepare(decoded.view,()=>this.#active(token,signal));this.#active(token,signal);
    // A validated complete view remains useful while a newer same-context
    // snapshot is available. Requiring exact latest identity can starve slow
    // presentation forever; the next single-flight poll will catch up.
    const last=await this.#getStatus(signal);this.#active(token,signal);
    check(last.available,'no_complete_snapshot_after_prepare');
    const latest=identity(last.latest_complete),current=decoded.identity;
    check(latest.publisher_epoch===current.publisher_epoch&&latest.generation===current.generation&&latest.map_frame===current.map_frame,'snapshot_context_changed_after_prepare');
    check(latest.sequence>=current.sequence&&latest.transform_revision>=current.transform_revision,'snapshot_rewind_after_prepare');
    if(latest.sequence===current.sequence)check(sameIdentity(latest,current),'changed_immutable_snapshot');
    const newer=latest.sequence>current.sequence;
    const stale=newer||last.stale||last.stopped,reason=newer?'newer_complete_available':last.reason;
    this.#commit({prepared,identity:decoded.identity,stale,reason,producerStopped:last.stopped,original_manifest_sha256:decoded.manifestSHA,
      original_DONE_sha256:decoded.doneSHA,transport_reconstructed_full_refresh:true});
    this.#visible=decoded.view;this.#latest=decoded.identity;this.#repaint=false;this.#state(stale,reason,last.stopped);return {kind:'commit'};
  }
}
