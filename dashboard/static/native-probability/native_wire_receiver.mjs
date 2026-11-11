// Optional display decoder only. No DOM, transport, native owner, or authority.
export const SCHEMA = 'ubts.cartographer.native_probability_submaps.v1';
const U32 = 4294967296;
const copyBytes = Uint8Array.prototype.set;
const fail = reason => { throw new Error(reason); };
const integer = (v, lo, hi) => Number.isSafeInteger(v) && v >= lo && v <= hi ? v : fail('bounded_integer');
const text = (v, cap) => typeof v === 'string' && v.length <= cap ? v : fail('bounded_text');
const finite = v => typeof v === 'number' && Number.isFinite(v) ? v : fail('finite_number');

// FNV1a64 using two unsigned 32-bit words. Integer multiply for carry is <2^41.
// FNV is a corruption diagnostic, not authentication. No per-byte BigInt.
export function fnv1a64(...parts) {
  let hi = 0xcbf29ce4, lo = 0x84222325;
  for (const bytes of parts) {
    if (!(bytes instanceof Uint8Array)) fail('uint8_bytes_required');
    for (let i = 0; i < bytes.length; ++i) {
      lo = (lo ^ bytes[i]) >>> 0;
      const carry = Math.floor(lo * 435 / U32);
      hi = (Math.imul(hi, 435) + Math.imul(lo, 256) + carry) >>> 0;
      lo = Math.imul(lo, 435) >>> 0;
    }
  }
  return hi.toString(16).padStart(8, '0') + lo.toString(16).padStart(8, '0');
}

function utf8Within(raw, cap) {
  if (typeof raw !== 'string' || raw.length > cap) fail('packet_byte_budget');
  let bytes = 0;
  for (let i = 0; i < raw.length; ++i) {
    const c = raw.charCodeAt(i);
    if (c < 128) ++bytes;
    else if (c < 2048) bytes += 2;
    else if (c >= 0xd800 && c <= 0xdbff) {
      const low = raw.charCodeAt(++i);
      if (!(low >= 0xdc00 && low <= 0xdfff)) fail('invalid_utf16');
      bytes += 4;
    } else if (c >= 0xdc00 && c <= 0xdfff) fail('invalid_utf16');
    else bytes += 3;
    if (bytes > cap) fail('packet_byte_budget');
  }
  return bytes;
}

// Bounded JSON parser rejects duplicate keys. Unsafe integer tokens are kept as
// exact decimal strings (timestamps), never rounded or used as numeric IDs.
export function parseNativePacket(raw, maxBytes = 1048576) {
  integer(maxBytes,1,1048576);
  utf8Within(raw, maxBytes);
  let at = 0, nodes = 0;
  const numberToken = /-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+-]?\d+)?/y;
  const white = () => {
    while (at < raw.length) {
      const c = raw.charCodeAt(at);
      if (c!==32 && c!==9 && c!==10 && c!==13) break;
      ++at;
    }
  };
  function string() {
    const start = at++;
    while (at < raw.length) {
      const c = raw[at++];
      if (c === '"') return JSON.parse(raw.slice(start, at));
      if (c === '\\') ++at;
    }
    fail('unterminated_string');
  }
  function value(depth) {
    if (++nodes > 50000 || depth > 32) fail('json_structure_budget');
    white(); const c = raw[at];
    if (c === '"') return string();
    if (c === '{') {
      ++at; white(); const result = Object.create(null);
      if (raw[at] === '}') { ++at; return result; }
      for (;;) {
        white(); if (raw[at] !== '"') fail('object_key');
        const key = string(); white(); if (raw[at++] !== ':') fail('object_colon');
        if (Object.hasOwn(result, key)) fail('duplicate_json_key');
        result[key] = value(depth + 1); white();
        const end = raw[at++]; if (end === '}') return result;
        if (end !== ',') fail('object_separator');
      }
    }
    if (c === '[') {
      ++at; white(); const result = [];
      if (raw[at] === ']') { ++at; return result; }
      for (;;) {
        result.push(value(depth + 1)); white(); const end = raw[at++];
        if (end === ']') return result;
        if (end !== ',') fail('array_separator');
      }
    }
    if (raw.startsWith('true',at)) { at+=4; return true; }
    if (raw.startsWith('false',at)) { at+=5; return false; }
    if (raw.startsWith('null',at)) { at+=4; return null; }
    numberToken.lastIndex = at;
    const match = numberToken.exec(raw);
    if (!match) fail('json_value');
    const token = match[0]; at += token.length; const number = Number(token);
    if (!Number.isFinite(number)) fail('nonfinite_json_number');
    return /^-?\d+$/.test(token) && !Number.isSafeInteger(number) ? token : number;
  }
  const result = value(0); white(); if (at !== raw.length) fail('json_trailing');
  if (!result || Array.isArray(result) || typeof result !== 'object') fail('packet_object');
  return result;
}

function array(v, n) {
  if (!Array.isArray(v) || v.length !== n) fail('fixed_array');
  return Object.freeze(v.map(finite));
}
function pose(v) {
  if (!v || typeof v !== 'object') fail('pose');
  const translation = array(v.translation, 3), q_xyzw = array(v.q_xyzw, 4);
  if (Math.abs(q_xyzw.reduce((s, x) => s + x*x, 0) - 1) > 1e-8) fail('unit_quaternion');
  return Object.freeze({translation, q_xyzw});
}
function rotate(q, v) {
  const [x,y,z,w] = q, [a,b,c] = v;
  const tx = 2*(y*c-z*b), ty = 2*(z*a-x*c), tz = 2*(x*b-y*a);
  return [a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx];
}
export function nativeCellLocal(entry, i0, i1) {
  const [n0,n1] = entry.crop_dimensions_n0_n1;
  integer(i0, 0, n0-1); integer(i1, 0, n1-1);
  const [o0,o1] = entry.crop_offset_i0_i1, [mx,my] = entry.native_max_xy, r = entry.resolution;
  const result = [Math.fround(mx-r*(o1+i1+.5)), Math.fround(my-r*(o0+i0+.5)), 0];
  result.forEach(finite); return result;
}
export function nativeCellMap(entry, i0, i1) {
  const local = nativeCellLocal(entry, i0, i1), a = entry.T_local_submap, b = entry.T_map_submap;
  const inv = [-a.q_xyzw[0],-a.q_xyzw[1],-a.q_xyzw[2],a.q_xyzw[3]];
  const submap = rotate(inv, local.map((x,i) => x-a.translation[i]));
  return rotate(b.q_xyzw, submap).map((x,i) => finite(x+b.translation[i]));
}

const alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
const base64Digits = new Int16Array(128).fill(-1);
for (let i=0;i<alphabet.length;++i) base64Digits[alphabet.charCodeAt(i)]=i;
const base64Digit = (s,i) => s.charCodeAt(i)<128 ? base64Digits[s.charCodeAt(i)] : -1;
function decodeInto(encoded, count, target, offset) {
  text(encoded, 4*Math.ceil(count/3));
  if (encoded.length !== 4*Math.ceil(count/3)) fail('base64_length');
  let out = offset;
  for (let i = 0; i < encoded.length; i += 4) {
    const left = count-(out-offset), a = base64Digit(encoded,i), b = base64Digit(encoded,i+1);
    const c = encoded[i+2] === '=' ? -1 : base64Digit(encoded,i+2);
    const d = encoded[i+3] === '=' ? -1 : base64Digit(encoded,i+3);
    if (a < 0 || b < 0 || (left > 1 ? c < 0 : encoded[i+2] !== '=') ||
        (left > 2 ? d < 0 : encoded[i+3] !== '=') ||
        (left === 1 && (b & 15)) || (left === 2 && (c & 3))) fail('invalid_base64');
    target[out++] = (a<<2)|(b>>4);
    if (left > 1) target[out++] = ((b&15)<<4)|(c>>2);
    if (left > 2) target[out++] = ((c&3)<<6)|d;
  }
}

const DEFAULTS = Object.freeze({maxPacketBytes:1048576, maxSubmaps:256,
  maxBodyCells:4194304, maxBatchCells:4194304, maxCacheCells:4194304,
  maxRetainedBodyBytes:41943040, maxBatchPackets:8192, maxBatchWireBytes:67108864,
  maxRetiredEpochs:16});
const HARD = Object.freeze({...DEFAULTS,maxBatchCells:16777216,maxCacheCells:16777216,
  maxRetainedBodyBytes:167772160,maxBatchWireBytes:268435456});

export class NativeWireReceiver {
  #limits; #epoch = null; #generation = 0; #frame = null; #retired = new Set();
  #cache = new Map(); #stage = null; #visible = null; #stale = true; #reason = 'awaiting_complete';
  #highSequence = 0; #highRevision = 0; #blocked = false; #closed = false;
  #counts = {packets:0, rejected:0, superseded:0, commits:0};
  constructor(limits = {}) {
    for (const k of Object.keys(limits)) if (!Object.hasOwn(DEFAULTS,k)) fail('unknown_budget');
    this.#limits = Object.freeze({...DEFAULTS,...limits});
    for (const [k,v] of Object.entries(this.#limits)) integer(v,1,HARD[k]);
  }
  view() { return this.#visible ? Object.freeze({...this.#visible,stale:this.#stale,staleReason:this.#reason}) : null; }
  state() { return Object.freeze({epoch:this.#epoch,generation:this.#generation,
    stagingSequence:this.#stage?.sequence ?? null, cacheBodies:this.#cache.size,
    visibleBodyBytes:this.#visible?.bodyBytes ?? 0, stagingBodyBytes:this.#stage?.allocatedBytes ?? 0,
    stale:this.#stale,reason:this.#reason,closed:this.#closed,...this.#counts}); }
  cancel(reason = 'local_cancel') {
    const boundedReason = text(reason,256);
    this.#closed = true; this.#stage = null; this.#cache.clear();
    this.#stale = true; this.#reason = boundedReason; return this.#event('status');
  }
  #event(kind, extra = {}) { return Object.freeze({kind,...extra,view:this.view(),state:this.state()}); }
  #prefix(p) {
    if (p.schema !== SCHEMA || !['manifest','cells','done','status'].includes(p.kind)) fail('wire_schema');
    if (!/^[0-9a-f]{32}$/.test(text(p.publisher_epoch,32))) fail('publisher_epoch');
    integer(p.generation,1,Number.MAX_SAFE_INTEGER); text(p.map_frame,256);
    if (!p.map_frame || /[\x00-\x1f]/.test(p.map_frame)) fail('map_frame');
    if (p.values_semantics !== 'native_float32_from_quantized_probability_grid' ||
        p.source_kind !== 'cartographer_node_state' || p.source_session !== null ||
        p.live_sensor_admission_qualified !== false || p.runtime_authority !== false || p.physical_outputs !== false)
      fail('display_scope');
  }
  #identity(p) {
    if (this.#retired.has(p.publisher_epoch)) return false;
    if (this.#epoch !== p.publisher_epoch) {
      if (this.#epoch && this.#retired.size >= this.#limits.maxRetiredEpochs) fail('epoch_history_budget');
      if (this.#epoch) this.#retired.add(this.#epoch);
      this.#epoch = p.publisher_epoch; this.#generation = p.generation; this.#frame = p.map_frame;
      this.#stage = null; this.#cache.clear(); this.#highSequence = 0; this.#highRevision = 0; this.#blocked = false;
      this.#stale = true; this.#reason = 'publisher_epoch_changed';
    } else if (p.generation < this.#generation) return false;
    else if (p.generation > this.#generation) {
      this.#generation = p.generation; this.#stage = null; this.#cache.clear(); this.#highSequence = 0; this.#highRevision = 0;
      this.#blocked = false; this.#stale = true; this.#reason = 'generation_changed';
    }
    if (p.map_frame !== this.#frame) fail('map_frame_changed_within_epoch');
    return true;
  }
  accept(raw) {
    if (this.#closed) return this.#event('ignored',{reason:'receiver_cancelled'});
    let identityBefore = null;
    try {
      const bytes = utf8Within(raw,this.#limits.maxPacketBytes), p = parseNativePacket(raw,this.#limits.maxPacketBytes);
      this.#prefix(p); ++this.#counts.packets;
      if (this.#epoch !== p.publisher_epoch || p.generation > this.#generation) {
        // Only a fully validated manifest/status may establish a new identity.
        if (p.kind === 'cells' || p.kind === 'done')
          return this.#event('ignored',{reason:'new_identity_requires_manifest_or_status'});
        identityBefore = {epoch:this.#epoch,generation:this.#generation,frame:this.#frame,
          retired:new Set(this.#retired),cache:new Map(this.#cache),
          highSequence:this.#highSequence,highRevision:this.#highRevision,blocked:this.#blocked};
      }
      if (!this.#identity(p)) return this.#event('ignored',{reason:'retired_identity'});
      if (p.kind === 'status') {
        if (typeof p.cancelled !== 'boolean' || typeof p.limited !== 'boolean') fail('typed_status');
        text(p.error,256);
        if (p.cancelled || p.limited || p.error) {
          this.#stage = null; this.#cache.clear(); this.#stale = true;
          this.#blocked = p.cancelled || p.limited;
          this.#reason = p.cancelled ? 'publisher_cancelled' : p.limited ? 'publisher_limited' : 'publisher_error';
        }
        return this.#event('status');
      }
      if (this.#blocked) return this.#event('ignored',{reason:'identity_cancelled_or_limited'});
      integer(p.sequence,1,Number.MAX_SAFE_INTEGER);
      if (p.kind === 'manifest') return this.#manifest(p,raw,bytes);
      if (!this.#stage || p.sequence !== this.#stage.sequence) {
        if (p.sequence > this.#highSequence) { this.#stage = null; this.#stale = true; this.#reason = 'missing_manifest'; }
        return this.#event('ignored',{reason:'not_current_staging_batch'});
      }
      if (++this.#stage.packets > this.#limits.maxBatchPackets ||
          (this.#stage.wireBytes += bytes) > this.#limits.maxBatchWireBytes) fail('batch_wire_budget');
      if (p.kind === 'cells') return this.#cells(p);
      return this.#done(p);
    } catch (error) {
      if (identityBefore) {
        this.#epoch=identityBefore.epoch; this.#generation=identityBefore.generation;
        this.#frame=identityBefore.frame; this.#retired=identityBefore.retired;
        this.#cache=identityBefore.cache; this.#highSequence=identityBefore.highSequence;
        this.#highRevision=identityBefore.highRevision; this.#blocked=identityBefore.blocked;
      }
      this.#stage = null; this.#stale = true; this.#reason = String(error.message || error).slice(0,256);
      ++this.#counts.rejected; return this.#event('rejected',{reason:this.#reason});
    }
  }
  #manifest(p,raw,bytes) {
    if (p.sequence < this.#highSequence || (p.sequence === this.#highSequence && !this.#stage))
      return this.#event('ignored',{reason:'old_sequence'});
    if (this.#stage?.sequence === p.sequence) {
      if (raw !== this.#stage.raw) fail('changed_duplicate_manifest');
      return this.#event('ignored',{reason:'duplicate_manifest'});
    }
    integer(p.transform_revision,0,Number.MAX_SAFE_INTEGER);
    if (p.transform_revision < this.#highRevision) fail('transform_revision_rewind');
    if (typeof p.full_refresh !== 'boolean' || !Array.isArray(p.submaps) || p.submaps.length > this.#limits.maxSubmaps) fail('manifest_budget');
    const entries = [], ids = new Set(), keys = new Set(); let total = 0, allocatedBytes = 0;
    for (const e of p.submaps) {
      const key = integer(e.trajectory_id,0,2147483647)+':'+integer(e.submap_index,0,2147483647);
      const id = integer(e.body_id,1,Number.MAX_SAFE_INTEGER), cells = integer(e.cells,1,this.#limits.maxBodyCells);
      if (keys.has(key) || ids.has(id)) fail('duplicate_submap_or_body'); keys.add(key); ids.add(id);
      total += cells; if (total > this.#limits.maxBatchCells || total > this.#limits.maxCacheCells) fail('batch_cell_budget');
      if (typeof e.body_included !== 'boolean' || typeof e.finished !== 'boolean' || (p.full_refresh && !e.body_included)) fail('typed_body_flags');
      const dims = array(e.crop_dimensions_n0_n1,2).map(x=>integer(x,1,2147483647));
      const offsets = array(e.crop_offset_i0_i1,2).map(x=>integer(x,0,2147483647));
      if (dims[0]*dims[1] !== cells || offsets.some((x,i)=>x+dims[i]>2147483647)) fail('crop_dimensions');
      const r = finite(e.resolution); if (!(r>0)) fail('resolution');
      const max = array(e.native_max_xy,2), local = pose(e.T_local_submap), global = pose(e.T_map_submap);
      if (!/^[0-9a-f]{16}$/.test(text(e.body_fingerprint_fnv64,16))) fail('body_fingerprint');
      if (e.probability_encoding !== 'float32_little_endian' || e.known_encoding !== 'uint8_0_or_1' || e.flat_index !== 'i0+n0*i1') fail('body_encoding');
      const metadata = Object.freeze({trajectory_id:e.trajectory_id,submap_index:e.submap_index,
        native_version:integer(e.native_version,0,2147483647),finished:e.finished,body_id:id,cells,
        known_cells:integer(e.known_cells,0,cells),body_fingerprint_fnv64:e.body_fingerprint_fnv64,
        crop_dimensions_n0_n1:Object.freeze(dims),crop_offset_i0_i1:Object.freeze(offsets),
        resolution:r,native_max_xy:max,T_local_submap:local,T_map_submap:global});
      for (const [i0,i1] of [[0,0],[dims[0]-1,0],[0,dims[1]-1],[dims[0]-1,dims[1]-1]])
        nativeCellMap(metadata,i0,i1); // Reject unrenderable f32/pose overflow before buffers.
      const geometry = JSON.stringify({...metadata,T_map_submap:null});
      let body = null;
      if (!e.body_included) {
        body = this.#cache.get(id);
        if (!body || body.geometry !== geometry) fail('missing_or_changed_cached_body');
      } else allocatedBytes += cells*5;
      entries.push({metadata,geometry,body,included:e.body_included});
    }
    if (allocatedBytes+(this.#visible?.bodyBytes??0) > this.#limits.maxRetainedBodyBytes) fail('retained_body_budget');
    if (bytes > this.#limits.maxBatchWireBytes) fail('batch_wire_budget');
    if (this.#stage) ++this.#counts.superseded;
    this.#stage = null; this.#stale = true; this.#reason = 'incomplete_batch';
    // All cell/cache/retention budgets checked BEFORE any cell buffer allocation.
    for (const e of entries) if (e.included) {
      const cells = e.metadata.cells, buffer = new ArrayBuffer(cells*5);
      e.body = {geometry:e.geometry,prob:new Uint8Array(buffer,0,cells*4),known:new Uint8Array(buffer,cells*4,cells),
        data:new DataView(buffer,0,cells*4),offset:0,knownCount:0};
    }
    this.#highSequence = p.sequence;
    this.#highRevision = p.transform_revision;
    this.#stage = {sequence:p.sequence,revision:p.transform_revision,raw,entries,
      byId:new Map(entries.map(e=>[e.metadata.body_id,e])),allocatedBytes,packets:1,wireBytes:bytes};
    return this.#event('pending');
  }
  #cells(p) {
    const e = this.#stage.byId.get(integer(p.body_id,1,Number.MAX_SAFE_INTEGER));
    if (!e?.included) fail('unreferenced_or_cached_body_chunk');
    const b = e.body, count = integer(p.count_cells,1,16384);
    if (integer(p.offset_cells,0,e.metadata.cells) !== b.offset || count > e.metadata.cells-b.offset) fail('missing_overlapping_or_unordered_chunk');
    decodeInto(p.probability_f32le_base64,count*4,b.prob,b.offset*4);
    decodeInto(p.known_u8_base64,count,b.known,b.offset);
    for (let i=b.offset;i<b.offset+count;++i) {
      const v = b.data.getFloat32(i*4,true), k = b.known[i];
      if ((k!==0&&k!==1) || !Number.isFinite(v) || v<0 || v>1) fail('invalid_probability_or_mask');
      if (k===0 && (b.prob[i*4]||b.prob[i*4+1]||b.prob[i*4+2]||b.prob[i*4+3])) fail('unknown_nonzero_placeholder');
      b.knownCount += k;
    }
    b.offset += count; return this.#event('pending');
  }
  #done(p) {
    const stage = this.#stage;
    if (p.producer_batch_complete !== true || p.subscriber_delivery_qualified !== false || p.transform_revision !== stage.revision) fail('incomplete_done');
    const cache = new Map(), submaps = []; let bodyBytes = 0;
    for (const e of stage.entries) {
      const b = e.body, m = e.metadata;
      if (e.included && (b.offset !== m.cells || b.knownCount !== m.known_cells || fnv1a64(b.prob,b.known) !== m.body_fingerprint_fnv64)) fail('incomplete_or_corrupt_body');
      cache.set(m.body_id,b); bodyBytes += m.cells*5;
      const readIndex = i=>integer(i,0,m.cells-1);
      // No mutable internal byte view escapes. Renderer can read cells or copy
      // into its OWN preallocated buffers; pan/zoom and rendering stay outside.
      const body = Object.freeze({cells:m.cells,
        probabilityAt:i=>b.data.getFloat32(readIndex(i)*4,true),knownAt:i=>b.known[readIndex(i)],
        forEachKnown:callback=>{ for(let i=0;i<m.cells;++i) if(b.known[i]) callback(i,b.data.getFloat32(i*4,true)); },
        copyBytesInto:(prob,known)=>{
          if (!(prob instanceof Uint8Array)||!(known instanceof Uint8Array)||prob.length!==m.cells*4||known.length!==m.cells) fail('caller_buffer_sizes');
          copyBytes.call(prob,b.prob); copyBytes.call(known,b.known);
        }});
      submaps.push(Object.freeze({...m,body}));
    }
    const view = Object.freeze({publisherEpoch:this.#epoch,generation:this.#generation,sequence:stage.sequence,
      transformRevision:stage.revision,mapFrame:this.#frame,bodyBytes,submaps:Object.freeze(submaps),
      rawManifest:stage.raw,valuesSemantics:'native_float32_from_quantized_probability_grid',
      globalPosteriorFused:false,liveSensorAdmissionQualified:false,runtimeAuthority:false,physicalOutputs:false});
    this.#cache = cache; this.#visible = view; this.#stage = null; this.#stale = false; this.#reason = null;
    ++this.#counts.commits; return this.#event('commit');
  }
}
