import {visibleSegments} from './archive-segments.mjs';
// Immutable ahead-of-time maps are applied only up to the media playback clock.
export class ArchiveMapBuffer {
  constructor(window){
    Object.assign(this,window);
    if(window.cell_codec==='xor2-v1')this.events=expandCellEvents(window);
  }
  contains(time){return time>=this.start_s && time<=this.end_s;}
  at(time,version=-1,epoch=null){
    if(!this.contains(time))return null;
    const frames=this.events.filter(e=>e.frame_t_s<=time);
    const latest=frames.at(-1)||this.seed;
    const incremental=epoch===this.seed.tile_epoch && version>=this.seed.tile_version && version<=latest.tile_version;
    let tiles=new Map(),path=[],reset=!incremental,packet=this.seed;
    if(!incremental){for(const t of this.seed.tiles||[])tiles.set(`${t.x},${t.y}`,t);path=[...(this.seed.path||[])];}
    for(const row of frames){
      if(incremental && row.tile_version<=version)continue;
      if(row.tiles_reset){tiles.clear();path=[];reset=true;}
      for(const tile of row.tiles||[])tiles.set(`${tile.x},${tile.y}`,tile);
      path.push(...(row.path||[]));packet=row;
    }
    return {...latest,segments:latest.segments??this.seed.segments,
      segment_catalog:visibleSegments(this.segment_catalog || this.seed.segment_catalog,time),
      needs_regeneration:latest.needs_regeneration??this.seed.needs_regeneration,
      tiles:[...tiles.values()],path,tiles_reset:reset,path_append:!reset};
  }
}

// Expand a seeded XOR chain once when downloaded, before media-clock playback.
// Independent windows and backward seeks never depend on earlier HTTP replies.
import {decodeTileCells} from './grid-cell-codec.mjs';
function packTile(tile){
  const states=decodeTileCells(tile),bytes=new Uint8Array(1024);
  for(let i=0;i<bytes.length;i++)for(let j=0;j<4;j++)bytes[i]|=states[i*4+j]<<(j*2);
  return bytes;
}
function expandCellEvents(window){
  const previous=new Map((window.seed.tiles||[]).map(t=>[`${t.x},${t.y}`,packTile(t)]));
  return window.events.map(event=>{
    if(event.tiles_reset)previous.clear();
    const tiles=(event.tiles||[]).map(tile=>{
      const key=`${tile.x},${tile.y}`;
      if(tile.encoding!=='xor2-v1'){previous.set(key,packTile(tile));return tile;}
      const base=previous.get(key);
      if(!base||typeof tile.cells!=='string'||tile.cells.length!==1368||!/^[A-Za-z0-9+/]*==$/.test(tile.cells))throw new Error('Invalid cell delta base');
      const raw=atob(tile.cells);
      if(raw.length!==1024)throw new Error('Invalid cell delta length');
      const next=base.map((byte,i)=>byte^raw.charCodeAt(i));previous.set(key,next);
      return {...tile,encoding:'packed2-v1',cells:btoa(String.fromCharCode(...next))};
    });
    return {...event,tiles};
  });
}
