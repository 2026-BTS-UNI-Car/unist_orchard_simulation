import {worldBounds, occupiedCellsFromPixels, nearbyWorldBounds, fullWorldBounds, fittedView, worldToScreen, pathOpacity, archivePath, virtualGpsPoint, orchardLayout} from './sim-map-model.mjs';
import {decodeTileCells, cellPixels, tileOccupiedIndices, nearbyTileBounds} from './grid-cell-codec.mjs';

const $ = id => document.getElementById(id);
export class MetricMap {
  constructor(id, schematic=false, headingOffset=0) {
    this.canvas = $(id); this.schematic = schematic; this.snapshot = {}; this.bitmap = null;
    this.headingOffset = headingOffset;
    this.view = null; this.bounds = null; this.auto = true; this.until = Infinity; this.imageKey = '';
    this.imageData = ''; this.pendingImage = null; this.requestFullImage = false; this.occupiedCells = undefined; this.fitMode = 'nearby';
    this.canvas.addEventListener('wheel', event => {event.preventDefault(); this.zoom(event.deltaY < 0 ? 'in' : 'out');}, {passive:false});
    let drag = null;
    this.canvas.addEventListener('pointerdown', event => {drag={x:event.clientX,y:event.clientY}; this.canvas.setPointerCapture(event.pointerId);});
    this.canvas.addEventListener('pointermove', event => {
      if (!drag || !this.view) return;
      this.view.x -= (event.clientX-drag.x)/this.view.scale; this.view.y += (event.clientY-drag.y)/this.view.scale;
      drag={x:event.clientX,y:event.clientY}; this.auto=false; this.draw();
    });
    for (const kind of ['pointerup','pointercancel','lostpointercapture']) this.canvas.addEventListener(kind, () => {drag=null;});
    this.resize = new ResizeObserver(() => this.draw()); this.resize.observe(this.canvas);
  }
  get imageVersion() { return this.requestFullImage ? undefined : this.snapshot.version; }
  async setTiles(snapshot) {
    this.pendingImage = null;
    const ticket = {}; this.pendingTiles = ticket;
    const epoch = snapshot.tile_epoch || snapshot.map_epoch;
    const decoded = [];
    for (const tile of snapshot.tiles || []) {
      if (tile.encoding) {
        const states=decodeTileCells(tile,snapshot.tile_size || 64);
        const image=document.createElement('canvas');image.width=64;image.height=64;
        const context=image.getContext('2d'),pixels=context.createImageData(64,64);
        pixels.data.set(cellPixels(states));context.putImageData(pixels,0,0);
        decoded.push({...tile,image,occupied:tileOccupiedIndices(states)});
      } else {
        // Read-only compatibility for earlier generated archives. New source
        // packets contain cells and never reach this image decoder.
        const image=await new Promise((resolve,reject)=>{
          const value=new Image();value.onload=()=>resolve(value);value.onerror=reject;
          value.src=`data:image/png;base64,${tile.image_base64}`;
        });
        decoded.push({...tile,image});
      }
    }
    if (this.pendingTiles !== ticket) return;
    if (snapshot.tiles_reset || this.tileEpoch !== epoch) this.tileBitmaps = new Map();
    this.tileBitmaps ||= new Map();
    for (const tile of decoded) this.tileBitmaps.set(`${tile.x},${tile.y}`,tile);
    this.tileEpoch = epoch; this.bitmap = null; this.occupiedCells = undefined;
    this.pendingTiles = null;
    const previousPath = this.snapshot.path || [];
    const path = snapshot.path_append && !snapshot.tiles_reset ? [...previousPath,...(snapshot.path || [])] : snapshot.path;
    this.commit({...snapshot,path,map_epoch:snapshot.view_epoch || snapshot.map_epoch || epoch});
  }
  set(snapshot) {
    if (Array.isArray(snapshot.tiles)) return this.setTiles(snapshot).catch(() => {this.tileEpoch = null;});
    this.pendingTiles = null;
    this.tileBitmaps = null; this.tileEpoch = null;
    snapshot = snapshot || {};
    const grid = snapshot.grid;
    // Empty/reset and schematic views do not wait for a grid decoder. Cancel
    // any earlier load so it cannot bring an old map back after a reset.
    if (this.schematic || !grid || snapshot.scan_count === 0) {
      this.pendingImage = null; this.bitmap = null; this.imageKey = ''; this.imageData = '';
      this.occupiedCells = undefined;
      this.commit(snapshot);
      return;
    }
    const key = JSON.stringify([grid.width, grid.height, grid.resolution,
      grid.origin?.x, grid.origin?.y, grid.encoding, grid.row_order]);
    const data = grid.image_base64;
    if (data === undefined) {
      // A PNG-less response only updates the generation whose pixels we have.
      // While that generation decodes, retain its newest pose/paused metadata.
      if (this.pendingImage) {
        if (this.pendingImage.key === key && this.pendingImage.snapshot.version === snapshot.version) this.pendingImage.snapshot = snapshot;
      } else if (!this.requestFullImage && this.bitmap && this.imageKey === key && this.snapshot.version === snapshot.version) {
        this.commit(snapshot);
      } else this.requestFullImage = true;
      return;
    }
    if (this.bitmap && this.imageKey === key && this.imageData === data) {
      this.pendingImage = null;
      this.commit(snapshot);
      return;
    }
    if (this.pendingImage?.key === key && this.pendingImage.data === data) {
      this.pendingImage.snapshot = snapshot;
      return;
    }
    const image = new Image();
    const pending = {key, data, image, snapshot};
    this.pendingImage = pending; this.requestFullImage = true;
    image.onload = () => {
      if (this.pendingImage !== pending) return;
      this.pendingImage = null; this.bitmap = image;
      this.imageKey = key; this.imageData = data;
      this.occupiedCells = this.readOccupiedCells(image, pending.snapshot.grid);
      this.commit(pending.snapshot);
    };
    image.onerror = () => {
      // Keep the last complete map; polling uses its version to retry a PNG.
      if (this.pendingImage === pending) this.pendingImage = null;
    };
    image.src = `data:image/png;base64,${data}`;
  }
  readOccupiedCells(image, grid) {
    // Cache bounded occupied cell indices once per accepted PNG, never on each
    // pose update, resize or drag. Global bounds cannot answer a local query.
    if (grid.encoding !== 'png' || grid.row_order !== 'min_y_first' ||
        !Number.isInteger(grid.width) || !Number.isInteger(grid.height) ||
        grid.width <= 0 || grid.height <= 0 || grid.width * grid.height > 1048576 ||
        image.naturalWidth !== grid.width || image.naturalHeight !== grid.height) return undefined;
    try {
      const canvas = document.createElement('canvas'); canvas.width = grid.width; canvas.height = grid.height;
      const ctx = canvas.getContext('2d', {willReadFrequently:true});
      ctx.drawImage(image, 0, 0);
      return occupiedCellsFromPixels(grid, ctx.getImageData(0, 0, grid.width, grid.height).data);
    } catch { return undefined; } // Unknown formats use a local window, not invented hits.
  }
  commit(snapshot) {
    // Grid pixels, geometry, pose and path become visible in one draw. Bounds
    // and automatic zoom change only when that same complete view is accepted.
    const previous = this.snapshot;
    this.beforeCommit?.(previous, snapshot);
    const historicalSeek = snapshot.preserve_view_on_seek === true &&
      snapshot.map_epoch && snapshot.map_epoch === previous.map_epoch;
    const frameChanged = snapshot.coordinate_frame && previous.coordinate_frame &&
      snapshot.coordinate_frame !== previous.coordinate_frame;
    if (historicalSeek && frameChanged && this.view && !this.auto) {
      // Isolated archive segments have unrelated origins. Keep the user's
      // pixels/metre, but recenter once because the old pan coordinates cannot
      // identify a location in the new local frame.
      this.view = {...this.view,x:Number(snapshot.pose?.x||0),y:Number(snapshot.pose?.y||0)};
    }
    const newMap = !this.schematic && ((!historicalSeek && !snapshot.grid && !!previous.grid) ||
      (!historicalSeek && snapshot.scan_count === 0 && previous.scan_count > 0) ||
      (snapshot.map_epoch && previous.map_epoch && snapshot.map_epoch !== previous.map_epoch) ||
      (!historicalSeek && !snapshot.view_epoch && Number.isFinite(snapshot.version) && Number.isFinite(previous.version) && snapshot.version < previous.version) ||
      (!historicalSeek && !snapshot.view_epoch && frameChanged));
    if (newMap) {this.view = null; this.auto = true; this.fitMode = 'nearby';}
    this.snapshot = snapshot; this.requestFullImage = false;
    this.layout = this.schematic ? orchardLayout(this.snapshot.world) : null;
    this.bounds = this.fitBounds();
    this.draw();
  }
  fitBounds(pose = this.snapshot.pose) {
    const snapshot = {...this.snapshot, pose};
    if (!this.schematic && this.fitMode==='nearby' && snapshot.grid?.encoding==='tiles-cells-v1' && this.tileBitmaps) {
      const local=nearbyTileBounds(snapshot,this.tileBitmaps);
      if(local) return local;
    }
    const bounds = this.layout ? {...this.layout.bounds} : this.schematic ? worldBounds(snapshot) :
      this.fitMode === 'full' ? fullWorldBounds(snapshot) : nearbyWorldBounds(snapshot, this.occupiedCells);
    if(this.layout && Number.isFinite(pose?.x) && Number.isFinite(pose?.y)){
      bounds.minX=Math.min(bounds.minX,pose.x-2);bounds.maxX=Math.max(bounds.maxX,pose.x+2);
      bounds.minY=Math.min(bounds.minY,pose.y-2);bounds.maxY=Math.max(bounds.maxY,pose.y+2);
    }
    return bounds;
  }
  zoom(direction) {
    if(direction==='fit'||direction==='full'){this.auto=true;this.view=null;this.fitMode=direction==='full'?'full':'nearby';}
    else if(this.view){this.auto=false;this.view.scale=Math.max(.5,Math.min(500,this.view.scale*(direction==='in'?1.35:1/1.35)));}
    this.draw();
  }
  draw() {
    const canvas=this.canvas, rect=canvas.getBoundingClientRect();
    if(rect.width<1 || rect.height<1) return;
    const width=rect.width,height=rect.height,dpr=Math.min(devicePixelRatio||1,1.5);
    if(canvas.width!==Math.round(width*dpr)||canvas.height!==Math.round(height*dpr)){canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);}
    const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.fillStyle='#14202b';ctx.fillRect(0,0,width,height);
    const path=Number.isFinite(this.until)?archivePath(this.snapshot.path,this.until,this.snapshot.recording_start_stamp):this.snapshot.path||[];
    const pose=Number.isFinite(this.until)?path.at(-1)??null:this.snapshot.pose;
    // Manual center and pixels/metre remain unchanged as the raster grows or
    // the canvas resizes. Only Fit (or a new map) returns to automatic framing.
    if(!this.view||this.auto){this.bounds=this.fitBounds(pose);this.view=fittedView(this.bounds,width,height);}
    const xy=point=>worldToScreen(point,this.view,width,height), scale=this.view.scale;
    const grid=this.snapshot.grid;
    if(this.schematic) this.drawOrchard(ctx,xy);
    else if(grid && this.tileBitmaps){
      const extent = (this.snapshot.tile_size || 64)*grid.resolution;
      ctx.imageSmoothingEnabled = false;
      for (const tile of this.tileBitmaps.values()) {
        const origin=xy({x:tile.x*extent,y:tile.y*extent});
        if (origin.x>width || origin.x+extent*scale<0 || origin.y<0 || origin.y-extent*scale>height) continue;
        ctx.save();ctx.translate(origin.x,origin.y);ctx.scale(1,-1);
        ctx.drawImage(tile.image,0,0,extent*scale,extent*scale);ctx.restore();
      }
    }
    else if(grid&&this.bitmap){
      const origin=xy(grid.origin);
      ctx.save();ctx.translate(origin.x,origin.y);ctx.scale(1,-1);ctx.imageSmoothingEnabled=false;
      ctx.drawImage(this.bitmap,0,0,grid.width*grid.resolution*scale,grid.height*grid.resolution*scale);ctx.restore();
    }
    const now=Number.isFinite(this.until)?this.until:Number(this.snapshot.pose?.stamp||this.snapshot.stamp||path.at(-1)?.stamp||0);
    ctx.lineCap='round';ctx.lineJoin='round';ctx.lineWidth=2.5;
    for(let i=1;i<path.length;i++){
      const a=xy(path[i-1]),b=xy(path[i]);
      const ageStamp=Number.isFinite(this.until)?Number(path[i].t_s??(path[i].stamp-(this.snapshot.recording_start_stamp||0))):Number(path[i].stamp);
      ctx.strokeStyle=`rgba(240,50,67,${pathOpacity(Number.isFinite(ageStamp)?ageStamp:now,now)})`;
      ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();
    }
    if(pose&&Number.isFinite(pose.x)&&Number.isFinite(pose.y)){
      const p=xy(pose);ctx.save();ctx.translate(p.x,p.y);ctx.rotate(-(Number(pose.yaw||0)+this.headingOffset));
      ctx.fillStyle='#ef3447';ctx.strokeStyle='#fff';ctx.lineWidth=1.5;
      ctx.beginPath();ctx.moveTo(11,0);ctx.lineTo(-7,-6);ctx.lineTo(-4,0);ctx.lineTo(-7,6);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
    }
    if(!grid&&!this.schematic){ctx.fillStyle='#b6c5d1';ctx.font='13px sans-serif';ctx.textAlign='center';ctx.fillText('그리기를 시작하면 스캔이 지도에 쌓입니다',width/2,height/2);}
    const meters=scale>40?1:scale>15?2:scale>5?5:10;
    ctx.fillStyle='rgba(10,18,27,.78)';ctx.fillRect(8,height-39,meters*scale+24,31);
    ctx.strokeStyle='#eef5fa';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(20,height-15);ctx.lineTo(20+meters*scale,height-15);ctx.stroke();
    ctx.fillStyle='#eef5fa';ctx.font='11px sans-serif';ctx.textAlign='left';ctx.fillText(`${meters} m`,20,height-23);
    this.afterDraw?.();
  }
  drawOrchard(ctx,xy) {
    if(!this.layout){ctx.fillStyle='#b6c5d1';ctx.font='12px sans-serif';ctx.textAlign='left';ctx.fillText('가상 과수원 배치 정보 수신 대기',12,24);return;}
    const {bounds,rows,minX,maxX}=this.layout;
    const a=xy({x:bounds.minX,y:bounds.maxY}),b=xy({x:bounds.maxX,y:bounds.minY});
    ctx.fillStyle='#243b31';ctx.fillRect(a.x,a.y,b.x-a.x,b.y-a.y);
    for(const y of rows){
      const left=xy({x:minX,y}),right=xy({x:maxX,y});
      ctx.strokeStyle='#426c47';ctx.lineWidth=Math.max(3,this.view.scale*.7);ctx.lineCap='round';
      ctx.beginPath();ctx.moveTo(left.x,left.y);ctx.lineTo(right.x,right.y);ctx.stroke();
      ctx.strokeStyle='#a8bb7a';ctx.lineWidth=1;
      ctx.beginPath();ctx.moveTo(left.x,left.y);ctx.lineTo(right.x,right.y);ctx.stroke();
    }
    ctx.fillStyle='#e3ebdd';ctx.font='12px sans-serif';ctx.textAlign='left';ctx.fillText('과수원 열 배치도 · 실제 지리 위치 아님',12,24);
  }
}
