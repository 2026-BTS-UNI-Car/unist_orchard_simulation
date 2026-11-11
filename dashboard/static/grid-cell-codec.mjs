// Wire cells are numeric occupancy states, never a transmitted raster image.
// Index i has world coordinates ((tile.x*64+i%64)*resolution,
// (tile.y*64+floor(i/64))*resolution). Row zero is the minimum world Y.
export function decodeTileCells(tile, size=64) {
  if (size!==64 || !Number.isSafeInteger(tile.x) || !Number.isSafeInteger(tile.y) ||
      Math.abs(tile.x)>2147483648 || Math.abs(tile.y)>2147483648) throw new Error('Invalid tile coordinates');
  const output=new Uint8Array(size*size);
  if(tile.encoding==='rle2-v1') {
    if(!Array.isArray(tile.cells) || tile.cells.length%2 || tile.cells.length>output.length*2) throw new Error('Invalid cell runs');
    let cursor=0;
    for(let i=0;i<tile.cells.length;i+=2) {
      const state=tile.cells[i],count=tile.cells[i+1];
      if(!Number.isInteger(state) || state<0 || state>3 || !Number.isInteger(count) || count<1 || cursor+count>output.length) throw new Error('Invalid cell run');
      output.fill(state,cursor,cursor+count);cursor+=count;
    }
    if(cursor!==output.length) throw new Error('Incomplete cell runs');
  } else if(tile.encoding==='packed2-v1') {
    if(typeof tile.cells!=='string' || tile.cells.length!==1368 || !/^[A-Za-z0-9+/]*==$/.test(tile.cells)) throw new Error('Invalid packed cells');
    const bytes=atob(tile.cells);
    if(bytes.length!==output.length/4) throw new Error('Incomplete packed cells');
    for(let i=0;i<bytes.length;i++) {
      const byte=bytes.charCodeAt(i);
      for(let j=0;j<4;j++) output[i*4+j]=(byte>>(j*2))&3;
    }
  } else throw new Error('Unsupported cell encoding');
  return output;
}

export function cellPixels(states) {
  const pixels=new Uint8ClampedArray(states.length*4),tones=[127,245,25,127];
  for(let i=0;i<states.length;i++) {
    const p=i*4,gray=tones[states[i]];
    pixels[p]=gray;pixels[p+1]=gray;pixels[p+2]=gray;pixels[p+3]=255;
  }
  return pixels;
}

export function tileOccupiedIndices(states) {
  const indices=[];
  for(let i=0;i<states.length;i++) if(states[i]===2) indices.push(i);
  return Uint16Array.from(indices);
}

export function nearbyTileBounds(snapshot, tiles, radius=12) {
  const pose=snapshot.pose,resolution=snapshot.grid?.resolution;
  if(!Number.isFinite(pose?.x) || !Number.isFinite(pose?.y) || !(resolution>0)) return null;
  const extent=64*resolution;
  let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
  for(const tile of tiles.values()) {
    const x0=tile.x*extent,y0=tile.y*extent;
    if(x0>pose.x+radius || x0+extent<pose.x-radius || y0>pose.y+radius || y0+extent<pose.y-radius) continue;
    for(const i of tile.occupied || []) {
      const x=x0+(i%64+.5)*resolution,y=y0+(Math.floor(i/64)+.5)*resolution;
      if((x-pose.x)**2+(y-pose.y)**2>radius**2) continue;
      minX=Math.min(minX,x-resolution*.5);maxX=Math.max(maxX,x+resolution*.5);
      minY=Math.min(minY,y-resolution*.5);maxY=Math.max(maxY,y+resolution*.5);
    }
  }
  if(!Number.isFinite(minX)) return {minX:pose.x-2,minY:pose.y-2,maxX:pose.x+2,maxY:pose.y+2};
  return {minX:Math.min(minX,pose.x)-1,minY:Math.min(minY,pose.y)-1,
          maxX:Math.max(maxX,pose.x)+1,maxY:Math.max(maxY,pose.y)+1};
}
