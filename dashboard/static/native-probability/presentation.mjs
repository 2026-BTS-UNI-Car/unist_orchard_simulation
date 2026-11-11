// SPDX-License-Identifier: Apache-2.0
import {imageToMap,probabilityColor} from './display_geometry.mjs';
import {nativeCellMap} from './native_wire_receiver.mjs';
import {yieldTask} from './transport.mjs';
export function displayAffine(e) {
  const [o0,o1]=e.crop_offset_i0_i1,[mx,my]=e.native_max_xy,r=e.resolution;
  // Screen affine uses double corners. Cursor uses exact native f32 centres.
  return imageToMap({resolution_m:r,max_xy_local_m:[mx-r*o1,my-r*o0],
    serialized_submap_to_local:{translation_xyz_m:e.T_local_submap.translation,quaternion_xyzw:e.T_local_submap.q_xyzw},
    optimized_submap_to_global:{translation_xyz_m:e.T_map_submap.translation,quaternion_xyzw:e.T_map_submap.q_xyzw}});
}
function projectedBounds(entry,indexBounds) {
  const [min0,min1,max0,max1]=indexBounds;if(max0<0)return null;let out=null;
  for(const [i0,i1]of [[min0,min1],[min0,max1],[max0,min1],[max0,max1]]){
    const [x,y]=nativeCellMap(entry,i0,i1);
    out=out?[Math.min(out[0],x),Math.min(out[1],y),Math.max(out[2],x),Math.max(out[3],y)]:[x,y,x,y];
  }
  return out;
}
function unionBounds(a,b) {return !b?a:!a?b:[Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.max(a[2],b[2]),Math.max(a[3],b[3])];}
export async function preparePresentation(view,active=()=>{},palette='gray',yieldFn=yieldTask) {
  if(!['gray','positive'].includes(palette))throw Error('palette_mode');
  let cells=0;for(const e of view.submaps){const [n0,n1]=e.crop_dimensions_n0_n1;
    if(n0>16384||n1>16384||(cells+=e.cells)>4194304)throw Error('render_cell_or_canvas_cap');displayAffine(e);}
  const models=[];let positiveBounds=null,knownBounds=null,fullBounds=null;
  for(const e of view.submaps){active();const {body,...metadata}=e,[n0,n1]=e.crop_dimensions_n0_n1;
    const rgba=new Uint8ClampedArray(e.cells*4);let min0=n0,min1=n1,max0=-1,max1=-1;
    let pmin0=n0,pmin1=n1,pmax0=-1,pmax1=-1;
    for(let start=0;start<e.cells;start+=65536){active();const end=Math.min(e.cells,start+65536);
      for(let i=start;i<end;++i)if(body.knownAt(i)){
        const probability=body.probabilityAt(i),color=probabilityColor(probability,palette);rgba.set(color,i*4);
        const i0=i%n0,i1=Math.floor(i/n0);min0=Math.min(min0,i0);max0=Math.max(max0,i0);min1=Math.min(min1,i1);max1=Math.max(max1,i1);
        // Display focus only: keep every original known cell and color unchanged.
        if(probability>.5){pmin0=Math.min(pmin0,i0);pmax0=Math.max(pmax0,i0);pmin1=Math.min(pmin1,i1);pmax1=Math.max(pmax1,i1);}
      }
      await yieldFn();active();
    }
    // Rectangular index support is projected through the exact full-SE3 native
    // center function; it is not a yaw-only/world-axis resampling operation.
    positiveBounds=unionBounds(positiveBounds,projectedBounds(e,[pmin0,pmin1,pmax0,pmax1]));
    knownBounds=unionBounds(knownBounds,projectedBounds(e,[min0,min1,max0,max1]));
    fullBounds=unionBounds(fullBounds,projectedBounds(e,[0,0,n0-1,n1-1]));
    models.push({metadata,affine:displayAffine(e),rgba:rgba.buffer});
  }
  const bounds=positiveBounds??knownBounds??fullBounds;
  const focus_kind=positiveBounds?'positive_evidence':knownBounds?'known_cells':fullBounds?'full_grid':'none';
  return {models,bounds,positiveBounds,knownBounds,fullBounds,focus_kind,palette,scope:'independent submaps; manifest-order screen overlay, not posterior fusion'};
}
