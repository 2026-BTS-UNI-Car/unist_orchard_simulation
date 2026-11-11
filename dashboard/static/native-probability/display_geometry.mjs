// Pure display geometry. No ROS, localization, thresholding or data mutation.
export function finite(value) {
  if (typeof value !== 'number' || !Number.isFinite(value)) throw Error('finite_number_required');
  return value;
}
export function pose(value) {
  const t=value?.translation_xyz_m, q=value?.quaternion_xyzw;
  if (!Array.isArray(t)||t.length!==3||!Array.isArray(q)||q.length!==4) throw Error('pose_shape');
  [...t,...q].forEach(finite);
  if (Math.abs(Math.hypot(...q)-1)>1e-6) throw Error('unit_quaternion');
  return {t:[...t],q:[...q]};
}
export function rotate(q,v) {
  const [x,y,z,w]=q, [vx,vy,vz]=v;
  const a=2*(y*vz-z*vy), b=2*(z*vx-x*vz), c=2*(x*vy-y*vx);
  return [vx+w*a+y*c-z*b,vy+w*b+z*a-x*c,vz+w*c+x*b-y*a];
}
export function multiply(a,b) {
  const [x,y,z,w]=a,[X,Y,Z,W]=b;
  return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z];
}
export function mapFromLocal(submap) {
  const a=pose(submap.optimized_submap_to_global), b=pose(submap.serialized_submap_to_local);
  const qi=[-b.q[0],-b.q[1],-b.q[2],b.q[3]];
  const q=multiply(a.q,qi), shift=rotate(q,b.t);
  return {q,t:a.t.map((x,i)=>x-shift[i])};
}
export function transform(p,v) {
  return rotate(p.q,v).map((x,i)=>x+p.t[i]);
}
export function cellCenter(submap,i0,i1) {
  const [nx,ny]=submap.dimensions_i0_i1;
  if (!Number.isInteger(i0)||!Number.isInteger(i1)||i0<0||i1<0||i0>=nx||i1>=ny) throw Error('cell_index');
  const r=finite(submap.resolution_m), [mx,my]=submap.max_xy_local_m;
  return transform(mapFromLocal(submap),[Math.fround(mx-r*(i1+.5)),Math.fround(my-r*(i0+.5)),0]);
}
export function imageToMap(submap) {
  // Display affine uses exact double corner geometry. Native f32 centers are
  // reported separately by cellCenter; tiny f32 rounding is not an affine grid.
  const p=mapFromLocal(submap), r=submap.resolution_m;
  const u=rotate(p.q,[0,-r,0]), v=rotate(p.q,[-r,0,0]);
  const origin=transform(p,[...submap.max_xy_local_m,0]);
  const out=[u[0],u[1],v[0],v[1],origin[0],origin[1]];
  if (Math.abs(out[0]*out[3]-out[1]*out[2])<1e-14) throw Error('edge_on_grid_projection_unsupported');
  return out;
}
export function inversePoint(a,x,y) {
  const det=a[0]*a[3]-a[1]*a[2];
  if (!Number.isFinite(det)||Math.abs(det)<1e-14) throw Error('singular_affine');
  x-=a[4];y-=a[5];
  return [(a[3]*x-a[2]*y)/det,(-a[1]*x+a[0]*y)/det];
}
export function probabilityColor(p,mode='gray') {
  if (p===-1) return [0,0,0,0];
  if (!Number.isFinite(p)||p<0||p>1) throw Error('native_probability_range');
  if (mode==='gray') { const g=Math.round(255*(1-p));return [g,g,g,255]; }
  if (mode!=='positive') throw Error('palette_mode');
  // Continuous display gradient; 0.5 is neutral evidence, no occupancy decision.
  const k=Math.max(0,2*p-1), g=Math.round(225-170*k);
  return [Math.round(225+25*k),g,Math.round(225-180*k),255];
}
export function worldAt(view,x,y) { return [(x-view.tx)/view.scale,(view.ty-y)/view.scale]; }
export function pan(view,dx,dy) { return {...view,tx:view.tx+dx,ty:view.ty+dy}; }
export function zoomAt(view,x,y,factor) {
  const world=worldAt(view,x,y),scale=Math.max(.02,Math.min(2000,view.scale*factor));
  return {scale,tx:x-scale*world[0],ty:y+scale*world[1]};
}
export function fit(bounds,width,height) {
  const [xmin,ymin,xmax,ymax]=bounds;
  const scale=Math.max(.02,Math.min(2000,Math.min((width-60)/Math.max(.01,xmax-xmin),(height-60)/Math.max(.01,ymax-ymin))));
  return {scale,tx:width/2-scale*(xmin+xmax)/2,ty:height/2+scale*(ymin+ymax)/2};
}
