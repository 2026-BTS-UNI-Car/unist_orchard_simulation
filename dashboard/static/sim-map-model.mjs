// World coordinates stay metric (+X right, +Y up); screen Y is inverted once.
function gridBounds(grid) {
  if (!Number.isInteger(grid?.width) || grid.width <= 0 || !Number.isInteger(grid?.height) || grid.height <= 0 ||
      !Number.isFinite(grid.resolution) || grid.resolution <= 0 ||
      !Number.isFinite(grid.origin?.x) || !Number.isFinite(grid.origin?.y)) return undefined;
  const bounds = {minX:grid.origin.x, minY:grid.origin.y,
    maxX:grid.origin.x + grid.width * grid.resolution, maxY:grid.origin.y + grid.height * grid.resolution};
  return Object.values(bounds).every(Number.isFinite) ? bounds : undefined;
}
export function occupiedBounds(grid, value = grid?.occupied_bounds) {
  const extent = gridBounds(grid);
  if (!extent) return undefined;
  if (value === null) return null; // Explicitly no occupied cells, not missing metadata.
  if (!value || !['minX','minY','maxX','maxY'].every(key => Number.isFinite(value[key])) ||
      value.minX >= value.maxX || value.minY >= value.maxY ||
      value.minX < extent.minX - 1e-6 || value.minY < extent.minY - 1e-6 ||
      value.maxX > extent.maxX + 1e-6 || value.maxY > extent.maxY + 1e-6) return undefined;
  return {minX:value.minX, minY:value.minY, maxX:value.maxX, maxY:value.maxY};
}
export function occupiedCellsFromPixels(grid, rgba) {
  // Historical sim_mapping.py PNGs use this exact opaque grayscale palette.
  // Never infer occupancy from an arbitrary dark image or an unknown row order.
  if (!gridBounds(grid) || grid.encoding !== 'png' || grid.row_order !== 'min_y_first' ||
      grid.width * grid.height > 1048576 || rgba?.length !== grid.width * grid.height * 4) return undefined;
  let xmin = grid.width, ymin = grid.height, xmax = -1, ymax = -1;
  const indices = [];
  for (let index = 0; index < rgba.length; index += 4) {
    const value = rgba[index];
    if ((value !== 25 && value !== 127 && value !== 245) || rgba[index+1] !== value || rgba[index+2] !== value || rgba[index+3] !== 255) return undefined;
    if (value !== 25) continue;
    const cell = index / 4, x = cell % grid.width, y = Math.floor(cell / grid.width);
    indices.push(cell);
    xmin = Math.min(xmin, x); xmax = Math.max(xmax, x);
    ymin = Math.min(ymin, y); ymax = Math.max(ymax, y);
  }
  return {indices:Uint32Array.from(indices), bounds:xmax < 0 ? null : {
    minX:grid.origin.x + xmin * grid.resolution, minY:grid.origin.y + ymin * grid.resolution,
    maxX:grid.origin.x + (xmax+1) * grid.resolution, maxY:grid.origin.y + (ymax+1) * grid.resolution}};
}
export function occupiedBoundsFromPixels(grid, rgba) {
  return occupiedCellsFromPixels(grid, rgba)?.bounds;
}
export function worldBounds(snapshot, pixelBounds) {
  const grid = snapshot?.grid, extent = gridBounds(grid), pose = snapshot?.pose;
  const point = Number.isFinite(pose?.x) && Number.isFinite(pose?.y) ? pose : null;
  let hits = occupiedBounds(grid);
  if (hits === undefined && pixelBounds !== undefined) hits = occupiedBounds(grid, pixelBounds);
  // Unknown legacy encodings retain the full raster instead of inventing hits.
  // Known empty maps follow the rover. Historic paths do not drive automatic fit.
  if (hits === undefined && extent) hits = extent;
  if (!hits) {
    const x = point?.x ?? (extent ? (extent.minX+extent.maxX)/2 : 0);
    const y = point?.y ?? (extent ? (extent.minY+extent.maxY)/2 : 0);
    return {minX:x-2, minY:y-2, maxX:x+2, maxY:y+2};
  }
  const margin = 1;
  return {minX:Math.min(hits.minX, point?.x ?? Infinity)-margin,
    minY:Math.min(hits.minY, point?.y ?? Infinity)-margin,
    maxX:Math.max(hits.maxX, point?.x ?? -Infinity)+margin,
    maxY:Math.max(hits.maxY, point?.y ?? -Infinity)+margin};
}
export function nearbyWorldBounds(snapshot, cells, radius = 12) {
  const grid = snapshot?.grid, pose = snapshot?.pose;
  if (!Number.isFinite(pose?.x) || !Number.isFinite(pose?.y)) return worldBounds(snapshot, cells?.bounds);
  if (!gridBounds(grid)) return {minX:pose.x-2,minY:pose.y-2,maxX:pose.x+2,maxY:pose.y+2};
  if (!cells) {
    // Unknown palette/encoding: show a labelled local window without claiming
    // we identified objects. The complete raster is still available via Full.
    return {minX:pose.x-radius,minY:pose.y-radius,maxX:pose.x+radius,maxY:pose.y+radius};
  }
  let xmin = Infinity, ymin = Infinity, xmax = -Infinity, ymax = -Infinity;
  for (const cell of cells.indices) {
    const column = cell % grid.width, row = Math.floor(cell / grid.width);
    const x = grid.origin.x + (column + .5) * grid.resolution;
    const y = grid.origin.y + (row + .5) * grid.resolution;
    if ((x-pose.x)**2 + (y-pose.y)**2 > radius**2) continue;
    xmin = Math.min(xmin,column); ymin = Math.min(ymin,row); xmax = Math.max(xmax,column); ymax = Math.max(ymax,row);
  }
  if (!Number.isFinite(xmin)) return {minX:pose.x-2,minY:pose.y-2,maxX:pose.x+2,maxY:pose.y+2};
  return {minX:Math.min(grid.origin.x+xmin*grid.resolution,pose.x)-1,
    minY:Math.min(grid.origin.y+ymin*grid.resolution,pose.y)-1,
    maxX:Math.max(grid.origin.x+(xmax+1)*grid.resolution,pose.x)+1,
    maxY:Math.max(grid.origin.y+(ymax+1)*grid.resolution,pose.y)+1};
}
export function fullWorldBounds(snapshot) {
  const extent = gridBounds(snapshot?.grid);
  return extent ? worldBounds({...snapshot,grid:{...snapshot.grid,occupied_bounds:extent}}) : worldBounds(snapshot);
}
export function fittedView(bounds, width, height, padding=28) {
  const spanX = Math.max(4, bounds.maxX - bounds.minX), spanY = Math.max(4, bounds.maxY - bounds.minY);
  return {x:(bounds.minX+bounds.maxX)/2, y:(bounds.minY+bounds.maxY)/2,
    scale:Math.max(.01, Math.min((width-padding*2)/spanX, (height-padding*2)/spanY))};
}
export function worldToScreen(point, view, width, height) {
  return {x:width/2+(point.x-view.x)*view.scale, y:height/2-(point.y-view.y)*view.scale};
}
export function pathOpacity(stamp, now) {
  return Math.max(.22, Math.min(1, .22 + .78 * Math.exp(-Math.max(0, now-stamp)/45)));
}
export function archivePath(path, seconds, startStamp) {
  const origin = Number.isFinite(startStamp) ? startStamp : Number(path?.[0]?.stamp || 0);
  return (path || []).filter(point => (Number.isFinite(point.t_s) ? point.t_s : Number(point.stamp)-origin) <= seconds + .001);
}
export function virtualGpsPoint(sample) {
  if (!sample?.valid) return null;
  if (Number.isFinite(sample.local_x) && Number.isFinite(sample.local_y)) return {x:sample.local_x,y:sample.local_y,yaw:Number(sample.yaw || 0),stamp:Number(sample.stamp ?? sample.received_at ?? 0)};
  const latitude = sample.origin_latitude ?? sample.origin?.latitude ?? 37, longitude = sample.origin_longitude ?? sample.origin?.longitude ?? 127;
  if (!Number.isFinite(sample.latitude) || !Number.isFinite(sample.longitude)) return null;
  return {x:(sample.longitude-longitude)*111320*Math.cos(latitude*Math.PI/180),y:(sample.latitude-latitude)*111320,yaw:(90-Number(sample.course_deg || 90))*Math.PI/180,stamp:Number(sample.received_at || 0)};
}

export function orchardLayout(world) {
  // GPS local_x/local_y use world axes translated by the actual spawn position.
  const spawn=world?.spawn, rows=world?.row_y, limits=world?.row_x_range;
  if (!Array.isArray(spawn)||!Array.isArray(rows)||!rows.length||!Array.isArray(limits)||limits.length!==2) return null;
  if (![spawn[0],spawn[1],...rows,...limits].every(Number.isFinite)) return null;
  const localRows=rows.map(y=>y-spawn[1]);
  const minX=Math.min(...limits)-spawn[0],maxX=Math.max(...limits)-spawn[0];
  return {rows:localRows,minX,maxX,bounds:{minX:Math.min(-3,minX-6),maxX:Math.max(3,maxX+6),
    minY:Math.min(-3,...localRows.map(y=>y-3)),maxY:Math.max(3,...localRows.map(y=>y+3))}};
}
