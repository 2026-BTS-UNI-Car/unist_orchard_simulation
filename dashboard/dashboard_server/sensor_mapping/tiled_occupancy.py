"""Sparse display-map storage; image size never truncates retained geometry.

The base log-odds/ray model is unchanged. Tiles retain the original resolution;
only the bounded overview PNG is coarsened, with occupied cells winning over
free/unknown pixels. This module has no localization or control authority.
"""
import base64
import math
import numpy as np
from dashboard_server.sensor_mapping.occupancy_map import OccupancyMap, grayscale_png


def _native_touched_tile_keys(tile_coordinates):
    """Same signed lexical keys as packed unique(), with a bounded tile bitmap.

    Unsupported dimensions/coordinates retain the existing selection fallback.
    The outer native path still owns beam, cell-cap and integer validity gates.
    """
    if (not isinstance(tile_coordinates, np.ndarray)
            or tile_coordinates.dtype != np.int64 or tile_coordinates.ndim != 2
            or tile_coordinates.shape[1] != 2 or not len(tile_coordinates)):
        return None
    # Reduce each strided column independently: avoid a scalar row-axis loop.
    low = np.array([tile_coordinates[:, 0].min(), tile_coordinates[:, 1].min()])
    high = np.array([tile_coordinates[:, 0].max(), tile_coordinates[:, 1].max()])
    if int(low.min()) < -2147483648 or int(high.max()) > 2147483647:
        return None
    # Python differences avoid int64 overflow on unsupported direct inputs.
    nx, ny = (int(high[index])-int(low[index])+1 for index in (0, 1))
    if nx > 128 or ny > 128 or nx*ny > 4096:
        return None
    present = np.zeros((nx, ny), dtype=bool)
    present[tile_coordinates[:, 0]-low[0], tile_coordinates[:, 1]-low[1]] = True
    # C order has x tile first, y tile second, identical to packed-key sorting.
    return np.argwhere(present)+low


class TiledOccupancyMap(OccupancyMap):
    TILE = 64

    def __init__(self, resolution=.10, max_cells=16000000, max_span=1024,
                 max_beams=360, laser_x=.18, laser_y=0.,conservative_free=False,endpoint_margin_cells=0):
        super().__init__(resolution, max_cells, max_span, max_beams, laser_x, laser_y,conservative_free,endpoint_margin_cells)
        if not math.isfinite(resolution) or resolution <= 0 or not 16 <= max_span <= 4096:
            raise ValueError('invalid_tiled_map_configuration')
        if not 1 <= max_cells <= 32000000:
            raise ValueError('invalid_tiled_cell_limit')
        self._tiles = {}

    @property
    def cells(self):
        # Diagnostic materialization, never used by the live raster path.
        if self._cells_cache is None:
            self._cells_cache = {}
            for (tx, ty), tile in self._tiles.items():
                ys, xs = np.nonzero(tile != 127)
                self._cells_cache.update({(int(x)+tx*self.TILE, int(y)+ty*self.TILE): int(tile[y,x])
                                          for x,y in zip(xs,ys)})
        return self._cells_cache

    def _update(self, key, delta):
        self._update_raster(np.asarray([key],dtype=np.int64),0,0,delta)

    def _update_raster(self, coordinates, x0, y0, delta):
        if not len(coordinates): return
        native_attempted = False
        if (type(coordinates) is np.ndarray and coordinates.flags.owndata
                and len(coordinates)>=256 and self._cell_count+len(coordinates)<=self.max_cells):
            from .native_raster import UPDATE,coordinate_metadata,update_tiles
            if UPDATE is not None:
                metadata=coordinate_metadata(coordinates,x0,y0)
                if metadata is not None:
                    extent,keys=metadata
                    native_attempted=True
                    count=update_tiles(coordinates,x0,y0,delta,self.conservative_free,self._tiles,keys)
                    if count is not None:
                        self._cell_count+=count
                        self._last_updated_tiles=[tuple(int(v) for v in key) for key in keys]
                        xmin,ymin,xmax,ymax=extent
                        if self.bounds:
                            old=self.bounds
                            xmin,ymin,xmax,ymax=min(old[0],xmin),min(old[1],ymin),max(old[2],xmax),max(old[3],ymax)
                        self.bounds=(xmin,ymin,xmax,ymax)
                        return
        absolute = coordinates + np.array([x0,y0],dtype=np.int64)
        tile_coordinates = absolute//self.TILE
        if (not native_attempted and len(absolute)>=4096 and self._cell_count+len(absolute)<=self.max_cells
                and np.all((tile_coordinates>=-2147483648)&(tile_coordinates<=2147483647))):
            from .native_raster import UPDATE,update_tiles
            if UPDATE is not None:
                keys=_native_touched_tile_keys(tile_coordinates)
                if keys is None:
                    encoded=((tile_coordinates[:,0]+2147483648).astype(np.uint64)<<np.uint64(32)) | (tile_coordinates[:,1]+2147483648).astype(np.uint64)
                    unique=np.unique(encoded)
                    keys=np.column_stack(((unique>>np.uint64(32)).astype(np.int64)-2147483648,
                        (unique&np.uint64(0xffffffff)).astype(np.int64)-2147483648))
                count=update_tiles(coordinates,x0,y0,delta,self.conservative_free,self._tiles,keys)
                if count is not None:
                    self._cell_count+=count
                    self._last_updated_tiles=[tuple(int(v) for v in key) for key in keys]
                    xmin,ymin=int(absolute[:,0].min()),int(absolute[:,1].min())
                    xmax,ymax=int(absolute[:,0].max()),int(absolute[:,1].max())
                    if self.bounds:
                        old=self.bounds
                        xmin,ymin,xmax,ymax=min(old[0],xmin),min(old[1],ymin),max(old[2],xmax),max(old[3],ymax)
                    self.bounds=(xmin,ymin,xmax,ymax)
                    return
        order=None
        # Scalar integer keys avoid structured-row sorting on every free cell.
        # Signed ordering and floor division remain identical across zero.
        if np.all((tile_coordinates >= -2147483648) & (tile_coordinates <= 2147483647)):
            encoded = ((tile_coordinates[:,0]+2147483648).astype(np.uint64)<<np.uint64(32)) | (tile_coordinates[:,1]+2147483648).astype(np.uint64)
            if self._cell_count+len(absolute)<=self.max_cells:
                # Sort tile keys once. The old path sorted them in unique(),
                # materialized an inverse array, then sorted that array again.
                order=np.argsort(encoded,kind='stable');sorted_keys=encoded[order]
                starts=np.r_[0,np.flatnonzero(sorted_keys[1:]!=sorted_keys[:-1])+1,len(order)]
                unique=sorted_keys[starts[:-1]]
            else:
                unique,membership=np.unique(encoded,return_inverse=True)
            keys = np.column_stack(((unique>>np.uint64(32)).astype(np.int64)-2147483648,
                                    (unique & np.uint64(0xffffffff)).astype(np.int64)-2147483648))
        else:
            keys, membership = np.unique(tile_coordinates,axis=0,return_inverse=True)
        self._last_updated_tiles = [tuple(int(v) for v in key) for key in keys]
        if order is not None and len(absolute)>=4096:
            # One bounded vector gather/scatter instead of a Python/NumPy
            # operation sequence for every tile. The admission-at-cap path
            # below stays unchanged. Values and increments are bit-identical.
            membership=np.empty(len(absolute),dtype=np.int64)
            membership[order]=np.repeat(np.arange(len(keys)),np.diff(starts))
            tiles=[]
            for key in self._last_updated_tiles:
                tile=self._tiles.get(key)
                if tile is None:
                    tile=np.full((self.TILE,self.TILE),127,dtype=np.int8)
                    self._tiles[key]=tile
                tiles.append(tile)
            batch=np.stack(tiles).reshape(-1)
            local=absolute%self.TILE
            indices=membership*(self.TILE*self.TILE)+local[:,1]*self.TILE+local[:,0]
            values=batch[indices].astype(np.int16);fresh=values==127
            self._cell_count+=int(fresh.sum());values[fresh]=0
            increments=np.where(values>=9,-1,delta) if self.conservative_free and delta<0 else delta
            batch[indices]=np.clip(values+increments,-40,60)
            for tile,values in zip(tiles,batch.reshape(-1,self.TILE,self.TILE)):tile[:]=values
            xmin,ymin=int(absolute[:,0].min()),int(absolute[:,1].min())
            xmax,ymax=int(absolute[:,0].max()),int(absolute[:,1].max())
            if self.bounds:
                old=self.bounds
                xmin,ymin,xmax,ymax=min(old[0],xmin),min(old[1],ymin),max(old[2],xmax),max(old[3],ymax)
            self.bounds=(xmin,ymin,xmax,ymax)
            return
        # Admission follows the original ray raster's global x/y order even
        # when a scan crosses tile boundaries. Updating existing cells remains
        # possible after reaching the memory cap.
        if self._cell_count+len(absolute)>self.max_cells:
            existing=np.zeros(len(absolute),dtype=bool)
            for group,key in enumerate(keys):
                selected=np.flatnonzero(membership==group);local=absolute[selected]%self.TILE
                tile=self._tiles.get(tuple(int(v) for v in key))
                if tile is not None:existing[selected]=tile[local[:,1],local[:,0]]!=127
            allowed=existing | (np.cumsum(~existing)<=self.max_cells-self._cell_count)
            if not allowed.all():self.limited=True
            absolute,membership=absolute[allowed],membership[allowed]
        if order is None:
            order = np.argsort(membership, kind='stable')
            sorted_groups = membership[order]
            starts = np.searchsorted(sorted_groups, np.arange(len(keys)+1))
        for group, key in enumerate(keys):
            cells = absolute[order[starts[group]:starts[group+1]]]
            if not len(cells):continue
            local = cells % self.TILE
            tile_key = tuple(int(v) for v in key)
            tile = self._tiles.get(tile_key)
            if tile is None:
                if self._cell_count >= self.max_cells:
                    self.limited = True
                    continue
                tile = np.full((self.TILE,self.TILE),127,dtype=np.int8)
                self._tiles[tile_key] = tile
            values = tile[local[:,1],local[:,0]].astype(np.int16)
            fresh = values == 127
            self._cell_count += int(fresh.sum())
            values[fresh] = 0
            # Repeated occupied evidence must survive a few contradictory rays
            # from pose/range jitter. Genuine sustained free observations still
            # clear it; no point is kept occupied forever.
            increments=np.where(values>=9,-1,delta) if self.conservative_free and delta<0 else delta
            tile[local[:,1],local[:,0]] = np.clip(values+increments,-40,60)
        if len(absolute):
            xmin,ymin=absolute.min(axis=0);xmax,ymax=absolute.max(axis=0)
            if self.bounds:
                old=self.bounds
                xmin,ymin,xmax,ymax=min(old[0],xmin),min(old[1],ymin),max(old[2],xmax),max(old[3],ymax)
            self.bounds=tuple(int(v) for v in (xmin,ymin,xmax,ymax))

    def grid(self):
        if self._grid_cache is not None: return self._grid_cache
        xmin,ymin,xmax,ymax=self.bounds or (0,0,0,0)
        factor=max(1,math.ceil(max(xmax-xmin+1,ymax-ymin+1)/self.max_span))
        width=math.ceil((xmax-xmin+1)/factor); height=math.ceil((ymax-ymin+1)/factor)
        classes=np.zeros((height,width),dtype=np.uint8)
        occupied_bounds=None
        for (tx,ty),tile in self._tiles.items():
            # Uncertain cells stay unknown in the display, as in the base map.
            ys,xs=np.nonzero((tile <= -4) | ((tile != 127)&(tile >= 9)))
            ax=xs+tx*self.TILE; ay=ys+ty*self.TILE
            hit=tile[ys,xs] >= 9
            np.maximum.at(classes,((ay-ymin)//factor,(ax-xmin)//factor),np.where(hit,2,1).astype(np.uint8))
            if hit.any():
                b=(int(ax[hit].min()),int(ay[hit].min()),int(ax[hit].max())+1,int(ay[hit].max())+1)
                if occupied_bounds:
                    old=occupied_bounds
                    b=(min(old[0],b[0]),min(old[1],b[1]),max(old[2],b[2]),max(old[3],b[3]))
                occupied_bounds=b
        pixels=np.array([127,245,25],dtype=np.uint8)[classes]
        self._grid_cache=dict(width=width,height=height,resolution=self.resolution*factor,
            source_resolution=self.resolution,downsample_factor=factor,
            origin={'x':xmin*self.resolution,'y':ymin*self.resolution},
            occupied_bounds=None if occupied_bounds is None else dict(zip(
                ('minX','minY','maxX','maxY'),(v*self.resolution for v in occupied_bounds))),
            encoding='png',row_order='min_y_first',
            image_base64=base64.b64encode(grayscale_png(width,height,pixels.tobytes())).decode())
        return self._grid_cache

    def snapshot(self):
        return dict(super().snapshot(),storage='tiled_logodds_v1',retained_tiles=len(self._tiles),
                    display_downsampled=self.grid()['downsample_factor']>1)

    def save_checkpoint(self, path):
        from dashboard_server.sensor_mapping.tiled_checkpoint import save
        return save(self, path)

    @classmethod
    def load_checkpoint(cls, path):
        from dashboard_server.sensor_mapping.tiled_checkpoint import load
        return load(cls, path)

    def planning(self):
        occupied=[];free_count=0
        for (tx,ty),tile in self._tiles.items():
            ys,xs=np.nonzero((tile != 127)&(tile >= 9))
            occupied.extend(np.column_stack((xs+tx*self.TILE,ys+ty*self.TILE)).tolist())
            free_count+=int((tile <= -4).sum())
        return dict(resolution=self.resolution,bounds=self.bounds,occupied=occupied,
                    free_count=free_count,pose=self.pose,stamp=self.stamp,coordinate_frame='odom')
