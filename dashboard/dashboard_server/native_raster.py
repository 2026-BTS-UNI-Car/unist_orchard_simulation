"""Optional exact raster kernel; absence/unsupported inputs use Python fallback."""
import ctypes
import os
from pathlib import Path

import numpy as np


def _load():
    try:
        library=ctypes.CDLL(str(Path(__file__).with_name('_native_raster.so')))
        kernel=library.ubts_raster
        kernel.argtypes=[ctypes.c_void_p]+[ctypes.c_int64]*6+[ctypes.c_void_p]
        kernel.restype=ctypes.c_int
        return kernel
    except (OSError, AttributeError):return None


KERNEL=_load()
try:
    UPDATE=ctypes.CDLL(str(Path(__file__).with_name('_native_raster.so'))).ubts_update_tiles
    UPDATE.argtypes=[ctypes.c_void_p]+[ctypes.c_int64]*5+[ctypes.c_void_p]+[ctypes.c_int64]*4
    UPDATE.restype=ctypes.c_int64
except (OSError,AttributeError):UPDATE=None


def backend_status():
    enabled=os.environ.get('UBTS_NATIVE_RASTER')!='0'
    return dict(ray_union='native_exact' if enabled and KERNEL is not None else 'numpy_reference',
                tile_update='native_exact' if enabled and UPDATE is not None else 'numpy_reference')


def raster_bitmap(rays, xmin, ymin, width, height, margin):
    if (KERNEL is None or os.environ.get('UBTS_NATIVE_RASTER')=='0'
            or not 0 <= margin <= 8 or max(width,height)>4096):return None
    values=np.ascontiguousarray(rays,dtype=np.int64)
    if values.ndim!=2 or values.shape[1]!=3 or not 1<=len(values)<=20000:return None
    bitmap=np.empty(width*height,dtype=np.uint8)
    ok=KERNEL(values.ctypes.data,len(values),xmin,ymin,width,height,margin,bitmap.ctypes.data)
    return bitmap if ok==1 else None


def update_tiles(coordinates, x0, y0, delta, conservative, tiles, keys):
    if (UPDATE is None or os.environ.get('UBTS_NATIVE_RASTER')=='0' or not len(keys)
            or not isinstance(delta,(int,np.integer))):return None
    low=keys.min(axis=0);size=keys.max(axis=0)-low+1
    if max(size)>128 or int(size[0])*int(size[1])>4096:return None
    # Production raster coordinates are lexicographic and unique. Keep the
    # reference gather semantics for any other caller, including duplicates.
    if not np.all((coordinates[1:,0]>coordinates[:-1,0]) | (
        (coordinates[1:,0]==coordinates[:-1,0]) & (coordinates[1:,1]>coordinates[:-1,1]))):return None
    table=np.zeros((int(size[0]),int(size[1])),dtype=np.uintp)
    for key in keys:
        tile=tiles.get(tuple(int(v) for v in key))
        if tile is not None and (tile.shape!=(64,64) or tile.dtype!=np.int8 or not tile.flags.c_contiguous):return None
    for key in keys:
        name=tuple(int(v) for v in key);tile=tiles.get(name)
        if tile is None:
            tile=np.full((64,64),127,dtype=np.int8);tiles[name]=tile
        table[tuple(key-low)]=tile.ctypes.data
    values=np.ascontiguousarray(coordinates,dtype=np.int64)
    fresh=UPDATE(values.ctypes.data,len(values),int(x0),int(y0),int(delta),int(conservative),
        table.ctypes.data,int(low[0]),int(low[1]),int(size[0]),int(size[1]))
    return int(fresh) if fresh>=0 else None
