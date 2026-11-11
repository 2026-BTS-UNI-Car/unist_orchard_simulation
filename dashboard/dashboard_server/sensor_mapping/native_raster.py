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


def _load_coordinate_codes():
    try:
        kernel=ctypes.CDLL(str(Path(__file__).with_name('_native_raster.so'))).ubts_coordinate_codes
        kernel.argtypes=[ctypes.c_void_p]+[ctypes.c_int64]*4+[ctypes.c_void_p]
        kernel.restype=ctypes.c_int
        return kernel
    except (OSError,AttributeError):return None


COORDINATES=_load_coordinate_codes()


def coordinate_codes(codes, height, xmin, ymin):
    """Optional bounded code-to-Nx2 fill; preserve input order and original fallback."""
    if (COORDINATES is None or os.environ.get('UBTS_NATIVE_RASTER')=='0'
            or os.environ.get('UBTS_NATIVE_COORDINATES')=='0'
            or type(codes) is not np.ndarray or codes.dtype!=np.int64
            or codes.ndim!=1 or not codes.flags.c_contiguous or not codes.flags.aligned
            or len(codes)>4_194_304):return None
    if any(not isinstance(v,(int,np.integer)) or isinstance(v,(bool,np.bool_))
           for v in (height,xmin,ymin)):return None
    if not 1<=height<=4096 or not -4095<=xmin<=0 or not -4095<=ymin<=0:return None
    result=np.empty((len(codes),2),dtype=np.int64)
    ok=COORDINATES(codes.ctypes.data,len(codes),int(height),int(xmin),int(ymin),result.ctypes.data)
    return result if ok==1 else None


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
        if tile is not None and (type(tile) is not np.ndarray or tile.shape!=(64,64) or tile.dtype!=np.int8 or not tile.flags.c_contiguous or not tile.flags.writeable):return None
    for key in keys:
        name=tuple(int(v) for v in key);tile=tiles.get(name)
        if tile is None:
            tile=np.full((64,64),127,dtype=np.int8);tiles[name]=tile
        table[tuple(key-low)]=tile.ctypes.data
    values=np.ascontiguousarray(coordinates,dtype=np.int64)
    fresh=UPDATE(values.ctypes.data,len(values),int(x0),int(y0),int(delta),int(conservative),
        table.ctypes.data,int(low[0]),int(low[1]),int(size[0]),int(size[1]))
    return int(fresh) if fresh>=0 else None

# Private standalone metadata helper, unchanged except same-library default.
"""Private optional metadata prototype; it has no product/runtime wiring."""
import ctypes
import os
from pathlib import Path

import numpy as np

LIMIT = 2147483647
MAX_COUNT = 4194304
MAX_KEYS = 4096


def load_kernel(path=None):
    try:
        library = ctypes.CDLL(str(path or Path(__file__).with_name('_native_raster.so')))
        function = library.ubts_coordinate_tile_metadata
        function.argtypes = [ctypes.c_void_p, ctypes.c_int64, ctypes.c_int64,
            ctypes.c_int64, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int64,
            ctypes.c_void_p]
        function.restype = ctypes.c_int
        return function
    except (OSError, AttributeError):
        return None


METADATA = load_kernel()


def _integer(value):
    return not isinstance(value, (bool, np.bool_)) and (
        type(value) is int or isinstance(value, np.integer))


def coordinate_metadata(coordinates, x0, y0):
    """Owned bounds/key outputs, or None for the unchanged NumPy fallback.

    Inputs are read directly, never converted/copied. Exact base ndarray type
    excludes subclasses whose NumPy arithmetic may differ from their raw bytes.
    The caller must own stable input for the whole synchronous call: a read-only
    flag alone does not prove exclusive ownership or absence of another writer.
    The intended caller keeps
    len>=4096, admission-at-cap and native-UPDATE gates unchanged. Small inputs
    are supported by this standalone helper for independent boundary tests.
    """
    if (METADATA is None or os.environ.get('UBTS_NATIVE_RASTER') == '0'
            or os.environ.get('UBTS_NATIVE_TILE_METADATA') == '0'):
        return None
    if (type(coordinates) is not np.ndarray or coordinates.dtype != np.int64
            or coordinates.ndim != 2 or coordinates.shape[1] != 2
            or not coordinates.flags.c_contiguous or not coordinates.flags.aligned
            or not 1 <= len(coordinates) <= MAX_COUNT):
        return None
    if not all(_integer(value) and -LIMIT <= int(value) <= LIMIT for value in (x0, y0)):
        return None
    bounds = np.empty(4, dtype=np.int64)
    keys = np.empty((MAX_KEYS, 2), dtype=np.int64)
    count = np.empty(1, dtype=np.int64)
    ok = METADATA(coordinates.ctypes.data, len(coordinates), int(x0), int(y0),
        bounds.ctypes.data, keys.ctypes.data, MAX_KEYS, count.ctypes.data)
    if ok != 1:
        return None
    size = int(count[0])
    if not 1 <= size <= MAX_KEYS:
        raise ValueError('native_metadata_invalid_owned_output')
    # This is a read-only view of this call's newly owned fixed-size key output.
    # It does not share input memory and is not a security-sealed immutable type.
    result_keys = keys[:size]
    result_keys.setflags(write=False)
    return tuple(int(value) for value in bounds), result_keys
