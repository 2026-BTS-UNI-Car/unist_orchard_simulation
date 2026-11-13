"""Lossless cell changes inside an immutable, independently seeded window.

HTTP map snapshots remain idempotent full tiles. Only negotiated archive windows
use XOR of the previous 2-bit cells, which gzip compresses without losing a scan.
Original histories and native log odds are never modified.
"""
import base64
import zlib
import numpy as np


def packed_cells(tile):
    if tile['encoding']=='packed2-v1':
        result=np.frombuffer(base64.b64decode(tile['cells'],validate=True),dtype=np.uint8)
        if len(result)!=1024:raise ValueError('invalid_window_tile')
        return result
    if tile['encoding']!='rle2-v1':raise ValueError('unsupported_window_tile')
    runs=np.asarray(tile['cells'],dtype=np.int64)
    if len(runs)%2 or np.any(runs[::2]<0) or np.any(runs[::2]>3) or np.any(runs[1::2]<1) or runs[1::2].sum()!=4096:
        raise ValueError('invalid_window_runs')
    groups=np.repeat(runs[::2],runs[1::2]).astype(np.uint8).reshape(-1,4)
    return groups[:,0] | (groups[:,1]<<2) | (groups[:,2]<<4) | (groups[:,3]<<6)


def compact_window(window, codec='xor2-v1'):
    previous={(t['x'],t['y']):packed_cells(t) for t in window['seed'].get('tiles',[])}
    events=[]
    for event in window['events']:
        if event.get('tiles_reset'):previous={}
        tiles=[]
        for tile in event.get('tiles',[]):
            key=(tile['x'],tile['y']);current=packed_cells(tile);before=previous.get(key)
            previous[key]=current
            if before is None:
                tiles.append(tile);continue
            delta=np.bitwise_xor(current,before)
            if np.any(delta):tiles.append(dict(x=key[0],y=key[1],encoding='xor2-v1',cells=base64.b64encode(delta.tobytes()).decode('ascii')))
        events.append(dict(event,tiles=tiles))
    result=dict(window,events=events,cell_codec='xor2-v1')
    if codec!='xor2-deflate-v1':return result
    # Compress native 2-bit bytes before base64 instead of encoding each tile's
    # zero-heavy XOR as a 1,368-character JSON string. No cell is subsampled.
    blob=bytearray();offsets={}
    def encode(frame):
        tiles=[]
        for tile in frame.get('tiles',[]):
            raw=base64.b64decode(tile['cells'],validate=True) if tile['encoding']=='xor2-v1' else packed_cells(tile).tobytes()
            if len(raw)!=1024:raise ValueError('invalid_window_block')
            if raw not in offsets:
                if len(blob)+1024>64*1024*1024:raise ValueError('window_binary_limit')
                offsets[raw]=len(blob);blob.extend(raw)
            tiles.append(dict(tile,encoding='xor2-v1' if tile['encoding']=='xor2-v1' else 'packed2-v1',cells=offsets[raw]))
        return dict(frame,tiles=tiles)
    seed=encode(result['seed']);events=[encode(e) for e in result['events']]
    return dict(result,seed=seed,events=events,cell_codec=codec,
                cell_blob=base64.b64encode(zlib.compress(blob,6)).decode('ascii'),cell_blob_bytes=len(blob))
