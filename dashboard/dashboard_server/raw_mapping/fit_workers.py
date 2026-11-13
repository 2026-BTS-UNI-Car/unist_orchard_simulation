"""Persistent bounded workers for independent registration hypotheses."""
import os
from concurrent.futures import ThreadPoolExecutor
WORKERS=max(1,min(4,int(os.environ.get('UBTS_MAP_FIT_WORKERS','1'))))
POOL=ThreadPoolExecutor(max_workers=WORKERS,thread_name_prefix='scan-fit') if WORKERS>1 else None


def evaluate(function, values):
    if POOL and len(values)>1:return list(POOL.map(function,values))
    return [function(value) for value in values]
