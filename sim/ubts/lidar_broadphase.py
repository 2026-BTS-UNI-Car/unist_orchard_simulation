"""Conservative all-hit rejection; never filters hit arrays.

Within the admitted domain, |dx|,|dy| <= 2e6. A binary64 subtraction and a
two-product dot have absolute rounding errors conservatively below1e-8 m.
Let u=dx*pc+dy*ps and v=-dx*ps+dy*pc. Their inverse is
dx=(pc*u-ps*v)/(pc**2+ps**2), and analogously for dy. With |pc|,|ps|<=1,
norm>=1-1e-10, and |u|<=half_length, |v|<=half_width<=1e3, an original
inside result implies |dx|,|dy| <= half_length+half_width+approximately2.1e-7.
The explicit1e-3 m margin dominates this loose bound, including the square
comparison's subtraction. Out-of-domain coordinates/coefficients fall back.

The square test may retain false positives: any retained hit causes the exact
original full-array rotation and first-hit diagnostic to run. It cannot change
beam order, map samples, or the order of LiDAR/map checks across horizons.
"""
import math

import numpy as np

COORD_LIMIT=1e6
EXTENT_LIMIT=1e3
NORM_TOLERANCE=1e-10
MARGIN_M=1e-3


def prepare(world_hits,half_length,half_width):
    """Return the rejection square radius or None to preserve original work."""
    if (world_hits.dtype != np.dtype('float64') or world_hits.ndim!=2 or
            world_hits.shape[1]!=2 or
            not all(math.isfinite(v) and 0<=v<=EXTENT_LIMIT
                    for v in (half_length,half_width))):
        return None
    # NaN comparison is False; infinities/extreme coordinates also fall back.
    if not np.max(np.abs(world_hits),initial=0.)<=COORD_LIMIT:
        return None
    return half_length+half_width+MARGIN_M


def all_outside(world_hits,px,py,pc,ps,bound):
    """True only when skipping the ORIGINAL horizon test is conservative."""
    if (bound is None or not all(math.isfinite(v) for v in (px,py,pc,ps)) or
            max(abs(px),abs(py))>COORD_LIMIT or max(abs(pc),abs(ps))>1. or
            abs(pc*pc+ps*ps-1.)>NORM_TOLERANCE):
        return False
    nearby=(np.abs(world_hits[:,0]-px)<=bound)&(np.abs(world_hits[:,1]-py)<=bound)
    return not np.any(nearby)
