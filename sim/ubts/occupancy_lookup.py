"""Exact occupied/unknown lookup with an optional bounded batch fast path.

The grid is an index of the existing occupied-cell set, never another map or an
inflation layer. World samples, floor/division rules and input order stay intact.
Large/extreme extents use the scalar set lookup without allocating a dense grid.
"""
import math

import numpy as np


class OccupancyLookup:
    MAX_DENSE_CELLS = 1_000_000
    MAX_CELL_COORDINATE = 2**31-1

    def __init__(self, resolution, occupied):
        self.resolution=float(resolution)
        if not math.isfinite(self.resolution) or not .01<=self.resolution<=1.:
            raise ValueError('invalid_map_metadata')
        try:
            self.occupied=frozenset(tuple(cell) for cell in occupied)
        except TypeError as error:
            raise ValueError('invalid_map_cells') from error
        if any(len(cell)!=2 or not all(isinstance(value,int) for value in cell)
               for cell in self.occupied):
            raise ValueError('invalid_map_cells')
        self.grid=None;self.minimum=(0,0);self.maximum=(-1,-1)
        if not self.occupied:return
        low=tuple(min(cell[i] for cell in self.occupied) for i in (0,1))
        high=tuple(max(cell[i] for cell in self.occupied) for i in (0,1))
        width,height=high[0]-low[0]+1,high[1]-low[1]+1
        if (width*height>self.MAX_DENSE_CELLS or
                max(map(abs,low+high))>self.MAX_CELL_COORDINATE):return
        grid=np.zeros((height,width),dtype=np.bool_)
        cells=np.asarray(tuple(self.occupied),dtype=np.int64)
        grid[cells[:,1]-low[1],cells[:,0]-low[0]]=True
        grid.flags.writeable=False
        self.grid=grid;self.minimum=low;self.maximum=high

    def __call__(self, x, y):
        # Same scalar fallback used before the batch optimization.
        return 100 if (math.floor(x/self.resolution),math.floor(y/self.resolution)) in self.occupied else -1

    def _scalar_first(self,points):
        for index,(x,y) in enumerate(points):
            value=self(float(x),float(y))
            if value>=60:return index,value
        return None

    def first_occupied(self, points):
        """Return (first input index,100), or None, matching scalar scan order."""
        points=np.asarray(points,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2:
            raise ValueError('invalid_occupancy_samples')
        # Preserve scalar short-circuit/error behavior for nonfinite values and
        # avoid integer casts outside the small, exactly representable index.
        if self.grid is None or not np.isfinite(points).all():
            return self._scalar_first(points)
        # Divide, then floor (not reciprocal multiplication): cell boundaries,
        # including negative coordinates, must match math.floor(x/resolution).
        with np.errstate(over='ignore'):
            cells=np.floor(points/self.resolution)
        if not np.isfinite(cells).all():return self._scalar_first(points)
        low_x,low_y=self.minimum;high_x,high_y=self.maximum
        in_bounds=((cells[:,0]>=low_x)&(cells[:,0]<=high_x)&
                   (cells[:,1]>=low_y)&(cells[:,1]<=high_y))
        indices=np.flatnonzero(in_bounds)
        if not len(indices):return None
        selected=cells[indices].astype(np.int64)
        hits=self.grid[selected[:,1]-low_y,selected[:,0]-low_x]
        first=np.flatnonzero(hits)
        return (int(indices[first[0]]),100) if len(first) else None
