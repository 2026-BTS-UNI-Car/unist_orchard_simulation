"""CPU intersections against actual static SDF visual triangles.

The simulation-only backend treats triangles as opaque and excludes the rover
itself. Unknown dynamic objects, semantic frames, includes, and geometry fail
explicitly. It does not publish or access hardware.
"""
from __future__ import annotations

import gc
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector

def rotation_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    """SDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]], dtype=np.float64)


def pose_matrix(node):
    result = np.eye(4)
    pose = node.find('pose')
    if pose is None:
        return result
    if pose.get('relative_to'):
        raise ValueError(f"Unsupported semantic pose frame: {pose.attrib}")
    fmt = pose.get('rotation_format', 'euler_rpy')
    xyzr = np.array([float(v) for v in (pose.text or '').split()])
    if not np.all(np.isfinite(xyzr)):
        raise ValueError('Non-finite SDF pose')
    if fmt == 'euler_rpy' and len(xyzr) == 6:
        angles = xyzr[3:]
        if pose.get('degrees', 'false').lower() in ('true', '1'):
            angles = np.deg2rad(angles)
        result[:3, :3] = rotation_rpy(*angles)
    elif fmt == 'quat_xyzw' and len(xyzr) == 7:
        x, y, z, w = xyzr[3:]
        if np.linalg.norm(xyzr[3:]) <= 0:
            raise ValueError('Zero SDF quaternion')
        result[:3, :3] = trimesh.transformations.quaternion_matrix([w, x, y, z])[:3, :3]
    else:
        raise ValueError(f'Unsupported SDF pose format/length: {fmt}/{len(xyzr)}')
    result[:3, 3] = xyzr[:3]
    return result


def rss_mb():
    status_file = Path('/proc/self/status')
    if not status_file.exists():
        return None
    for line in status_file.read_text().splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) / 1024.0
    return None


class StaticVisualWorld:
    def __init__(self, sdf_file, *, excluded_dynamic_models=('ubts_rover',)):
        self.sdf_file = Path(sdf_file)
        self.excluded_dynamic_models = frozenset(excluded_dynamic_models)
        self.sources = []
        self.skipped_dynamic = []
        self.primitive_approximations = []
        self.mesh_cache = {}
        self.parts = []
        root = ET.parse(self.sdf_file).getroot()
        world = root.find('world')
        if world is None:
            raise ValueError('Expected SDF with <world>')
        if world.findall('include') or world.findall('frame') or world.findall('actor'):
            raise ValueError('Unresolved world includes, semantic frames, or dynamic actors')
        world_transform = pose_matrix(world)
        started = time.perf_counter()
        for model in world.findall('model'):
            self._model(model, world_transform, '', False)
        if not self.parts:
            raise ValueError('No static visual triangles loaded')
        self.mesh = trimesh.util.concatenate(self.parts)
        self.parts.clear()
        self.mesh_cache.clear()
        gc.collect()
        self.load_seconds = time.perf_counter() - started
        self.rss_after_load_mb = rss_mb()
        self.rays = RayMeshIntersector(self.mesh, scale_to_box=True)

    def _model(self, model, parent_transform, parent_name, inherited_static):
        name = '/'.join(v for v in (parent_name, model.get('name', '?')) if v)
        is_static = inherited_static or model.findtext('static', 'false').strip().lower() in ('1', 'true')
        if not is_static:
            if name not in self.excluded_dynamic_models:
                raise ValueError(f'Unsupported dynamic model (cannot silently omit obstacle): {name}')
            self.skipped_dynamic.append(name)
            return
        if model.get('placement_frame') or model.findall('frame') or model.findall('include'):
            raise ValueError(f'Unresolved include/frame in static model {name}')
        model_transform = parent_transform @ pose_matrix(model)
        for link in model.findall('link'):
            link_transform = model_transform @ pose_matrix(link)
            for visual in link.findall('visual'):
                visual_name = f"{name}/{link.get('name', '?')}/{visual.get('name', '?')}"
                geometry = visual.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError(f'Missing/ambiguous geometry: {visual_name}')
                mesh, uri = self._geometry(geometry[0], visual_name)
                mesh.apply_transform(link_transform @ pose_matrix(visual))
                self.parts.append(mesh)
                self.sources.append({'visual': visual_name, 'uri': uri,
                                     'vertices': len(mesh.vertices), 'triangles': len(mesh.faces)})
        for child in model.findall('model'):
            self._model(child, model_transform, name, is_static)

    def _geometry(self, shape, visual_name):
        if shape.tag == 'mesh':
            uri = shape.findtext('uri', '')
            if uri.startswith('file://'):
                uri = uri[7:]
            if '://' in uri:
                raise ValueError(f'Unresolved mesh URI: {uri}')
            path = Path(uri)
            if not path.is_absolute():
                path = self.sdf_file.parent / path
            if shape.find('submesh') is not None:
                raise ValueError(f'Unsupported mesh submesh: {visual_name}')
            if path not in self.mesh_cache:
                loaded = trimesh.load_mesh(str(path), process=False,
                                           maintain_order=True, skip_materials=True)
                if isinstance(loaded, trimesh.Scene):
                    loaded = loaded.to_mesh()
                if not isinstance(loaded, trimesh.Trimesh) or not len(loaded.faces):
                    raise ValueError(f'Empty/unrecognized triangle mesh: {path}')
                if not np.all(np.isfinite(loaded.vertices)):
                    raise ValueError(f'Non-finite mesh vertices: {path}')
                self.mesh_cache[path] = loaded
            mesh = self.mesh_cache[path].copy()
            scale = np.array([float(v) for v in shape.findtext('scale', '1 1 1').split()])
            if scale.shape != (3,) or not np.all(np.isfinite(scale)) or np.any(scale == 0):
                raise ValueError(f'Invalid mesh scale: {visual_name}')
            mesh.apply_scale(scale)
            return mesh, str(path)
        if shape.tag == 'box':
            dims = [float(v) for v in shape.findtext('size').split()]
            if len(dims) != 3 or not np.all(np.isfinite(dims)) or np.any(np.array(dims) <= 0):
                raise ValueError(f'Invalid box dimensions: {visual_name}')
            return trimesh.creation.box(extents=dims), 'primitive:box'
        if shape.tag == 'cylinder':
            radius, length = float(shape.findtext('radius')), float(shape.findtext('length'))
            if not np.all(np.isfinite([radius, length])) or min(radius, length) <= 0:
                raise ValueError(f'Invalid cylinder dimensions: {visual_name}')
            self.primitive_approximations.append(visual_name + ': cylinder, 96 sections')
            return trimesh.creation.cylinder(radius=radius, height=length, sections=96), 'primitive:cylinder'
        if shape.tag == 'sphere':
            radius = float(shape.findtext('radius'))
            if not np.isfinite(radius) or radius <= 0:
                raise ValueError(f'Invalid sphere dimensions: {visual_name}')
            self.primitive_approximations.append(visual_name + ': sphere, 4 subdivisions')
            return trimesh.creation.icosphere(subdivisions=4, radius=radius), 'primitive:sphere'
        raise ValueError(f'Unsupported visual geometry {shape.tag}: {visual_name}')

    def describe(self):
        return {'sdf': str(self.sdf_file), 'static_visual_count': len(self.sources),
                'triangles': len(self.mesh.faces), 'vertices': len(self.mesh.vertices),
                'bounds': self.mesh.bounds.tolist(), 'skipped_dynamic_models': self.skipped_dynamic,
                'primitive_approximations': self.primitive_approximations,
                'mesh_load_seconds': self.load_seconds, 'rss_after_load_mb': self.rss_after_load_mb,
                'sources': self.sources}


class PlanarScanner:
    def __init__(self, intersector, count=3200, minimum=.05, maximum=30., mount=(.18, 0., .6)):
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError('Ray count must be a positive integer')
        if not np.isfinite(minimum) or not np.isfinite(maximum) or minimum < 0 or maximum <= minimum:
            raise ValueError('Invalid scan range limits')
        self.intersector = intersector
        self.minimum = minimum
        self.maximum = maximum
        self.mount = np.array(mount, dtype=np.float64)
        if self.mount.shape != (3,) or not np.all(np.isfinite(self.mount)):
            raise ValueError('Sensor mount must contain three finite coordinates')
        self.angles = -np.pi + np.arange(count) * (2 * np.pi / count)
        self.local_directions = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros(count)))

    def scan(self, xyz, rpy=(0., 0., 0.), *, rotation=None):
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError('Body position must contain three finite coordinates')
        if rotation is None:
            rpy = np.asarray(rpy, dtype=np.float64)
            if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
                raise ValueError('Body RPY must contain three finite angles')
            rotation = rotation_rpy(*rpy)
        else:
            rotation = np.asarray(rotation, dtype=np.float64)
            if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
                    or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                    or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)):
                raise ValueError('Body rotation must be an orthonormal proper 3x3 matrix')
        origin = xyz + rotation @ self.mount
        directions = self.local_directions @ rotation.T
        origins = np.tile(origin, (len(directions), 1))
        locations, index_ray, _ = self.intersector.intersects_location(origins, directions, multiple_hits=False)
        ranges = np.full(len(directions), np.inf, dtype=np.float64)
        distance = np.linalg.norm(locations - origin, axis=1)
        valid = np.isfinite(distance) & (distance <= self.maximum)
        # A real surface inside the minimum sensing distance remains a near
        # obstacle. Returning +Inf here would incorrectly clear occupied space.
        ranges[index_ray[valid]] = np.maximum(distance[valid], self.minimum)
        return ranges, origin, directions
