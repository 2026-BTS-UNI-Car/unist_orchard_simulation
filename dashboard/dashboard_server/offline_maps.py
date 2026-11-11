"""Local map bundle metadata and confined asset lookup (no online fallback)."""
import json
import os
from pathlib import Path


def map_root():
    return Path(os.environ.get('OFFLINE_MAP_DIR', str(Path(__file__).resolve().parents[1] / 'data/maps')))


def asset_path(relative):
    root = map_root().resolve()
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file(): return None
    if path.suffix.lower() not in ('.pmtiles', '.pbf', '.png', '.json', '.woff2', '.txt'): return None
    return path


def map_status():
    root = map_root()
    try:
        manifest = json.loads((root / 'manifest.json').read_text(encoding='utf-8'))
        path = asset_path('korea.pmtiles')
        if path is None: raise OSError('missing map')
        with path.open('rb') as handle:
            if handle.read(8) != b'PMTiles\x03': raise ValueError('invalid map header')
        required = ['fonts/Noto Sans Regular/0-255.pbf', 'fonts/korean.woff2',
                    'sprites/v4/light.json', 'sprites/v4/light.png',
                    'sprites/v4/dark.json', 'sprites/v4/dark.png']
        missing = [name for name in required if asset_path(name) is None]
        return {'ok': not missing, 'state': 'ready' if not missing else 'assets_missing',
                'title': '대한민국 오프라인 지도', 'source': manifest.get('source'),
                'data_date': manifest.get('data_date'), 'bounds': manifest.get('bounds'),
                'maxzoom': manifest.get('maxzoom', 15), 'bytes': path.stat().st_size,
                'sha256': manifest.get('sha256'), 'missing': missing, 'url': '/maps/korea.pmtiles',
                'offline': True, 'attribution': '© OpenStreetMap contributors · Protomaps'}
    except (OSError, ValueError, TypeError):
        return {'ok': False, 'state': 'not_installed', 'offline': True,
                'error': '대한민국 지도 데이터가 설치되지 않았습니다.'}
