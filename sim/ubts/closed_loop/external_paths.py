"""External data locations of the closed-loop SIM tools (no machine-specific path in the repository).

Each location is looked up in this order:
  1. its own environment variable, when set and not blank;
  2. its key in the JSON file named by the environment variable UBTS_STEER_PATHS (a file kept outside the
     repository).
If neither gives a value, the lookup raises PathNotConfigured (a FileNotFoundError) that names both. A value
that does not exist on disk raises FileNotFoundError naming where the value came from. Nothing falls back to
a built-in local path.

  key                      environment variable           location
  maps_1010_root           UBTS_MAPS_1010                 10/10 maps folder (contains offline/maps/<record>/)
  corridor_catalogue       UBTS_SINGLE_ROW_CATALOGUE      single-row corridor catalogue (CATALOGUE.json)
  rows_root                UBTS_ROWS_1010                 10/10 row analysis folder (rows_offline_<record>.json)
  hold_dir                 UBTS_HOLD_DIR                  folder that may hold the HOLD_MEASUREMENT file
  baseline_546fa8c_traces  UBTS_BASELINE_546FA8C_TRACES   reference traces of commit 546fa8c (tests only)

Config file: {"schema": "ubts.steer_sim.paths.v1", "note": "...", "<key>": <value>, ...}. A value is a path
string, or an object {"nt": "...", "posix": "..."} whose entry is chosen by os.name (one file can serve
Windows and Linux). A relative path is taken relative to the folder of the config file. Unknown keys and
wrong value types are rejected, so a typo cannot pass silently. Documentation: docs/steering/PATHS_KO.md.
"""
import json
import os
from pathlib import Path

CONFIG_ENV = 'UBTS_STEER_PATHS'
SCHEMA = 'ubts.steer_sim.paths.v1'
KEYS = {
    'maps_1010_root': 'UBTS_MAPS_1010',
    'corridor_catalogue': 'UBTS_SINGLE_ROW_CATALOGUE',
    'rows_root': 'UBTS_ROWS_1010',
    'hold_dir': 'UBTS_HOLD_DIR',
    'baseline_546fa8c_traces': 'UBTS_BASELINE_546FA8C_TRACES',
}
DESCRIPTIONS = {
    'maps_1010_root': 'the 10/10 maps folder (offline/maps/<record>/)',
    'corridor_catalogue': 'the single-row corridor catalogue (CATALOGUE.json)',
    'rows_root': 'the 10/10 row analysis folder (rows_offline_<record>.json)',
    'hold_dir': 'the folder checked for the HOLD_MEASUREMENT file',
    'baseline_546fa8c_traces': 'the 546fa8c reference trace folder (MANIFEST.json)',
}
OS_NAMES = ('nt', 'posix')
OTHER_TOP_KEYS = ('schema', 'note')
HOLD_FILE_NAME = 'HOLD_MEASUREMENT'


class PathNotConfigured(FileNotFoundError):
    """No environment variable and no config-file entry for a location."""


def config_file():
    """Path named by UBTS_STEER_PATHS, or None when the variable is not set (or blank)."""
    value = os.environ.get(CONFIG_ENV, '').strip()
    return Path(value) if value else None


def load_config(path=None):
    """Checked content of the config file (dict); {} when no file is named. Raises on a bad file."""
    if path is None:
        path = config_file()
        if path is None:
            return {}
    path = Path(path)
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise FileNotFoundError('%s names %s, which cannot be read (%s)' % (CONFIG_ENV, path, error))
    try:
        doc = json.loads(raw.decode('utf-8'))
    except ValueError as error:
        raise ValueError('%s file %s is not UTF-8 JSON: %s' % (CONFIG_ENV, path, error))
    if not isinstance(doc, dict):
        raise ValueError('%s file %s must hold a JSON object' % (CONFIG_ENV, path))
    if doc.get('schema', SCHEMA) != SCHEMA:
        raise ValueError('%s file %s: schema %r, expected %r' % (CONFIG_ENV, path, doc.get('schema'), SCHEMA))
    unknown = sorted(set(doc) - set(KEYS) - set(OTHER_TOP_KEYS))
    if unknown:
        raise ValueError('%s file %s: unknown keys %s (known: %s)' % (CONFIG_ENV, path, unknown, sorted(KEYS)))
    for key in KEYS:
        value = doc.get(key)
        if value is None or isinstance(value, str):
            continue
        if (not isinstance(value, dict) or not value or set(value) - set(OS_NAMES)
                or not all(isinstance(v, str) for v in value.values())):
            raise ValueError('%s file %s: %r must be a path string or an object with string entries %s'
                             % (CONFIG_ENV, path, key, OS_NAMES))
    return doc


def lookup(key):
    """(path text, source description) of one location. Raises PathNotConfigured when it has no value."""
    if key not in KEYS:
        raise KeyError('unknown location %r (known: %s)' % (key, sorted(KEYS)))
    env = KEYS[key]
    value = os.environ.get(env, '').strip()
    if value:
        return value, 'environment variable %s' % env
    path = config_file()
    entry = load_config(path).get(key)
    if isinstance(entry, dict):
        entry = entry.get(os.name)
    if entry and entry.strip():
        resolved = Path(entry.strip())
        if not resolved.is_absolute():
            resolved = path.parent / resolved
        return str(resolved), '%r in %s file %s' % (key, CONFIG_ENV, path)
    if path is None:
        where = '%s is not set' % CONFIG_ENV
    else:
        where = '%s file %s has no %r value for os.name %r' % (CONFIG_ENV, path, key, os.name)
    raise PathNotConfigured('%s is not configured: set %s, or %r in the JSON file named by %s (%s); see '
                            'docs/steering/PATHS_KO.md' % (DESCRIPTIONS[key], env, key, CONFIG_ENV, where))


def configured_path(key, kind=None):
    """Path of one location. kind 'dir' or 'file' also requires it to exist as such (FileNotFoundError)."""
    if kind not in (None, 'dir', 'file'):
        raise ValueError('kind %r' % (kind,))
    value, source = lookup(key)
    path = Path(value)
    if kind == 'dir' and not path.is_dir():
        raise FileNotFoundError('%s (from %s) is not an existing folder: %s' % (DESCRIPTIONS[key], source, path))
    if kind == 'file' and not path.is_file():
        raise FileNotFoundError('%s (from %s) is not an existing file: %s' % (DESCRIPTIONS[key], source, path))
    return path


def hold_file():
    """The HOLD_MEASUREMENT file in the configured hold folder (the file itself may be absent)."""
    return configured_path('hold_dir', 'dir') / HOLD_FILE_NAME
