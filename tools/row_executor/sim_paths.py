"""External locations of the SIM-side tools of this package (no machine-specific path in the repository).

Users: sim_in_loop.py (dashboard folder), single_row_sim.py (single-row PREREG.json), single_row_video.py (ffmpeg),
field_records.py (10/10 manual field records) and the tests (dashboard folder, field records, POSIX shell). The executor
runtime (executor_main, core, config, guard, bridge_client, ...) reads nothing from here. The HOLD_MEASUREMENT folder,
the 10/10 maps and the corridor catalogue come from the SIM tree resolver sim/ubts/closed_loop/external_paths.py.

Each location is looked up in this order:
  1. its environment variable, when set and not blank;
  2. its key in the JSON file named by UBTS_ROW_EXECUTOR_PATHS; when that variable is not set, the file
     row_executor_paths.json in the folder of the file named by UBTS_STEER_PATHS (the steering SIM config folder),
     when that file exists;
  3. its default, only for the locations that list one below (never a machine-specific path);
otherwise PathNotConfigured (a FileNotFoundError) naming the variable, the key and docs/steering/EXECUTOR_IMPORT_KO.md.
A value that does not exist on disk raises FileNotFoundError naming where the value came from (configured_path).

  key                environment variable     default                       location
  dashboard_dir      UBTS_DASH_DIR            <repository>/dashboard        dashboard folder (scripts/rover_control_core.py)
  single_row_prereg  UBTS_SINGLE_ROW_PREREG   -                             single-row PREREG.json (read only)
  field_manual_1010  UBTS_RD_1010             -                             10/10 manual field records (lidar/)
  field_manual_1010_drive  UBTS_RD_1010_DRIVE  -                             10/10 drive extract files (<rid>.drive.json.gz)
  ffmpeg             UBTS_FFMPEG              ffmpeg on PATH                ffmpeg executable (videos only)
  posix_shell        UBTS_POSIX_SHELL         sh or bash on PATH            POSIX shell of the launcher tests (tests only)

Config file: {"schema": "ubts.row_executor.sim_paths.v1", "note": "...", "<key>": <value>, ...}. A value is a path
string, or an object {"nt": "...", "posix": "..."} whose entry is chosen by os.name. A relative path is taken relative
to the folder of the config file. Unknown keys, another schema and wrong value types are errors.

The dashboard default is this repository's dashboard/ folder: its scripts/rover_control_core.py has the same bytes as
the real dashboard checkout (design 2.4; A26 compares the two copies). Standard library only.
"""
import json
import os
from pathlib import Path
import shutil

CONFIG_ENV = 'UBTS_ROW_EXECUTOR_PATHS'
STEER_CONFIG_ENV = 'UBTS_STEER_PATHS'
SIBLING_NAME = 'row_executor_paths.json'
SCHEMA = 'ubts.row_executor.sim_paths.v1'
DOC = 'docs/steering/EXECUTOR_IMPORT_KO.md'
REPO_ROOT = Path(__file__).resolve().parents[2]
KEYS = {
    'dashboard_dir': 'UBTS_DASH_DIR',
    'single_row_prereg': 'UBTS_SINGLE_ROW_PREREG',
    'field_manual_1010': 'UBTS_RD_1010',
    'field_manual_1010_drive': 'UBTS_RD_1010_DRIVE',
    'ffmpeg': 'UBTS_FFMPEG',
    'posix_shell': 'UBTS_POSIX_SHELL',
}
DESCRIPTIONS = {
    'dashboard_dir': 'the dashboard folder (scripts/rover_control_core.py)',
    'single_row_prereg': 'the single-row pre-registration file PREREG.json',
    'field_manual_1010': 'the 10/10 manual field records folder (lidar/)',
    'field_manual_1010_drive': 'the folder of the 10/10 drive extract files (<rid>.drive.json.gz, IMU and motor rows)',
    'ffmpeg': 'the ffmpeg executable',
    'posix_shell': 'a POSIX shell for the launcher tests',
}
OS_NAMES = ('nt', 'posix')
OTHER_TOP_KEYS = ('schema', 'note')


class PathNotConfigured(FileNotFoundError):
    """No environment variable, no config-file entry and no default for a location."""


def config_file():
    """(path, how) of the config file in use, or (None, reason) when there is none."""
    value = os.environ.get(CONFIG_ENV, '').strip()
    if value:
        return Path(value), 'named by %s' % CONFIG_ENV
    steer = os.environ.get(STEER_CONFIG_ENV, '').strip()
    if steer:
        sibling = Path(steer).parent / SIBLING_NAME
        if sibling.is_file():
            return sibling, '%s next to the file named by %s' % (SIBLING_NAME, STEER_CONFIG_ENV)
        return None, '%s is not set and %s has no %s next to it' % (CONFIG_ENV, STEER_CONFIG_ENV, SIBLING_NAME)
    return None, 'neither %s nor %s is set' % (CONFIG_ENV, STEER_CONFIG_ENV)


def load_config(path):
    """Checked content of one config file (dict). Raises on a bad file."""
    path = Path(path)
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise FileNotFoundError('row executor path config %s cannot be read (%s)' % (path, error))
    try:
        doc = json.loads(raw.decode('utf-8'))
    except ValueError as error:
        raise ValueError('row executor path config %s is not UTF-8 JSON: %s' % (path, error))
    if not isinstance(doc, dict):
        raise ValueError('row executor path config %s must hold a JSON object' % path)
    if doc.get('schema', SCHEMA) != SCHEMA:
        raise ValueError('row executor path config %s: schema %r, expected %r' % (path, doc.get('schema'), SCHEMA))
    unknown = sorted(set(doc) - set(KEYS) - set(OTHER_TOP_KEYS))
    if unknown:
        raise ValueError('row executor path config %s: unknown keys %s (known: %s)' % (path, unknown, sorted(KEYS)))
    for key in KEYS:
        value = doc.get(key)
        if value is None or isinstance(value, str):
            continue
        if (not isinstance(value, dict) or not value or set(value) - set(OS_NAMES)
                or not all(isinstance(v, str) for v in value.values())):
            raise ValueError('row executor path config %s: %r must be a path string or an object with string '
                             'entries %s' % (path, key, OS_NAMES))
    return doc


def default(key):
    """The built-in default of a location (None when it has none or the default finds nothing)."""
    if key == 'dashboard_dir':
        return str(REPO_ROOT / 'dashboard')
    if key == 'ffmpeg':
        return shutil.which('ffmpeg')
    if key == 'posix_shell':
        return posix_shell_on_path()
    return None


def posix_shell_on_path():
    """sh or bash from PATH; on Windows not the System32 launcher (that is WSL, which cannot open drive-letter paths)."""
    system_root = os.environ.get('SystemRoot', '')
    for name in ('sh', 'bash'):
        found = shutil.which(name)
        if not found:
            continue
        if os.name == 'nt' and system_root and os.path.normcase(os.path.abspath(found)).startswith(
                os.path.normcase(os.path.join(system_root, 'System32'))):
            continue
        return found
    return None


def lookup(key):
    """(path text, source description) of one location. Raises PathNotConfigured when it has no value."""
    if key not in KEYS:
        raise KeyError('unknown location %r (known: %s)' % (key, sorted(KEYS)))
    env = KEYS[key]
    value = os.environ.get(env, '').strip()
    if value:
        return value, 'environment variable %s' % env
    path, how = config_file()
    if path is not None:
        entry = load_config(path).get(key)
        if isinstance(entry, dict):
            entry = entry.get(os.name)
        if entry and entry.strip():
            resolved = Path(entry.strip())
            if not resolved.is_absolute():
                resolved = path.parent / resolved
            return str(resolved), '%r in %s (%s)' % (key, path, how)
        how = '%s has no %r value for os.name %r' % (path, key, os.name)
    value = default(key)
    if value:
        return value, 'default'
    raise PathNotConfigured('%s is not configured: set %s, or %r in the JSON file named by %s (or %s next to the '
                            'file named by %s) (%s); see %s'
                            % (DESCRIPTIONS[key], env, key, CONFIG_ENV, SIBLING_NAME, STEER_CONFIG_ENV, how, DOC))


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


def optional_path(key, kind=None):
    """configured_path, or None when the location has no value at all (a configured but missing path still raises)."""
    try:
        return configured_path(key, kind)
    except PathNotConfigured:
        return None
