"""closed_loop.external_paths and its users: no built-in local path, a clear error when a location is missing.

Every test builds its own environment (monkeypatch) and config files (tmp_path); nothing is written into the
work tree and the real data is not needed.
"""
import json
import os
from pathlib import Path
import re

import pytest

from closed_loop import external_paths as EP
from closed_loop import raster_catalogue as RCAT
from closed_loop import raster_corridor as RC

HERE = Path(__file__).resolve().parent


@pytest.fixture
def clean_env(monkeypatch):
    """No location variable and no config file named (the environment of a fresh machine)."""
    for name in (EP.CONFIG_ENV,) + tuple(EP.KEYS.values()):
        monkeypatch.delenv(name, raising=False)
    return monkeypatch


def write_config(path, doc):
    path.write_bytes(json.dumps(doc).encode('utf-8'))
    return path


def test_every_location_raises_a_clear_error_when_nothing_is_configured(clean_env):
    for key, env in EP.KEYS.items():
        with pytest.raises(EP.PathNotConfigured) as caught:
            EP.lookup(key)
        message = str(caught.value)
        assert env in message and EP.CONFIG_ENV in message and repr(key) in message
        assert 'PATHS_KO.md' in message
        assert isinstance(caught.value, FileNotFoundError)
    with pytest.raises(KeyError, match='unknown location'):
        EP.lookup('maps_root')


def test_users_raise_instead_of_guessing_a_path(clean_env):
    with pytest.raises(EP.PathNotConfigured, match='UBTS_MAPS_1010'):
        RC.maps_root()
    with pytest.raises(EP.PathNotConfigured, match='UBTS_SINGLE_ROW_CATALOGUE'):
        RC.catalogue_path()
    with pytest.raises(EP.PathNotConfigured, match='UBTS_ROWS_1010'):
        RCAT.rows_path('040927')
    with pytest.raises(EP.PathNotConfigured, match='UBTS_HOLD_DIR'):
        EP.hold_file()
    assert RC.catalogue_path('given.json') == Path('given.json')       # an explicit path needs no configuration


def test_single_row_eval_refuses_to_start_without_a_hold_folder(clean_env, tmp_path):
    from closed_loop import single_row_eval as SRE       # imports the harness (scipy): kept out of module import
    with pytest.raises(EP.PathNotConfigured, match='UBTS_HOLD_DIR'):
        SRE.main(['--out', str(tmp_path / 'unused.json'), '--seeds', '0', '--scenes', 'nominal_open_end'])
    assert not (tmp_path / 'unused.json').exists()                     # stopped before any episode


def test_environment_variable_wins_over_the_config_file(clean_env, tmp_path):
    env_maps = tmp_path / 'env_maps'
    file_maps = tmp_path / 'file_maps'
    for root in (env_maps, file_maps):
        (root / 'offline' / 'maps').mkdir(parents=True)
    config = write_config(tmp_path / 'paths.json', dict(schema=EP.SCHEMA, maps_1010_root=str(file_maps)))
    clean_env.setenv(EP.CONFIG_ENV, str(config))
    assert Path(RC.maps_root()) == file_maps
    clean_env.setenv('UBTS_MAPS_1010', str(env_maps))
    assert Path(RC.maps_root()) == env_maps
    value, source = EP.lookup('maps_1010_root')
    assert source == 'environment variable UBTS_MAPS_1010'
    clean_env.setenv('UBTS_MAPS_1010', '   ')                     # blank = not set
    assert Path(RC.maps_root()) == file_maps


def test_config_file_values_per_os_relative_paths_and_existence(clean_env, tmp_path):
    rows = tmp_path / 'data' / 'rows'
    rows.mkdir(parents=True)
    (rows / 'rows_offline_040927.json').write_bytes(b'{}')
    catalogue = tmp_path / 'data' / 'CATALOGUE.json'
    catalogue.write_bytes(b'{}')
    other = 'posix' if os.name == 'nt' else 'nt'
    config = write_config(tmp_path / 'paths.json', {
        'schema': EP.SCHEMA, 'note': 'test',
        'rows_root': 'data/rows',                                          # relative to the config file folder
        'corridor_catalogue': {os.name: str(catalogue), other: '/elsewhere/CATALOGUE.json'},
        'hold_dir': {other: '/elsewhere'},                                 # nothing for this os.name
        'maps_1010_root': str(tmp_path / 'missing'),
    })
    clean_env.setenv(EP.CONFIG_ENV, str(config))
    assert RCAT.rows_path('040927') == rows / 'rows_offline_040927.json'
    with pytest.raises(FileNotFoundError, match='rows_offline_051127.json'):
        RCAT.rows_path('051127')
    assert RC.catalogue_path() == catalogue
    with pytest.raises(EP.PathNotConfigured, match='has no .hold_dir. value for os.name'):
        EP.hold_file()
    with pytest.raises(FileNotFoundError, match='not an existing folder') as caught:
        RC.maps_root()
    assert not isinstance(caught.value, EP.PathNotConfigured) and str(config) in str(caught.value)
    (tmp_path / 'missing').mkdir()
    with pytest.raises(FileNotFoundError, match='has no offline/maps folder'):
        RC.maps_root()


def test_hold_file_is_in_the_configured_folder(clean_env, tmp_path):
    clean_env.setenv('UBTS_HOLD_DIR', str(tmp_path))
    assert EP.hold_file() == tmp_path / 'HOLD_MEASUREMENT'
    assert not EP.hold_file().exists()


@pytest.mark.parametrize('doc, message', [
    ([1, 2], 'must hold a JSON object'),
    ({'schema': 'other.v1'}, 'schema'),
    ({'maps_root': 'x'}, 'unknown keys'),
    ({'rows_root': 5}, 'path string or an object'),
    ({'rows_root': {'windows': 'x'}}, 'path string or an object'),
    ({'rows_root': {}}, 'path string or an object'),
])
def test_bad_config_files_are_rejected(clean_env, tmp_path, doc, message):
    config = write_config(tmp_path / 'paths.json', doc)
    clean_env.setenv(EP.CONFIG_ENV, str(config))
    with pytest.raises(ValueError, match=message):
        EP.lookup('rows_root')


def test_unreadable_or_invalid_config_file_is_an_error(clean_env, tmp_path):
    clean_env.setenv(EP.CONFIG_ENV, str(tmp_path / 'absent.json'))
    with pytest.raises(FileNotFoundError, match='cannot be read'):
        EP.lookup('rows_root')
    bad = tmp_path / 'bad.json'
    bad.write_bytes(b'{"rows_root": ')
    clean_env.setenv(EP.CONFIG_ENV, str(bad))
    with pytest.raises(ValueError, match='not UTF-8 JSON'):
        EP.lookup('rows_root')


# a drive letter path, a WSL mount path, or the removed parents[4] lookup
ABSOLUTE_LOCAL_PATH = re.compile(r'(?<![A-Za-z])[A-Za-z]:[/\\]|/mnt/[a-z]/|parents\[4\]')


def test_no_absolute_local_path_left_in_the_cleaned_modules():
    for name in ('raster_corridor.py', 'raster_catalogue.py', 'single_row_eval.py', 'external_paths.py',
                 'harness.py'):
        text = (HERE / 'closed_loop' / name).read_text(encoding='utf-8')
        found = [(number, line.strip()) for number, line in enumerate(text.splitlines(), 1)
                 if ABSOLUTE_LOCAL_PATH.search(line)]
        assert not found, (name, found)
