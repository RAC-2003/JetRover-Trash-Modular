"""Loads config.yaml and gives dotted-path access to it. No framework, no schema
validation library - just a dict and a getter, since every value here is a
plain scalar/list tunable, not something that needs its own class."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def _package_share_dir():
    try:
        share_dir = get_package_share_directory('trash_modular')
        if os.path.isdir(share_dir):
            return share_dir
    except Exception:
        pass
    # Fall back to the source tree (useful for test_nodes run without a build)
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..'))


def default_config_path():
    return os.path.join(_package_share_dir(), 'config', 'config.yaml')


def load_config(path=None):
    share_dir = _package_share_dir()
    path = path or os.path.join(share_dir, 'config', 'config.yaml')
    with open(path, 'r') as f:
        config = yaml.safe_load(f) or {}
    config['_share_dir'] = share_dir
    return config


def resolve_path(config, path):
    """Resolves a path from config.yaml relative to the package's share
    directory (e.g. 'config/grasp_calibration.csv' -> .../share/trash_modular/
    config/grasp_calibration.csv), so it works regardless of process CWD.
    Absolute paths pass through unchanged."""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(config.get('_share_dir', '.'), path))


def get(config, dotted_key, default=None):
    """get(config, 'navigation.linear_speed', 0.1)"""
    node = config
    for part in dotted_key.split('.'):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node
