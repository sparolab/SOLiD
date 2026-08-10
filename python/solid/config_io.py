"""Load SOLiD parameters from YAML config files.

Parameters are managed via config, not hardcoded. Bundled profiles ship with the
package (``solid/profiles/*.yaml``); you can also point at your own YAML file.

    import solid
    cfg = solid.load_config("gs_rgbd")          # bundled profile by name
    cfg = solid.load_config("/path/to/my.yaml")  # or a custom file
    print(solid.list_profiles())                 # ['gs_rgbd', 'lidar_ouster', ...]

Works with whichever backend is active — the returned object is a ``solid.Config``
(C++ or pure-Python); YAML keys map 1:1 to Config fields, and unspecified fields
keep their defaults.
"""
from __future__ import annotations

import os
from importlib import resources
from typing import List

import yaml

_PROFILES_PKG = "solid.profiles"


def list_profiles() -> List[str]:
    """Names of the bundled profiles (without the .yaml extension)."""
    out = []
    for entry in resources.files(_PROFILES_PKG).iterdir():
        name = entry.name
        if name.endswith(".yaml"):
            out.append(name[: -len(".yaml")])
    return sorted(out)


def _read_yaml(name_or_path: str) -> dict:
    # A path to an existing file wins; otherwise treat it as a bundled profile.
    if os.path.isfile(name_or_path):
        with open(name_or_path, "r") as f:
            return yaml.safe_load(f) or {}
    stem = name_or_path[:-5] if name_or_path.endswith(".yaml") else name_or_path
    res = resources.files(_PROFILES_PKG) / (stem + ".yaml")
    if not res.is_file():
        raise FileNotFoundError(
            f"'{name_or_path}' is neither a file nor a bundled profile. "
            f"Available profiles: {list_profiles()}")
    return yaml.safe_load(res.read_text()) or {}


def load_config(name_or_path: str):
    """Return a solid.Config populated from a bundled profile name or a YAML path.

    Unknown keys raise, so typos in a config file are caught early.
    """
    from . import Config  # resolved backend (cpp or pure)

    data = _read_yaml(name_or_path)
    cfg = Config()
    valid = {f for f in dir(cfg) if not f.startswith("_")}
    for key, value in data.items():
        if key not in valid:
            raise KeyError(f"unknown config key '{key}' in '{name_or_path}'. "
                           f"valid keys: {sorted(valid)}")
        setattr(cfg, key, value)
    return cfg
