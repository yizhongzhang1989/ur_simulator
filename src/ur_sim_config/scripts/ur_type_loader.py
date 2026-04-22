"""Shared loader for `config/ur_types/<ur_type>.yaml`.

Merges defaults from `_defaults.yaml` with the ur_type-specific overrides.
Used by gravity_compensation.py, sim_broadcasters.py, and the MJCF/URDF
generators.
"""
import os
from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as e:
    raise RuntimeError("PyYAML is required to load ur_type configs") from e


def _deep_merge(base, override):
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_ur_type(ur_type, *, package_share=None):
    """Return a dict with keys: effort_limits, home_positions, pid_hold."""
    share = package_share or get_package_share_directory("ur_sim_config")
    cfg_dir = os.path.join(share, "config", "ur_types")

    with open(os.path.join(cfg_dir, "_defaults.yaml")) as f:
        defaults_doc = yaml.safe_load(f) or {}
    defaults = defaults_doc.get("defaults", {})

    path = os.path.join(cfg_dir, f"{ur_type}.yaml")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"No ur_type config at {path}; add one in "
            f"src/ur_sim_config/config/ur_types/."
        )
    with open(path) as f:
        overrides = yaml.safe_load(f) or {}

    cfg = _deep_merge(defaults, overrides)

    required = ("effort_limits", "home_positions", "pid_hold")
    for key in required:
        if key not in cfg:
            raise KeyError(
                f"ur_type '{ur_type}' config missing '{key}' "
                f"(check {path} and _defaults.yaml)"
            )
    return cfg
