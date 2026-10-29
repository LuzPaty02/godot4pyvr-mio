"""Locating and reading the plain-text config files.

The original code built every path from ``os.getcwd()`` which only works when
launched from one specific directory.  Here the ``module_main`` directory is
resolved once, in this order:

1. an explicit argument,
2. the ``STRAS_MODULE_MAIN`` environment variable,
3. ``<current working directory>/module_main`` (the old behaviour).
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Union

PathLike = Union[str, os.PathLike]

QP_RANGE_FILE = Path("dicts") / "dofDictRange-qp.txt"
MODEL_CONFIG_FILE = Path("model_utilities") / "datafolders.txt"


def module_main_dir(override: Optional[PathLike] = None) -> Path:
    if override is not None:
        return Path(override)
    env = os.environ.get("STRAS_MODULE_MAIN")
    if env:
        return Path(env)
    return Path.cwd() / "module_main"


def read_key_value_file(path: PathLike) -> Dict[str, str]:
    """Parse a ``key=value`` per line text file (blank / ``#`` lines ignored)."""
    out: Dict[str, str] = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, val = line.split("=", 1)
            out[key.strip()] = val.strip()
    return out


@dataclass(frozen=True)
class ModelFiles:
    camera_calibration: Optional[Path]   # only needed by image renderers
    robot_right: Path
    robot_left: Path
    robot_endoscope: Path


def load_model_files(module_main: Optional[PathLike] = None) -> ModelFiles:
    """Resolve the camera / robot config files listed in datafolders.txt."""
    root = module_main_dir(module_main)
    d = read_key_value_file(root / MODEL_CONFIG_FILE)
    model_utils = root / "model_utilities"
    return ModelFiles(
        camera_calibration=(root / d["camera_calibration"]
                            if "camera_calibration" in d else None),
        robot_right=model_utils / d["robot_config_right"],
        robot_left=model_utils / d["robot_config_left"],
        robot_endoscope=model_utils / d["robot_config_endoscope"],
    )