"""Put this directory and ``scripts/`` on ``sys.path``.

This repo builds with colcon (``ament_cmake``) and carries no Python package: the Python here is scripts --
``scripts/derive_finger_kinematics.py`` and its siblings, and the bridge under ``src/rg6_control/scripts/``.  The
repository's ``pyproject.toml`` carries the tooling only, and its pytest run collects these files by path.

Both entries are needed because the root run uses ``--import-mode=importlib``: under that mode pytest does NOT prepend
a test file's directory the way the classic ``prepend`` mode does, so neither ``import table_sources`` nor
``import derive_finger_kinematics`` would resolve on its own.  An ``__init__.py`` next to the tests is not the
alternative -- no package in this workspace ships its test tree.

The flip side of that path entry: helper module names are effectively GLOBAL across every test tree in the root run.
``table_sources`` is named for this repo's subject rather than something generic for exactly that reason -- see its
docstring.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
for _entry in (str(_HERE), str(_HERE.parent / "scripts")):
    if _entry not in sys.path:
        sys.path.insert(0, _entry)


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return _HERE.parent
