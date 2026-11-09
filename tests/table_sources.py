"""Files in the neighbouring repos, and the workspace root above them.

Named for this repo's subject rather than the generic ``siblings``: the conftest here puts its own directory on
``sys.path``, and so does ``deploy/offboard/tests``, which carries a helper of exactly that generic name.  In
the shared root run whichever landed on the path first wins for BOTH trees, and the loser's tests die at import with
a missing attribute.  Test-helper module names are effectively global here -- give them a name no other tree would
pick.

The linkage table is generated here but read in two other repos, so most tests in this directory have to reach across
repo boundaries.  That is the point of them -- ``robot/contract/tests/test_ssot_parity.py`` does the same for
the arm poses -- but the workspace convention is that every repo stays usable on its own, so a repo that is not
checked out makes the test skip by name instead of failing.
"""

from __future__ import annotations

from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent


def workspace_root() -> Path | None:
    """Walk up to the ``workspace.repos`` marker (workspace-wide convention)."""
    for candidate in _HERE.parents:
        if (candidate / "workspace.repos").is_file():
            return candidate
    return None


def sibling(relpath: str) -> Path:
    """A file in a neighbouring repo.

    Skips only while there is NO workspace above this repo -- checked out on its own, this repo cannot know where
    its siblings would be, and that is a legitimate way to use it.

    Once a workspace IS found, a missing file FAILS instead.  The difference matters most in CI, which is the one
    place where the checkouts are scripted: a mistyped path there would turn every cross-repo comparison into a skip
    and report green having compared nothing.  Same rule as ``log/console``'s shell-parity test, and the same
    reasoning as ``robot-contract``'s ``test_ssot_parity.py`` -- a parity guard that stops guarding without a word is
    worse than one that is absent.
    """
    root = workspace_root()
    if root is None:
        pytest.skip("not inside the clearpath workspace (no workspace.repos above this file)")
    path = root / relpath
    if not path.is_file():
        pytest.fail(
            f"{relpath} is missing, but the workspace root {root} is right there. "
            "The linkage table cannot be compared against a copy that is not checked out -- "
            "run `vcs import < workspace.repos`, or fix the path if the file moved."
        )
    return path
