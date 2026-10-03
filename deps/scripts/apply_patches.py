#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
PATCH_ROOT = ROOT / "deps" / "patches" / "vendor"
VENDOR_ROOT = ROOT / "vendor"


def _run_git(repo_dir: Path, args: list[str]) -> subprocess.CompletedProcess[str]:
    """
    Run a git command in `repo_dir`.

    Parameters
    ----------
    repo_dir : pathlib.Path
        Git repository directory.
    args : list[str]
        Git arguments, e.g. ``["status", "--porcelain"]``.

    Returns
    -------
    subprocess.CompletedProcess[str]
        Completed process result.
    """
    return subprocess.run(
        ["git", "-C", str(repo_dir), *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def _is_patch_applied(repo_dir: Path, patch_path: Path) -> bool:
    """
    Check if a patch is already applied to a git checkout.

    Parameters
    ----------
    repo_dir : pathlib.Path
        Git repository directory.
    patch_path : pathlib.Path
        Patch file path.

    Returns
    -------
    bool
        True if the patch can be cleanly reversed (i.e., it is already applied).
    """
    res = _run_git(repo_dir, ["apply", "--reverse", "--check", str(patch_path)])
    return res.returncode == 0


def _apply_patch(repo_dir: Path, patch_path: Path, *, reverse: bool = False) -> None:
    """
    Apply (or reverse) a patch to a git checkout.

    Parameters
    ----------
    repo_dir : pathlib.Path
        Git repository directory.
    patch_path : pathlib.Path
        Patch file path.
    reverse : bool, optional
        If True, unapply the patch.

    Raises
    ------
    RuntimeError
        If the patch does not apply cleanly.
    """
    check_args = ["apply", "--check", str(patch_path)]
    apply_args = ["apply", "--whitespace=nowarn", str(patch_path)]
    action = "apply"

    if reverse:
        check_args = ["apply", "--reverse", "--check", str(patch_path)]
        apply_args = ["apply", "--reverse", "--whitespace=nowarn", str(patch_path)]
        action = "reverse"

    res = _run_git(repo_dir, check_args)
    if res.returncode != 0:
        raise RuntimeError(
            f"Patch failed to {action} cleanly in '{repo_dir}': {patch_path}\n{res.stderr}"
        )

    res = _run_git(repo_dir, apply_args)
    if res.returncode != 0:
        raise RuntimeError(
            f"Patch failed to {action} in '{repo_dir}': {patch_path}\n{res.stderr}"
        )


def _iter_repo_patches(repo_name: str) -> list[Path]:
    """
    List patch files for a given vendored repo.

    Parameters
    ----------
    repo_name : str
        Vendor repo name (directory under `vendor/`).

    Returns
    -------
    list[pathlib.Path]
        Patch files, sorted lexicographically.
    """
    repo_patch_dir = PATCH_ROOT / repo_name
    if not repo_patch_dir.is_dir():
        return []
    return sorted([p for p in repo_patch_dir.iterdir() if p.is_file()])


def main(argv: list[str]) -> int:
    """
    Apply repository-local patches to vendored dependencies.

    Parameters
    ----------
    argv : list[str]
        CLI arguments (excluding the program name).

    Returns
    -------
    int
        Process exit code.
    """
    parser = argparse.ArgumentParser(
        description="Apply patches in deps/patches/vendor to vendor/ checkouts."
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Unapply patches (reverse) instead of applying them.",
    )
    parser.add_argument(
        "--status",
        action="store_true",
        help="Print whether each patch is applied and exit without modifying anything.",
    )
    parser.add_argument(
        "--repo",
        default="",
        help="Only process a single vendored repo (directory name under vendor/).",
    )
    args = parser.parse_args(argv)

    if not PATCH_ROOT.is_dir():
        print(f"No patch directory found: {PATCH_ROOT}", file=sys.stderr)
        return 1

    repo_names = [p.name for p in sorted(PATCH_ROOT.iterdir()) if p.is_dir()]
    if args.repo:
        if args.repo not in repo_names:
            print(
                f"Unknown repo '{args.repo}'. Known repos: {repo_names}",
                file=sys.stderr,
            )
            return 1
        repo_names = [args.repo]

    for repo_name in repo_names:
        repo_dir = VENDOR_ROOT / repo_name
        if not repo_dir.is_dir():
            print(f"[SKIP] vendor repo not found: {repo_dir}", file=sys.stderr)
            continue

        patches = _iter_repo_patches(repo_name)
        for patch_path in patches:
            applied = _is_patch_applied(repo_dir, patch_path)
            if args.status:
                state = "APPLIED" if applied else "MISSING"
                print(f"[{state}] {repo_name}: {patch_path.name}")
                continue

            if args.reverse:
                if not applied:
                    print(f"[SKIP] {repo_name}: {patch_path.name} (not applied)")
                    continue
                _apply_patch(repo_dir, patch_path, reverse=True)
                print(f"[UNDO] {repo_name}: {patch_path.name}")
            else:
                if applied:
                    print(f"[SKIP] {repo_name}: {patch_path.name} (already applied)")
                    continue
                _apply_patch(repo_dir, patch_path, reverse=False)
                print(f"[APPLY] {repo_name}: {patch_path.name}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
