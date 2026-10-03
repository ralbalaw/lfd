#!/usr/bin/env python3

from __future__ import annotations

import sys
import urllib.parse
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "deps/manifests/dependency.repos"
OUT = ROOT / "deps/manifests/dependency.lock.repos"
NESTED = ("dependency.repos", "dependencies.repos", "franka.repos")


def parse_manifest_repos(data: Any) -> dict[str, dict[str, Any]]:
    """
    Normalize a `.repos`/`.rosinstall` parsed YAML object into a repo mapping.

    Parameters
    ----------
    data : Any
        Parsed YAML object.

    Returns
    -------
    dict[str, dict[str, Any]]
        Mapping of checkout path/name to a spec dict.
    """
    if isinstance(data, dict):
        return data["repositories"]

    repos: dict[str, dict[str, Any]] = {}
    for entry in data:
        repo_type = next(iter(entry.keys()))
        spec = entry[repo_type]
        repos[spec["local-name"]] = {"type": repo_type, "url": spec.get("uri") or spec.get("url"), "version": spec.get("version")}
    return repos


def normalize_repo_spec(spec: dict[str, Any]) -> dict[str, Any]:
    """
    Normalize a single repo spec to always have `type`, `url`, and `version`.

    Parameters
    ----------
    spec : dict[str, Any]
        Repo spec dict.

    Returns
    -------
    dict[str, Any]
        Normalized spec dict.
    """
    out = dict(spec)
    if "url" not in out and "uri" in out:
        out["url"] = out["uri"]
    out.setdefault("version", None)
    return out


def normalize_repos(repos: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """
    Normalize a mapping of repos.

    Parameters
    ----------
    repos : dict[str, dict[str, Any]]
        Mapping from checkout path/name to a spec dict.

    Returns
    -------
    dict[str, dict[str, Any]]
        Normalized mapping.
    """
    return {k: normalize_repo_spec(v) for k, v in repos.items()}


def load_manifest_repos(path: Path) -> dict[str, dict[str, Any]]:
    """
    Load and normalize a manifest from disk.

    Parameters
    ----------
    path : pathlib.Path
        Manifest file path.

    Returns
    -------
    dict[str, dict[str, Any]]
        Normalized repo mapping.
    """
    return normalize_repos(parse_manifest_repos(yaml.safe_load(path.read_text(encoding="utf-8"))))


def load_manifest_repos_text(text: str) -> dict[str, dict[str, Any]]:
    """
    Load and normalize a manifest from text.

    Parameters
    ----------
    text : str
        Manifest contents.

    Returns
    -------
    dict[str, dict[str, Any]]
        Normalized repo mapping.
    """
    return normalize_repos(parse_manifest_repos(yaml.safe_load(text)))


def _normalize_repo_url(url: str) -> str:
    """
    Normalize common git URL forms into an HTTPS URL when possible.

    Parameters
    ----------
    url : str
        Git remote URL.

    Returns
    -------
    str
        Normalized URL.
    """
    if url.startswith("git@github.com:"):
        return "https://github.com/" + url.removeprefix("git@github.com:")
    if url.startswith("git@gitlab.com:"):
        return "https://gitlab.com/" + url.removeprefix("git@gitlab.com:")
    return url


def raw_url_for_ref(url: str, ref: str, relpath: str) -> str:
    """
    Build a raw-file URL for GitHub/GitLab at a ref (branch/tag/SHA).

    Parameters
    ----------
    url : str
        Git remote URL.
    ref : str
        Ref string used by the host raw endpoint (branch/tag/commit-ish).
    relpath : str
        File path within the repo.

    Returns
    -------
    str
        Raw-file URL.
    """
    url = _normalize_repo_url(url)
    if url.endswith(".git"):
        url = url[:-4]

    parsed = urllib.parse.urlparse(url)
    host = parsed.netloc
    repo_path = parsed.path.lstrip("/")

    ref_enc = urllib.parse.quote(ref, safe="")
    relpath_enc = urllib.parse.quote(relpath, safe="/")

    if host == "github.com":
        owner, repo = repo_path.split("/", 2)[:2]
        return f"https://raw.githubusercontent.com/{owner}/{repo}/{ref_enc}/{relpath_enc}"

    if host.endswith("gitlab.com"):
        return f"https://{host}/{repo_path}/-/raw/{ref_enc}/{relpath_enc}"

    raise RuntimeError(f"Unsupported host for raw fetch: {host}")


def http_get_text(url: str) -> str:
    """
    Fetch a URL as UTF-8 text.

    Parameters
    ----------
    url : str
        URL to fetch.

    Returns
    -------
    str
        Response body decoded as UTF-8.
    """
    req = urllib.request.Request(url, headers={"User-Agent": "lock_repos_git"})  # noqa: S310
    with urllib.request.urlopen(req, timeout=30) as resp:  # noqa: S310
        return resp.read().decode("utf-8")


def _version_sort_key(version: str | None) -> tuple[int, tuple[int, ...], str]:
    """
    Build a sortable key for a manifest `version` string.

    Parameters
    ----------
    version : str | None
        Version string (e.g., `0.7.4`, `v1.9.8`, `jazzy`, `master`).

    Returns
    -------
    tuple[int, tuple[int, ...], str]
        Sort key where a larger value is treated as "newer".
    """
    if not version:
        return (0, (), "")

    v = str(version).strip()

    digits: list[int] = []
    cur = ""
    for ch in v.lstrip("vV"):
        if ch.isdigit():
            cur += ch
        elif cur:
            digits.append(int(cur))
            cur = ""
    if cur:
        digits.append(int(cur))
    if digits:
        return (3, tuple(digits), "")

    vl = v.lower()
    if vl in {"main", "master", "rolling"}:
        return (2, (), vl)

    return (1, (), vl)


def warn(msg: str) -> None:
    """
    Print a warning to stderr.

    Parameters
    ----------
    msg : str
        Warning message.
    """
    print(f"[lock_repos_git] warning: {msg}", file=sys.stderr)


def collect_repos(root_repos: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """
    Expand a manifest graph by downloading nested manifests at their `version` refs.

    Parameters
    ----------
    root_repos : dict[str, dict[str, Any]]
        Root manifest repos (path -> spec).

    Returns
    -------
    dict[str, dict[str, Any]]
        Expanded repos mapping (path -> spec).
    """
    repos = dict(root_repos)

    root_upstream_version: dict[tuple[str, str], str | None] = {}
    upstream_version: dict[tuple[str, str], str | None] = {}
    for path, spec in list(repos.items()):
        spec = normalize_repo_spec(spec)
        repos[path] = spec
        upstream = (str(spec.get("type")), str(spec.get("url")))
        root_upstream_version[upstream] = spec.get("version")
        upstream_version[upstream] = spec.get("version")

    seen: set[str] = set()
    q = deque(repos.keys())
    while q:
        repo_path = q.popleft()
        if repo_path in seen:
            continue
        seen.add(repo_path)

        spec = repos[repo_path]
        if spec.get("type") != "git":
            continue
        url = str(spec["url"])
        ref = spec.get("version")
        if ref is None:
            continue

        base = Path(repo_path).parent
        for name in NESTED:
            manifest_url = raw_url_for_ref(url, str(ref), name)
            try:
                manifest_text = http_get_text(manifest_url)
            except urllib.error.HTTPError as exc:
                if exc.code == 404:
                    continue
                raise

            nested_repos = load_manifest_repos_text(manifest_text)
            for rel_path in sorted(nested_repos.keys()):
                child_spec = nested_repos[rel_path]
                child_path = (base / rel_path).as_posix()

                child_spec = normalize_repo_spec(child_spec)
                if child_spec.get("type") == "git":
                    child_url = str(child_spec["url"])
                    child_upstream = (str(child_spec.get("type")), child_url)
                    child_v = child_spec.get("version")
                    root_v = root_upstream_version.get(child_upstream)
                    existing_v = upstream_version.get(child_upstream)
                    if existing_v is None:
                        upstream_version[child_upstream] = child_v
                    elif existing_v != child_v:
                        keep_root = root_v is not None and _version_sort_key(root_v) >= _version_sort_key(child_v)
                        keep_v = root_v if keep_root else child_v
                        warn(f"version mismatch for upstream {child_upstream}; keeping {keep_v!r}")
                        child_spec["version"] = keep_v
                        upstream_version[child_upstream] = keep_v

                existing = repos.get(child_path)
                if existing is None:
                    repos[child_path] = child_spec
                    q.append(child_path)
                    continue

                if (existing.get("type"), existing.get("url")) != (child_spec.get("type"), child_spec.get("url")):
                    raise RuntimeError(f"path conflict for {child_path!r}: {existing} vs {child_spec}")

                if existing.get("version") != child_spec.get("version"):
                    upstream = (str(existing.get("type")), str(existing.get("url")))
                    root_v = root_upstream_version.get(upstream)
                    existing_v = existing.get("version")
                    child_v = child_spec.get("version")
                    keep_v = child_v
                    if _version_sort_key(existing_v) >= _version_sort_key(child_v):
                        keep_v = existing_v
                    if root_v is not None and _version_sort_key(root_v) >= _version_sort_key(keep_v):
                        keep_v = root_v
                    warn(f"version mismatch for {child_path!r}; keeping {keep_v!r}")
                    existing["version"] = keep_v

    return repos


def make_repos_distinct(repos: dict[str, dict[str, Any]], root_paths: set[str]) -> dict[str, dict[str, Any]]:
    """
    Collapse duplicates (same upstream) into one entry using a deterministic rule.

    Parameters
    ----------
    repos : dict[str, dict[str, Any]]
        Locked repos mapping (path -> spec).
    root_paths : set[str]
        Paths that appeared in the root manifest.

    Returns
    -------
    dict[str, dict[str, Any]]
        Distinct mapping with one entry per `(type,url)`.
    """
    buckets: dict[tuple[str, str], list[str]] = {}
    for path, spec in repos.items():
        key = (str(spec.get("type")), str(spec.get("url")))
        buckets.setdefault(key, []).append(path)

    out: dict[str, dict[str, Any]] = {}
    for key, paths in buckets.items():
        root_candidates = sorted(p for p in paths if p in root_paths)
        keep_path = root_candidates[0] if root_candidates else sorted(paths)[0]
        out[keep_path] = repos[keep_path]
    return out


def main() -> None:
    """Generate `dependency.lock.repos` by expanding nested manifests."""
    root_repos = load_manifest_repos(MAIN)
    locked = collect_repos(root_repos)
    locked = make_repos_distinct(locked, set(root_repos.keys()))

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(
        yaml.safe_dump({"repositories": dict(sorted(locked.items()))}, sort_keys=False),
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
