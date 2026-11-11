#!/usr/bin/env python3
"""Check docs/provenance/IMPORT_MANIFEST.json, the file-by-file origin table of the import (docs/provenance/
IMPORT_NOTES_KO.md).

  python -I tools/steer_sim/check_import_manifest.py [--rev HEAD] [--strict] [--worktree] [--old-repo PATH] [--json]

The table lists every file that came from the earlier simulation repository (its path and blob id there, the blob id,
sha256 and size it had here at the import, and whether it was changed on purpose) and the files that were newly
written for this repository.

Always checked (exit 1 on a problem):
  - the table is well formed: sorted unique paths, the stated counts, modified == (blob != source_blob), no path in
    both lists;
  - with git (not with --worktree): every recorded blob is an object of this repository and its bytes have the
    recorded sha256 and size.
Compared with a tree (the tree of --rev, default HEAD; with --worktree the files on disk as they are, without git):
  - listed files whose content is no longer the imported one, listed files that are gone, and files that are in
    neither list are counted and named. Once work continues this drift is expected; it is a problem only with
    --strict, which is for the import itself and for a fresh clone of it (the table must describe the tree exactly,
    file modes included when git is used).
--old-repo PATH: additionally check, in a clone of the earlier repository that still has the source commit, that
  every source_path has the recorded source_blob and mode there (git ls-tree; read-only).
Reads only; writes nothing.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_REL = "docs/provenance/IMPORT_MANIFEST.json"
MANIFEST = ROOT / MANIFEST_REL
SCHEMA = "ubts.sim_repo.import_manifest.v1"
SKIP_DIRS = (".git", "__pycache__", ".pytest_cache")
LIST_LIMIT = 40


def blob_id(data: bytes) -> str:
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def git(args: list[str], cwd: Path, data: bytes | None = None) -> bytes:
    out = subprocess.run(["git", *args], cwd=str(cwd), input=data, capture_output=True)
    if out.returncode != 0:
        raise SystemExit("git %s failed in %s: %s" % (" ".join(args), cwd,
                                                      out.stderr.decode("utf-8", "replace").strip()[:300]))
    return out.stdout


def tree_of(rev: str, cwd: Path) -> dict[str, tuple[str, str]]:
    """path -> (mode, blob id) of every file of a commit."""
    rows = {}
    for line in git(["ls-tree", "-r", "-z", rev], cwd).decode("utf-8").split("\0"):
        if not line:
            continue
        meta, path = line.split("\t", 1)
        mode, kind, oid = meta.split()
        if kind == "blob":
            rows[path] = (mode, oid)
    return rows


def disk_tree() -> dict[str, tuple[None, str]]:
    rows = {}
    for folder, dirs, files in os.walk(ROOT):
        dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
        for name in sorted(files):
            if name.endswith((".pyc", ".pyo")):
                continue
            path = Path(folder) / name
            rows[path.relative_to(ROOT).as_posix()] = (None, blob_id(path.read_bytes()))
    return rows


def check_table(manifest: dict) -> list[str]:
    problems = []
    if manifest.get("schema") != SCHEMA:
        problems.append("schema is not %s" % SCHEMA)
    files, new = manifest.get("files"), manifest.get("new_files")
    if not isinstance(files, list) or not files or not isinstance(new, list):
        return problems + ["files / new_files are missing"]
    paths = [e.get("path") for e in files]
    new_paths = [e.get("path") for e in new]
    if paths != sorted(set(paths)) or new_paths != sorted(set(new_paths)):
        problems.append("paths are not sorted and unique")
    both = sorted(set(paths) & set(new_paths))
    if both:
        problems.append("paths in both lists: %s" % both[:LIST_LIMIT])
    if MANIFEST_REL not in new_paths:
        problems.append("%s is not in new_files" % MANIFEST_REL)
    for e in files:
        for key in ("path", "source_path", "source_blob", "blob", "sha256", "bytes", "mode", "modified", "category"):
            if key not in e:
                problems.append("%s: field %s is missing" % (e.get("path"), key))
                break
        else:
            if e["modified"] != (e["blob"] != e["source_blob"]):
                problems.append("%s: modified does not match the blob ids" % e["path"])
    counts = manifest.get("counts", {})
    actual = dict(imported=len(files), imported_unchanged=sum(1 for e in files if not e.get("modified")),
                  imported_modified=sum(1 for e in files if e.get("modified")), new_files=len(new),
                  tracked_files=len(files) + len(new))
    for key, value in actual.items():
        if counts.get(key) != value:
            problems.append("counts.%s is %r, the lists give %d" % (key, counts.get(key), value))
    return problems


def check_objects(manifest: dict) -> list[str]:
    """Every recorded blob exists here and has the recorded sha256 and size (git cat-file --batch)."""
    entries = [e for e in manifest["files"] + manifest["new_files"] if e.get("blob")]
    out = git(["cat-file", "--batch"], ROOT, ("\n".join(e["blob"] for e in entries) + "\n").encode("ascii"))
    problems, pos = [], 0
    for e in entries:
        end = out.index(b"\n", pos)
        header = out[pos:end].decode("ascii", "replace").split()
        pos = end + 1
        if len(header) != 3 or header[1] != "blob":
            problems.append("%s: blob %s is not an object of this repository" % (e["path"], e["blob"]))
            continue
        size = int(header[2])
        data = out[pos:pos + size]
        pos += size + 1
        if size != e.get("bytes") or hashlib.sha256(data).hexdigest() != e.get("sha256"):
            problems.append("%s: blob %s does not have the recorded sha256 / size" % (e["path"], e["blob"]))
    return problems


def compare(manifest: dict, tree: dict, with_modes: bool) -> dict:
    listed = {e["path"]: e for e in manifest["files"] + manifest["new_files"]}
    changed, gone, mode_changed = [], [], []
    for path, e in sorted(listed.items()):
        if path not in tree:
            gone.append(path)
            continue
        mode, oid = tree[path]
        if e.get("blob") and oid != e["blob"]:
            changed.append(path)
        if with_modes and e.get("mode") and mode != e["mode"]:
            mode_changed.append(path)
    added = sorted(p for p in tree if p not in listed)
    return dict(changed=changed, gone=gone, added=added, mode_changed=mode_changed)


def check_old_repo(manifest: dict, old_repo: Path) -> list[str]:
    commit = manifest.get("source", {}).get("commit")
    if not commit:
        return ["source.commit is missing"]
    tree = tree_of(commit, old_repo)
    problems = []
    for e in manifest["files"]:
        if e["source_path"] not in tree:
            problems.append("%s: source_path %s is not in %s" % (e["path"], e["source_path"], commit[:12]))
        elif tree[e["source_path"]] != (e["mode"], e["source_blob"]):
            problems.append("%s: source blob or mode differs in %s" % (e["path"], commit[:12]))
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--rev", default="HEAD")
    ap.add_argument("--strict", action="store_true", help="any drift from the table is a problem")
    ap.add_argument("--worktree", action="store_true", help="compare the files on disk (no git)")
    ap.add_argument("--old-repo", type=Path, help="a clone of the earlier repository that has the source commit")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    try:
        manifest = json.loads(MANIFEST.read_bytes().decode("utf-8"))
    except (OSError, ValueError) as exc:
        print("import manifest unreadable: %s" % exc)
        return 1
    problems = check_table(manifest)
    drift = old = None
    if not problems:
        if a.worktree:
            tree, against = disk_tree(), "files on disk"
        else:
            problems += check_objects(manifest)
            tree, against = tree_of(a.rev, ROOT), "tree of %s" % a.rev
        drift = compare(manifest, tree, with_modes=not a.worktree)
        drift["against"] = against
        if a.strict:
            for key in ("changed", "gone", "added", "mode_changed"):
                if drift[key]:
                    problems.append("strict: %d %s: %s" % (len(drift[key]), key.replace("_", " "),
                                                           drift[key][:LIST_LIMIT]))
        if a.old_repo is not None:
            old = check_old_repo(manifest, a.old_repo)
            problems += old
    report = dict(ok=not problems, problems=problems, counts=manifest.get("counts"), strict=a.strict, drift=drift,
                  old_repo_checked=None if old is None else not old)
    if a.json:
        print(json.dumps(report, indent=1, ensure_ascii=False))
    else:
        counts = manifest.get("counts") or {}
        print("import manifest: %s imported (%s unchanged, %s changed on purpose), %s new" % (
            counts.get("imported"), counts.get("imported_unchanged"), counts.get("imported_modified"),
            counts.get("new_files")))
        if drift is not None:
            print("  %s: changed since the import %d, gone %d, added %d, mode changed %d%s" % (
                drift["against"], len(drift["changed"]), len(drift["gone"]), len(drift["added"]),
                len(drift["mode_changed"]), " (strict)" if a.strict else ""))
        if old is not None:
            print("  earlier repository: %s" % ("source blobs match" if not old else "MISMATCH"))
        for problem in problems:
            print("  PROBLEM: %s" % problem)
        print("OK" if report["ok"] else "FAIL")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
