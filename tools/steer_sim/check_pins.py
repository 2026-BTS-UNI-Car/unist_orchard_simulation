#!/usr/bin/env python3
"""Guard for the reused base files of the steering simulation repository.

1. Base files listed in PIN_MANIFEST.json next to this file (the modules, dashboard copy and
   original-project files that the steering simulation reuses from the earlier simulator;
   sha256 and size as imported) must keep their pinned body, unless the path is listed in
   PIN_ALLOWLIST.json next to this file.
2. New Python modules or packages must not shadow a standard-library module, a third-party
   package used here, or a top-level package of this repository. "New" means added since the
   merge base with --base; when that ref does not exist (before the first push, or in a clone
   without that remote branch) it means every Python file that is not in the python_baseline
   list of PIN_MANIFEST.json (the Python files present at the import).

Usage: python -I tools/steer_sim/check_pins.py [--base origin/main] [--json]
Exit code 0 only when both checks pass.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).resolve().with_name("PIN_MANIFEST.json")
ALLOWLIST = Path(__file__).resolve().with_name("PIN_ALLOWLIST.json")

# Third-party or repository top-level names that a new module must not reuse.
EXTRA_RESERVED = {
    "yaml", "numpy", "scipy", "cv2", "rclpy", "websockets", "PIL", "pytest", "sdformat",
    "gz", "ros_gz", "launch", "launch_ros", "sensor_msgs", "geometry_msgs", "nav_msgs",
    "std_msgs", "builtin_interfaces", "tf2_ros", "scripts", "dashboard_server",
    "orchard_perception", "closed_loop", "encoderless", "command_sink", "tools", "sim",
    "dashboard", "tests", "test", "conftest",
}


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def git(args: list[str]) -> str:
    out = subprocess.run(["git", *args], cwd=str(ROOT), capture_output=True, text=True,
                         encoding="utf-8", errors="replace")
    if out.returncode != 0:
        raise SystemExit(f"git {' '.join(args)} failed: {out.stderr.strip()}")
    return out.stdout


def load_manifest() -> dict:
    manifest = json.loads(MANIFEST.read_bytes())
    if not isinstance(manifest.get("files"), dict) or not manifest["files"]:
        raise SystemExit(f"{MANIFEST.name}: no pinned files")
    if not isinstance(manifest.get("python_baseline"), list):
        raise SystemExit(f"{MANIFEST.name}: python_baseline is missing")
    return manifest


def check_pins(manifest: dict) -> dict:
    files = manifest["files"]
    allow = set(json.loads(ALLOWLIST.read_bytes()).get("allowed_changed_pinned_files", []))
    unknown_allow = sorted(a for a in allow if a not in files)
    changed, missing = [], []
    for rel, spec in sorted(files.items()):
        path = ROOT / rel
        if not path.is_file():
            missing.append(rel)
            continue
        data = path.read_bytes()
        digest = sha256(data)
        if spec["sha256"] != digest or spec["bytes"] != len(data):
            changed.append(rel)
    unexpected = sorted(p for p in changed + missing if p not in allow)
    return {
        "pinned_files": len(files),
        "unchanged": len(files) - len(changed) - len(missing),
        "changed": changed,
        "missing": missing,
        "allowed": sorted(allow),
        "allowlist_entries_not_pinned": unknown_allow,
        "unexpected": unexpected,
        "ok": not unexpected and not unknown_allow,
    }


def base_exists(base: str) -> bool:
    out = subprocess.run(["git", "rev-parse", "--verify", "--quiet", base + "^{commit}"], cwd=str(ROOT),
                         capture_output=True, text=True, encoding="utf-8", errors="replace")
    return out.returncode == 0


def new_python_paths(base: str, baseline: list[str]) -> tuple[list[str], str]:
    """(new Python paths, how "new" was decided)."""
    untracked = git(["ls-files", "--others", "--exclude-standard"]).splitlines()
    staged = git(["diff", "--cached", "--name-only", "--diff-filter=A"]).splitlines()
    if base_exists(base):
        merge_base = git(["merge-base", base, "HEAD"]).strip()
        added = git(["diff", "--name-only", "--diff-filter=A", merge_base, "HEAD"]).splitlines()
        how = f"added since the merge base with {base} ({merge_base[:12]})"
    else:
        known = set(baseline)
        added = [p for p in git(["ls-files"]).splitlines() if p not in known]
        how = f"{base} does not exist here: tracked files outside the import baseline of {MANIFEST.name}"
    return sorted({p for p in added + untracked + staged if p.endswith(".py")}), how


def check_shadowing(base: str, baseline: list[str]) -> dict:
    reserved = set(getattr(sys, "stdlib_module_names", ())) | EXTRA_RESERVED
    bad = []
    paths, how = new_python_paths(base, baseline)
    for rel in paths:
        p = Path(rel)
        names = [p.stem] if p.stem != "__init__" else []
        if p.name == "__init__.py":
            names.append(p.parent.name)
        for name in names:
            if name in reserved:
                bad.append({"path": rel, "shadows": name})
    return {"shadowing": bad, "new_python_files": len(paths), "new_means": how, "ok": not bad}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="origin/main")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    manifest = load_manifest()
    report = {"pins": check_pins(manifest), "names": check_shadowing(a.base, manifest["python_baseline"])}
    report["ok"] = report["pins"]["ok"] and report["names"]["ok"]
    if a.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        pins = report["pins"]
        print(f"pinned {pins['pinned_files']}: unchanged {pins['unchanged']}, "
              f"changed {len(pins['changed'])}, missing {len(pins['missing'])}, "
              f"allowed {len(pins['allowed'])}, unexpected {len(pins['unexpected'])}")
        for p in pins["unexpected"]:
            print(f"  UNEXPECTED pinned change: {p}")
        for p in pins["allowlist_entries_not_pinned"]:
            print(f"  allowlist entry is not a pinned file: {p}")
        print(f"new Python files {report['names']['new_python_files']} ({report['names']['new_means']})")
        for b in report["names"]["shadowing"]:
            print(f"  SHADOWING: {b['path']} reuses module name {b['shadows']!r}")
        print("OK" if report["ok"] else "FAIL")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
