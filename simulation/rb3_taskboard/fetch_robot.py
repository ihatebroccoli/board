#!/usr/bin/env python3
"""Download the pinned manufacturer RB3-730ES-U URDF and mesh sources.

Only the description package is copied; no ROS driver or executable graph is
downloaded or run. Existing files must match their recorded hashes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

REVISION = "5e8294a985e7ce5e20e70564c2681130afc5f502"
REPOSITORY = "https://github.com/RainbowRobotics/rbpodo_ros2"
DEFAULT_DIRECTORY = Path(__file__).resolve().parents[2] / "logs/simulation/rb3_taskboard/vendor/rbpodo_description"
FILES = ["LICENSE", "robots/rb3_730es_u.urdf"] + [
    f"meshes/rb3_730es_u/{kind}/link{i}.{extension}"
    for kind, extension in (("visual", "dae"), ("collision", "stl"))
    for i in range(7)
]


def fetch(destination: Path = DEFAULT_DIRECTORY) -> dict:
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=True)
    manifest_path = destination / "source_manifest.json"
    previous = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    if previous and previous.get("revision") != REVISION:
        raise RuntimeError("Asset directory already contains a different source revision")
    records = {}
    for relative in FILES:
        path = destination / relative
        url = f"https://raw.githubusercontent.com/RainbowRobotics/rbpodo_ros2/{REVISION}/rbpodo_description/{relative}"
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            request = urllib.request.Request(url, headers={"User-Agent": "rb3-taskboard-asset-fetch/1"})
            with urllib.request.urlopen(request, timeout=120) as response:
                content = response.read()
            temporary = path.with_suffix(path.suffix + ".partial")
            temporary.write_bytes(content)
            temporary.replace(path)
        content = path.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        old = previous.get("files", {}).get(relative)
        if old and old["sha256"] != digest:
            raise RuntimeError(f"Previously downloaded asset changed: {path}")
        records[relative] = {"url": url, "bytes": len(content), "sha256": digest}
        print(f"verified {relative}: {len(content):,} bytes", flush=True)
    manifest = {"repository": REPOSITORY, "revision": REVISION,
                "model": "rb3_730es_u", "license": "Apache-2.0; see LICENSE",
                "files": records}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_DIRECTORY)
    arguments = parser.parse_args()
    fetch(arguments.output)
