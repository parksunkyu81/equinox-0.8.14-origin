#!/usr/bin/env python3
"""Verify and atomically activate the official openpilot v0.8.13 C2 model."""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys


MODEL_VERSION = "commaai/openpilot v0.8.13 supercombo"
MODEL_TAG_COMMIT = "6ca0269f47d048600335ac5776fc7966d9ca339f"
STAGING_NAME = "v0813_official_20260813"
EXPECTED = {
  "supercombo.dlc": {
    "sha256": "209e9544e456dbc2a7d60490da65154e129bc84830909d8d931f97b3df93949b",
    "size": 56684955,
  },
  "supercombo.onnx": {
    "sha256": "2365bae967cce21ce68707c30bf2981bb7081ee5c3e6a3dff793e660f23ff622",
    "size": 57554657,
  },
}
EXPECTED_EON_THNEED = {
  "sha256": "15f2b2acbb47a9a69ff041a7420c6cc24eb12d6559a61f203f466f2f43159db0",
  "size": 29109366,
}


def sha256_file(path):
  digest = hashlib.sha256()
  with open(path, "rb") as file:
    for chunk in iter(lambda: file.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def file_info(path):
  path = Path(path)
  if not path.is_file():
    return {"present": False}
  return {
    "present": True,
    "size": path.stat().st_size,
    "sha256": sha256_file(path),
  }


def verify_official_files(directory):
  directory = Path(directory)
  result = {}
  for name, expected in EXPECTED.items():
    info = file_info(directory / name)
    info["valid"] = (info.get("size") == expected["size"] and
                     info.get("sha256") == expected["sha256"])
    result[name] = info
  if not all(info["valid"] for info in result.values()):
    raise RuntimeError(f"official v0.8.13 model verification failed: {result}")
  return result


def verify_eon_thneed(path):
  info = file_info(path)
  info["valid"] = (info.get("size") == EXPECTED_EON_THNEED["size"] and
                   info.get("sha256") == EXPECTED_EON_THNEED["sha256"])
  if not info["valid"]:
    raise RuntimeError(f"EON-generated v0.8.13 THNEED verification failed: {info}")
  return info


def verify_source_stack(root):
  root = Path(root)
  header = (root / "selfdrive/modeld/models/driving.h").read_text(encoding="utf-8")
  driving = (root / "selfdrive/modeld/models/driving.cc").read_text(encoding="utf-8")
  modeld = (root / "selfdrive/modeld/modeld.cc").read_text(encoding="utf-8")

  checks = {
    "noStopLineOutputBlock": "STOP_LINE_MHP_N" not in header and "ModelOutputStopLines" not in header,
    "singleModelFrame": "wide_frame" not in header and "wide_frame" not in driving,
    "singleImageModel": "s->m->addExtra" not in driving and "USE_GPU_RUNTIME, false" in driving,
    "singleCameraIpc": "vipc_client_extra" not in modeld and "use_extra_client" not in modeld,
  }
  if not all(checks.values()):
    raise RuntimeError(f"source is not the v0.8.13 single-camera parser stack: {checks}")
  return checks


def ensure_offroad(root):
  root = Path(root).resolve()
  if not (root / ".git").exists():
    raise RuntimeError(f"not an openpilot checkout: {root}")
  if Path("/EON").exists():
    param = Path("/data/params/d/IsOffroad")
    try:
      is_offroad = param.read_bytes().strip() == b"1"
    except OSError as error:
      raise RuntimeError(f"cannot verify EON offroad state: {error}")
    if not is_offroad:
      raise RuntimeError("model activation is only allowed while the device is offroad")


def copy_atomic(source, destination):
  source = Path(source)
  destination = Path(destination)
  temporary = destination.with_name(f".{destination.name}.v0813.tmp")
  shutil.copyfile(str(source), str(temporary))
  with open(temporary, "rb") as file:
    os.fsync(file.fileno())
  os.chmod(temporary, 0o600)
  os.replace(temporary, destination)


def write_json_atomic(path, data):
  path = Path(path)
  temporary = path.with_name(f".{path.name}.tmp")
  with open(temporary, "w", encoding="utf-8") as file:
    json.dump(data, file, indent=2, sort_keys=True)
    file.write("\n")
    file.flush()
    os.fsync(file.fileno())
  os.replace(temporary, path)


def activate(root):
  root = Path(root).resolve()
  ensure_offroad(root)
  source_checks = verify_source_stack(root)
  staging = root / "model_staging" / STAGING_NAME
  verify_official_files(staging)
  staged_thneed = staging / "supercombo.thneed"
  verify_eon_thneed(staged_thneed)

  models = root / "models"
  models.mkdir(mode=0o700, parents=True, exist_ok=True)
  backup = root / "model_backups" / f"pre_v0813_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
  backup.mkdir(mode=0o700, parents=True, exist_ok=False)

  previous = {}
  for name in ("supercombo.dlc", "supercombo.onnx", "supercombo.thneed"):
    source = models / name
    previous[name] = file_info(source)
    if source.is_file():
      shutil.copy2(str(source), str(backup / name))
  write_json_atomic(backup / "MODEL_BACKUP.json", {
    "createdAt": datetime.now().isoformat(timespec="seconds"),
    "previous": previous,
  })

  copy_atomic(staging / "supercombo.dlc", models / "supercombo.dlc")
  copy_atomic(staging / "supercombo.onnx", models / "supercombo.onnx")
  copy_atomic(staged_thneed, models / "supercombo.thneed")

  active = verify_official_files(models)
  active["supercombo.thneed"] = verify_eon_thneed(models / "supercombo.thneed")
  manifest = {
    "model": MODEL_VERSION,
    "officialTagCommit": MODEL_TAG_COMMIT,
    "activatedAt": datetime.now().isoformat(timespec="seconds"),
    "backup": str(backup),
    "sourceChecks": source_checks,
    "files": active,
  }
  write_json_atomic(models / "MODEL_SOURCE.json", manifest)
  return manifest


def status(root):
  root = Path(root).resolve()
  result = {
    "model": MODEL_VERSION,
    "officialTagCommit": MODEL_TAG_COMMIT,
    "root": str(root),
    "sourceChecks": verify_source_stack(root),
    "staging": verify_official_files(root / "model_staging" / STAGING_NAME),
    "active": {name: file_info(root / "models" / name)
               for name in ("supercombo.dlc", "supercombo.onnx", "supercombo.thneed")},
  }
  result["staging"]["supercombo.thneed"] = verify_eon_thneed(
    root / "model_staging" / STAGING_NAME / "supercombo.thneed")
  return result


def main():
  default_root = Path(__file__).resolve().parents[2]
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("action", choices=("status", "activate"), nargs="?", default="status")
  parser.add_argument("--root", default=str(default_root))
  args = parser.parse_args()

  try:
    result = activate(args.root) if args.action == "activate" else status(args.root)
  except Exception as error:
    print(f"ERROR: {error}", file=sys.stderr)
    return 1
  print(json.dumps(result, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
