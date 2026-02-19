#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Simple YAML tuner for MPC parameters.

Use this tool to quickly inspect and modify per-CAV parameters in
`src/bisa/config/cav_config.yaml` (or any compatible YAML file).

Examples:
  python3 mpc_parameter_tuner.py show --all
  python3 mpc_parameter_tuner.py set --cav 1 --key max_omega_rate --value 45
  python3 mpc_parameter_tuner.py set-many --cav 1 --set Q_heading=120 --set weight_input=0.2
  python3 mpc_parameter_tuner.py apply-profile --name fast_corner
  python3 mpc_parameter_tuner.py save-best --source cav_config.yaml --out cav_config.best.yaml
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
from pathlib import Path
from typing import Any, Dict, List

import yaml


DEFAULT_REPO = Path.home() / "Mobility_Challenge_Simulator"
DEFAULT_CONFIG = DEFAULT_REPO / "src" / "bisa" / "config" / "cav_config.yaml"
DEFAULT_BEST = DEFAULT_REPO / "src" / "bisa" / "config" / "cav_config.best.yaml"

CAV_SECTIONS = [f"mpc_tracker_cav{i:02d}" for i in range(1, 5)]

# Practical presets for quick iteration.
PROFILES: Dict[str, Dict[str, Any]] = {
    "stable": {
        "Q_heading": 95.0,
        "weight_input": 0.35,
        "max_omega_abs": 1.8,
        "max_omega_rate": 25.0,
        "ref_preview_steps": 10,
        "horizon": 55,
        "enable_path_fallback": True,
        "fallback_blend": 0.45,
    },
    "fast_corner": {
        "Q_heading": 120.0,
        "weight_input": 0.2,
        "max_omega_abs": 2.3,
        "max_omega_rate": 45.0,
        "ref_preview_steps": 16,
        "horizon": 40,
        "enable_path_fallback": True,
        "fallback_min_abs_omega": 0.04,
        "fallback_lookahead_index": 12,
        "fallback_max_abs_omega": 1.5,
        "fallback_blend": 0.7,
    },
    "smooth_corner": {
        "Q_heading": 105.0,
        "weight_input": 0.3,
        "max_omega_abs": 2.0,
        "max_omega_rate": 30.0,
        "ref_preview_steps": 12,
        "horizon": 50,
        "enable_path_fallback": True,
        "fallback_blend": 0.55,
    },
}


class TunerError(RuntimeError):
    pass

FLOAT_PARAM_KEYS = {
    "time_step",
    "Ts",
    "Q_pos",
    "Q_heading",
    "weight_position",
    "weight_heading",
    "weight_curvature",
    "weight_input",
    "w_d",
    "w_theta",
    "w_kappa",
    "w_u",
    "max_velocity",
    "min_velocity",
    "max_accel",
    "max_angular_vel",
    "max_omega_abs",
    "max_omega_rate",
    "max_v_rate",
    "kappa_limit_ref_velocity",
    "fallback_min_abs_omega",
    "fallback_max_abs_omega",
    "fallback_blend",
    "path_reset_distance_threshold",
    "u_min",
    "u_max",
    "kappa_min_delta",
    "kappa_max_delta",
}

def coerce_value_for_key(key: str, value: Any) -> Any:
    if key in FLOAT_PARAM_KEYS and isinstance(value, int):
        return float(value)
    return value

def load_yaml(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise TunerError(f"Config not found: {path}")
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise TunerError("Top-level YAML is not a map")
    return data


def save_yaml(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=False)


def backup_path(path: Path) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return path.with_suffix(path.suffix + f".{stamp}.bak")


def cav_section(cav_id: int) -> str:
    if cav_id < 1 or cav_id > 99:
        raise TunerError(f"Invalid cav id: {cav_id}")
    return f"mpc_tracker_cav{cav_id:02d}"


def ensure_ros_params(doc: Dict[str, Any], section: str) -> Dict[str, Any]:
    if section not in doc or not isinstance(doc[section], dict):
        doc[section] = {"ros__parameters": {}}
    sec = doc[section]
    if "ros__parameters" not in sec or not isinstance(sec["ros__parameters"], dict):
        sec["ros__parameters"] = {}
    return sec["ros__parameters"]


def parse_scalar(text: str) -> Any:
    s = text.strip()
    low = s.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    if low == "null":
        return None
    try:
        if any(ch in s for ch in [".", "e", "E"]):
            return float(s)
        return int(s)
    except ValueError:
        return s


def parse_kv(item: str) -> (str, Any):
    if "=" not in item:
        raise TunerError(f"Expected key=value, got: {item}")
    key, value = item.split("=", 1)
    key = key.strip()
    if not key:
        raise TunerError(f"Empty key in: {item}")
    return key, coerce_value_for_key(key, parse_scalar(value))


def target_sections(doc: Dict[str, Any], cav: int | None, all_cavs: bool) -> List[str]:
    if all_cavs:
        return [s for s in CAV_SECTIONS if isinstance(doc.get(s), dict)]
    if cav is None:
        raise TunerError("Specify --cav N or --all-cavs")
    return [cav_section(cav)]


def show_values(doc: Dict[str, Any], sections: List[str], keys: List[str] | None) -> None:
    for sec in sections:
        params = ensure_ros_params(doc, sec)
        print(f"[{sec}]")
        if keys:
            for k in keys:
                print(f"  {k}: {params.get(k, '<unset>')}")
        else:
            for k in sorted(params.keys()):
                print(f"  {k}: {params[k]}")


def write_changes(path: Path, doc: Dict[str, Any], dry_run: bool, make_backup: bool) -> None:
    if dry_run:
        print("[dry-run] no file written")
        return
    if make_backup and path.exists():
        bak = backup_path(path)
        save_yaml(bak, load_yaml(path))
        print(f"backup: {bak}")
    save_yaml(path, doc)
    print(f"saved: {path}")


def cmd_show(args: argparse.Namespace) -> int:
    path = Path(args.config)
    doc = load_yaml(path)
    sections = target_sections(doc, args.cav, args.all_cavs)
    show_values(doc, sections, args.key)
    return 0


def cmd_set(args: argparse.Namespace) -> int:
    path = Path(args.config)
    doc = load_yaml(path)
    sections = target_sections(doc, args.cav, args.all_cavs)
    value = coerce_value_for_key(args.key, parse_scalar(args.value))

    for sec in sections:
        params = ensure_ros_params(doc, sec)
        old = params.get(args.key, "<unset>")
        params[args.key] = value
        print(f"{sec}: {args.key}: {old} -> {value}")

    write_changes(path, doc, args.dry_run, not args.no_backup)
    return 0


def cmd_set_many(args: argparse.Namespace) -> int:
    path = Path(args.config)
    doc = load_yaml(path)
    sections = target_sections(doc, args.cav, args.all_cavs)
    kvs = [parse_kv(s) for s in args.set]

    for sec in sections:
        params = ensure_ros_params(doc, sec)
        for key, value in kvs:
            old = params.get(key, "<unset>")
            params[key] = value
            print(f"{sec}: {key}: {old} -> {value}")

    write_changes(path, doc, args.dry_run, not args.no_backup)
    return 0


def cmd_apply_profile(args: argparse.Namespace) -> int:
    path = Path(args.config)
    doc = load_yaml(path)
    if args.name not in PROFILES:
        raise TunerError(
            f"Unknown profile: {args.name}. Available: {', '.join(sorted(PROFILES.keys()))}"
        )
    sections = target_sections(doc, args.cav, args.all_cavs)
    updates = PROFILES[args.name]

    for sec in sections:
        params = ensure_ros_params(doc, sec)
        for key, value in updates.items():
            old = params.get(key, "<unset>")
            params[key] = value
            print(f"{sec}: {key}: {old} -> {value}")

    write_changes(path, doc, args.dry_run, not args.no_backup)
    return 0


def cmd_profiles(_: argparse.Namespace) -> int:
    for name, kv in sorted(PROFILES.items()):
        print(f"[{name}]")
        for k, v in kv.items():
            print(f"  {k}: {v}")
    return 0


def cmd_save_best(args: argparse.Namespace) -> int:
    src = Path(args.source)
    out = Path(args.out)
    doc = load_yaml(src)
    save_yaml(out, copy.deepcopy(doc))
    print(f"saved best snapshot: {out}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="MPC YAML parameter tuner")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_common(sp: argparse.ArgumentParser, *, for_show: bool = False) -> None:
        sp.add_argument("--config", default=str(DEFAULT_CONFIG), help="target cav_config yaml")
        sp.add_argument("--cav", type=int, help="single cav id (e.g. 1)")
        sp.add_argument("--all-cavs", action="store_true", help="apply/show to all cav sections")
        if not for_show:
            sp.add_argument("--dry-run", action="store_true", help="print only, do not write")
            sp.add_argument("--no-backup", action="store_true", help="skip .bak creation")

    sp = sub.add_parser("show", help="show parameter values")
    add_common(sp, for_show=True)
    sp.add_argument("--key", action="append", help="specific key(s) to display")
    sp.set_defaults(func=cmd_show)

    sp = sub.add_parser("set", help="set one key/value")
    add_common(sp)
    sp.add_argument("--key", required=True)
    sp.add_argument("--value", required=True)
    sp.set_defaults(func=cmd_set)

    sp = sub.add_parser("set-many", help="set multiple key=value pairs")
    add_common(sp)
    sp.add_argument("--set", action="append", required=True, help="key=value (repeatable)")
    sp.set_defaults(func=cmd_set_many)

    sp = sub.add_parser("profiles", help="list built-in profiles")
    sp.set_defaults(func=cmd_profiles)

    sp = sub.add_parser("apply-profile", help="apply a built-in profile")
    add_common(sp)
    sp.add_argument("--name", required=True, help=f"one of: {', '.join(sorted(PROFILES.keys()))}")
    sp.set_defaults(func=cmd_apply_profile)

    sp = sub.add_parser("save-best", help="copy current config to best snapshot file")
    sp.add_argument("--source", default=str(DEFAULT_CONFIG))
    sp.add_argument("--out", default=str(DEFAULT_BEST))
    sp.set_defaults(func=cmd_save_best)

    return p


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.func(args)
    except TunerError as e:
        print(f"error: {e}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
