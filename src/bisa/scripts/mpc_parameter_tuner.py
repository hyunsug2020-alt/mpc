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
import os
from pathlib import Path
import sys
from typing import Any, Dict, List
import tkinter as tk
from tkinter import ttk, messagebox

import yaml


DEFAULT_REPO = Path.home() / "Mobility_Challenge_Simulator"
DEFAULT_CONFIG = DEFAULT_REPO / "src" / "bisa" / "config" / "cav_config.yaml"
DEFAULT_BEST = DEFAULT_REPO / "src" / "bisa" / "config" / "cav_config.best.yaml"

CAV_SECTIONS = [f"mpc_tracker_cav{i:02d}" for i in range(1, 5)]

PARAM_HELP: Dict[str, str] = {
    "Q_pos": "횡방향 위치 오차 가중치. 올리면 경로 밀착, 너무 높으면 출렁일 수 있음.",
    "Q_heading": "헤딩(방향) 오차 가중치. 올리면 커브를 더 빨리/강하게 반응.",
    "weight_input": "조향 입력 변화 패널티. 올리면 부드럽지만 반응이 둔해짐.",
    "max_velocity": "목표 속도 상한.",
    "max_v_rate": "속도 변화율 제한. 올리면 가감속 빠름, 내리면 부드러움.",
    "max_omega_abs": "요레이트(회전속도) 절대 상한. 올리면 코너링이 강해짐.",
    "max_omega_rate": "요레이트 변화율 제한. 올리면 조향 반응이 빨라짐.",
    "ref_preview_steps": "경로 미리보기 스텝 수. 올리면 커브를 더 일찍 봄.",
    "enable_path_fallback": "경로 이탈 시 보조 조향(fallback) 사용 여부.",
    "fallback_lookahead_index": "보조 조향이 참조할 전방 인덱스.",
    "fallback_max_abs_omega": "보조 조향에서 허용할 최대 요레이트.",
    "fallback_blend": "MPC와 보조 조향 혼합 비율.",
    "horizon": "예측 지평선 길이. 크면 안정적이나 반응이 느려질 수 있음.",
    "path_hold_distance_gain": "경로 고정 보조의 거리 오차 이득.",
    "path_hold_heading_gain": "경로 고정 보조의 헤딩 보정 이득.",
    "path_hold_max_omega": "경로 고정 보조의 최대 요레이트.",
    "path_hold_lookahead_index": "경로 고정 보조가 볼 전방 인덱스.",
    "path_reset_distance_threshold": "로컬 경로 점프 시 컨트롤러 리셋 거리 임계값.",
    "off_path_stop_distance": "경로에서 너무 멀면 정지하는 거리 임계값(코드 활성 시).",
}

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
    "path_hold_distance_gain",
    "path_hold_heading_gain",
    "path_hold_max_omega",
    "off_path_stop_distance",
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


def cmd_keys(_: argparse.Namespace) -> int:
    for key in sorted(PARAM_HELP.keys()):
        print(f"{key}: {PARAM_HELP[key]}")
    return 0


def cmd_save_best(args: argparse.Namespace) -> int:
    src = Path(args.source)
    out = Path(args.out)
    doc = load_yaml(src)
    save_yaml(out, copy.deepcopy(doc))
    print(f"saved best snapshot: {out}")
    return 0


def cmd_gui(args: argparse.Namespace) -> int:
    path = Path(args.config)
    doc = load_yaml(path)

    display = os.environ.get("DISPLAY", "").strip()
    xauth = os.environ.get("XAUTHORITY", "").strip()
    home_xauth = str(Path.home() / ".Xauthority")

    def _try_open(display_value: str | None, xauth_value: str | None) -> tk.Tk:
        if display_value:
            os.environ["DISPLAY"] = display_value
        if xauth_value:
            os.environ["XAUTHORITY"] = xauth_value
        return tk.Tk()

    root = None
    attempts = []
    candidate_displays: List[str] = []
    if display:
        candidate_displays.append(display)
    for p in sorted(Path("/tmp/.X11-unix").glob("X*")):
        cand = f":{p.name[1:]}"
        if cand not in candidate_displays:
            candidate_displays.append(cand)
    for cand in (":0", ":1"):
        if cand not in candidate_displays:
            candidate_displays.append(cand)

    candidate_xauth = [xauth] if xauth else []
    if home_xauth not in candidate_xauth and Path(home_xauth).exists():
        candidate_xauth.append(home_xauth)
    if "" not in candidate_xauth:
        candidate_xauth.append("")

    last_error: Exception | None = None
    for d in candidate_displays:
        for xa in candidate_xauth:
            try:
                root = _try_open(d, xa or None)
                break
            except tk.TclError as e:
                attempts.append(f"DISPLAY={d}, XAUTHORITY={xa or '<unset>'}: {e}")
                last_error = e
        if root is not None:
            break

    if root is None:
        short_attempts = "\n".join(attempts[:4])
        if len(attempts) > 4:
            short_attempts += f"\n... ({len(attempts) - 4} more attempts)"
        raise TunerError(
            "GUI open failed. Check desktop session env.\n"
            f"Tried DISPLAY values: {', '.join(candidate_displays)}\n"
            f"Current DISPLAY={display or '<unset>'}, XAUTHORITY={xauth or '<unset>'}\n"
            "Recent failures:\n"
            f"{short_attempts}\n"
            "Try in your GUI terminal:\n"
            "  export DISPLAY=:0\n"
            "  export XAUTHORITY=$HOME/.Xauthority\n"
            "  python3 src/bisa/scripts/mpc_parameter_tuner.py gui"
        ) from last_error
    root.title("MPC 파라미터 튜너")
    root.geometry("1220x780")
    style = ttk.Style(root)
    style.configure("Treeview", rowheight=30, font=("Noto Sans CJK KR", 11))
    style.configure("Treeview.Heading", font=("Noto Sans CJK KR", 12, "bold"))
    style.configure("TButton", font=("Noto Sans CJK KR", 11))
    style.configure("TLabel", font=("Noto Sans CJK KR", 11))
    style.configure("TCombobox", font=("Noto Sans CJK KR", 11))

    top = ttk.Frame(root, padding=10)
    top.pack(fill="x")
    body = ttk.Frame(root, padding=10)
    body.pack(fill="both", expand=True)
    bottom = ttk.Frame(root, padding=10)
    bottom.pack(fill="x")

    ttk.Label(top, text=f"설정 파일: {path}").pack(side="left")

    cav_var = tk.StringVar(value="1")
    ttk.Label(top, text="차량(CAV)").pack(side="left", padx=(12, 4))
    cav_combo = ttk.Combobox(top, textvariable=cav_var, state="readonly", width=6)
    cav_combo["values"] = ("1", "2", "3", "4")
    cav_combo.pack(side="left")
    filter_var = tk.StringVar()
    ttk.Label(top, text="키 검색").pack(side="left", padx=(16, 4))
    ttk.Entry(top, textvariable=filter_var, width=28).pack(side="left")

    cols = ("key", "value", "hint")
    tree = ttk.Treeview(body, columns=cols, show="headings")
    tree.heading("key", text="파라미터")
    tree.heading("value", text="값")
    tree.heading("hint", text="설명")
    tree.column("key", width=340, anchor="w")
    tree.column("value", width=220, anchor="w")
    tree.column("hint", width=620, anchor="w")
    yscroll = ttk.Scrollbar(body, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=yscroll.set)
    tree.pack(side="left", fill="both", expand=True)
    yscroll.pack(side="right", fill="y")

    edit_key_var = tk.StringVar()
    edit_val_var = tk.StringVar()
    ttk.Label(bottom, text="파라미터").grid(row=0, column=0, sticky="w")
    key_combo = ttk.Combobox(bottom, textvariable=edit_key_var, width=36)
    key_combo["values"] = tuple(sorted(PARAM_HELP.keys()))
    key_combo.grid(row=0, column=1, sticky="ew", padx=6)
    ttk.Label(bottom, text="값").grid(row=0, column=2, sticky="w")
    ttk.Entry(bottom, textvariable=edit_val_var, width=18).grid(row=0, column=3, sticky="ew", padx=6)

    status_var = tk.StringVar(value="준비됨")
    ttk.Label(bottom, textvariable=status_var).grid(row=1, column=0, columnspan=4, sticky="w", pady=(8, 0))
    bottom.columnconfigure(1, weight=1)

    def sec_name() -> str:
        return cav_section(int(cav_var.get()))

    def refresh_table() -> None:
        nonlocal doc
        doc = load_yaml(path)
        sec = sec_name()
        params = ensure_ros_params(doc, sec)
        tree.delete(*tree.get_children())
        keys = sorted(set(params.keys()) | set(PARAM_HELP.keys()))
        key_filter = filter_var.get().strip().lower()
        for k in keys:
            if key_filter and key_filter not in k.lower():
                continue
            v = params.get(k, "<unset>")
            hint = PARAM_HELP.get(k, "")
            tree.insert("", "end", values=(k, v, hint))
        status_var.set(f"{sec} 로드 완료")

    def select_row(_evt=None) -> None:
        sel = tree.selection()
        if not sel:
            return
        vals = tree.item(sel[0], "values")
        if not vals:
            return
        edit_key_var.set(str(vals[0]))
        edit_val_var.set(str(vals[1]))

    def apply_one() -> None:
        nonlocal doc
        k = edit_key_var.get().strip()
        if not k:
            messagebox.showerror("오류", "파라미터 키를 입력하세요")
            return
        v_raw = edit_val_var.get().strip()
        v = coerce_value_for_key(k, parse_scalar(v_raw))
        sec = sec_name()
        params = ensure_ros_params(doc, sec)
        params[k] = v
        write_changes(path, doc, dry_run=False, make_backup=True)
        refresh_table()
        status_var.set(f"저장 완료: {sec} / {k}={v}")

    def apply_all() -> None:
        nonlocal doc
        k = edit_key_var.get().strip()
        if not k:
            messagebox.showerror("오류", "파라미터 키를 입력하세요")
            return
        v_raw = edit_val_var.get().strip()
        v = coerce_value_for_key(k, parse_scalar(v_raw))
        for cid in (1, 2, 3, 4):
            params = ensure_ros_params(doc, cav_section(cid))
            params[k] = v
        write_changes(path, doc, dry_run=False, make_backup=True)
        refresh_table()
        status_var.set(f"전체 CAV 저장 완료: {k}={v}")

    btns = ttk.Frame(bottom)
    btns.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))
    ttk.Button(btns, text="새로고침", command=refresh_table).pack(side="left")
    ttk.Button(btns, text="현재 CAV에 적용", command=apply_one).pack(side="left", padx=6)
    ttk.Button(btns, text="전체 CAV에 적용", command=apply_all).pack(side="left")

    cav_combo.bind("<<ComboboxSelected>>", lambda _e: refresh_table())
    tree.bind("<<TreeviewSelect>>", select_row)
    filter_var.trace_add("write", lambda *_: refresh_table())
    refresh_table()
    root.mainloop()
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

    sp = sub.add_parser("keys", help="list tunable keys with short meaning")
    sp.set_defaults(func=cmd_keys)

    sp = sub.add_parser("apply-profile", help="apply a built-in profile")
    add_common(sp)
    sp.add_argument("--name", required=True, help=f"one of: {', '.join(sorted(PROFILES.keys()))}")
    sp.set_defaults(func=cmd_apply_profile)

    sp = sub.add_parser("save-best", help="copy current config to best snapshot file")
    sp.add_argument("--source", default=str(DEFAULT_CONFIG))
    sp.add_argument("--out", default=str(DEFAULT_BEST))
    sp.set_defaults(func=cmd_save_best)

    sp = sub.add_parser("gui", help="open simple GUI tuner window")
    sp.add_argument("--config", default=str(DEFAULT_CONFIG), help="target cav_config yaml")
    sp.set_defaults(func=cmd_gui)

    return p


def main() -> int:
    parser = build_parser()
    if len(sys.argv) == 1:
        args = parser.parse_args(["gui"])
    else:
        args = parser.parse_args()
    try:
        return args.func(args)
    except TunerError as e:
        print(f"error: {e}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
