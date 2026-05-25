#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Offline integrated scoring entry (no simulation).

Usage:
  python main.py /path/to/traj_csv_dir --scene-root /path/to/latest_scene_root

Behavior:
  1) Discover AV scene ids from trajectory CSV filenames.
  2) Discover BV scene ids strictly from BV xosc files.
  3) Build temporary roots:
     - BV raw root: only BV xosc files + xodr from GT root.
     - GT root: scenes needed by BV and AV.
  4) Run integrated_bv_av_scoring.py (BV first, then AV; missing side is filled as 0 in merge).
  5) Keep only final 2 csv reports; remove temporary files.
"""

import argparse
import datetime
import glob
import os
import re
import shutil
import subprocess
import sys
import warnings
from shapely.errors import ShapelyDeprecationWarning

# 忽略 Shapely 的弃用警告
warnings.filterwarnings("ignore", category=ShapelyDeprecationWarning)

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def _abs_from_root(path_value: str) -> str:
    if os.path.isabs(path_value):
        return path_value
    return os.path.abspath(os.path.join(PROJECT_ROOT, path_value))


def _extract_scene_id(name: str) -> str:
    m = re.search(r"(scenario_[0-9a-fA-F]+)", str(name))
    return m.group(1) if m else ""


def _iter_traj_csv_files(traj_dir: str):
    patterns = [
        "inference_data*.csv",
        "*scenario_*_output_*.csv",
        "**/inference_data*.csv",
        "**/*scenario_*_output_*.csv",
    ]
    out = []
    seen = set()
    for pat in patterns:
        for p in glob.glob(os.path.join(traj_dir, pat), recursive=True):
            ap = os.path.abspath(p)
            if ap in seen:
                continue
            seen.add(ap)
            out.append(ap)
    out.sort(key=lambda p: os.path.getmtime(p))
    return out


def _scene_ids_from_traj(traj_dir: str):
    ids = []
    for p in _iter_traj_csv_files(traj_dir):
        sid = _extract_scene_id(os.path.basename(p))
        if sid:
            ids.append(sid)
    # stable unique
    seen = set()
    uniq = []
    for s in ids:
        if s in seen:
            continue
        seen.add(s)
        uniq.append(s)
    return uniq


def _is_valid_scene_dir(scene_dir: str) -> bool:
    if not os.path.isdir(scene_dir):
        return False
    files = os.listdir(scene_dir)
    has_xosc = any(f.lower().endswith(".xosc") for f in files)
    has_xodr = any(f.lower().endswith(".xodr") for f in files)
    return has_xosc and has_xodr


def _find_scene_dir(scene_source_root: str, scene_id: str) -> str:
    direct = os.path.join(scene_source_root, scene_id)
    if _is_valid_scene_dir(direct):
        return direct

    # fallback: recursive search by folder name
    for cur, dirs, _ in os.walk(scene_source_root):
        for d in dirs:
            if d != scene_id:
                continue
            cand = os.path.join(cur, d)
            if _is_valid_scene_dir(cand):
                return cand
    return ""


def _collect_bv_scene_sources(bv_scene_root: str):
    """
    Strict BV source discovery:
    - ONLY *.xosc under bv_scene_root
    - NO fallback from GT xosc
    """
    by_scene = {}
    xosc_files = sorted(glob.glob(os.path.join(bv_scene_root, "**", "*.xosc"), recursive=True))
    for p in xosc_files:
        sid = _extract_scene_id(os.path.basename(p))
        if not sid:
            continue
        # stable first-win
        if sid not in by_scene:
            by_scene[sid] = os.path.abspath(p)
    return by_scene


def _pick_scene_files(scene_dir: str, scene_id: str):
    xodrs = sorted(glob.glob(os.path.join(scene_dir, "*.xodr")))
    xoscs = sorted(glob.glob(os.path.join(scene_dir, "*.xosc")))
    if (not xodrs) or (not xoscs):
        return "", "", ""

    gt_xosc = ""
    exam_xosc = ""
    for p in xoscs:
        low = os.path.basename(p).lower()
        if (not gt_xosc) and ("_gt.xosc" in low or low.endswith("gt.xosc")):
            gt_xosc = p
        if (not exam_xosc) and ("_exam.xosc" in low or low.endswith("exam.xosc")):
            exam_xosc = p

    if not gt_xosc:
        # fallback: if only one xosc exists, use it as gt.
        gt_xosc = xoscs[0]
    if not exam_xosc:
        # fallback: prefer one different from gt; otherwise reuse gt.
        for p in xoscs:
            if p != gt_xosc:
                exam_xosc = p
                break
        if not exam_xosc:
            exam_xosc = gt_xosc

    return xodrs[0], exam_xosc, gt_xosc


def _stage_bv_raw_and_av_gt_roots(
    bv_scene_sources,
    av_scene_ids,
    gt_scene_root: str,
    split: str,
    output_root: str,
    strict: bool,
):
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    stage_base = os.path.join(output_root, "_staging_scene_roots_" + ts)
    raw_root = os.path.join(stage_base, "raw")
    gt_root = os.path.join(stage_base, "gt")
    raw_split_dir = os.path.join(raw_root, split)
    gt_split_dir = os.path.join(gt_root, split)
    os.makedirs(raw_split_dir, exist_ok=True)
    os.makedirs(gt_split_dir, exist_ok=True)

    staged_bv = []
    staged_gt = []
    missing = []
    missing_gt_scene = []
    missing_gt_files = []
    missing_av_gt_scene = []

    # 1) Stage BV raw strictly from BV xosc list.
    for sid, bv_xosc in sorted(bv_scene_sources.items()):
        gt_src = _find_scene_dir(gt_scene_root, sid)
        if not gt_src:
            missing.append(sid)
            missing_gt_scene.append(sid)
            continue

        src_xodr, _, src_gt_xosc = _pick_scene_files(gt_src, sid)
        if (not src_xodr) or (not src_gt_xosc):
            missing.append(sid)
            missing_gt_files.append(sid)
            continue

        raw_dst = os.path.join(raw_split_dir, sid)
        gt_dst = os.path.join(gt_split_dir, sid)
        os.makedirs(raw_dst, exist_ok=True)
        os.makedirs(gt_dst, exist_ok=True)

        # BV raw candidate scene (single xosc + xodr).
        shutil.copy2(src_xodr, os.path.join(raw_dst, f"{sid}.xodr"))
        shutil.copy2(bv_xosc, os.path.join(raw_dst, f"{sid}.xosc"))

        # GT/AV scene (gt xosc + xodr).
        shutil.copy2(src_xodr, os.path.join(gt_dst, f"{sid}.xodr"))
        shutil.copy2(src_gt_xosc, os.path.join(gt_dst, f"{sid}_gt.xosc"))
        staged_bv.append(sid)
        staged_gt.append(sid)

    # 2) Stage AV-required GT scenes (no BV xosc needed here).
    for sid in av_scene_ids:
        if sid in staged_gt:
            continue
        gt_src = _find_scene_dir(gt_scene_root, sid)
        if not gt_src:
            if strict:
                missing.append(sid)
            missing_av_gt_scene.append(sid)
            continue
        src_xodr, _, src_gt_xosc = _pick_scene_files(gt_src, sid)
        if (not src_xodr) or (not src_gt_xosc):
            if strict:
                missing.append(sid)
            missing_gt_files.append(sid)
            continue
        gt_dst = os.path.join(gt_split_dir, sid)
        os.makedirs(gt_dst, exist_ok=True)
        shutil.copy2(src_xodr, os.path.join(gt_dst, f"{sid}.xodr"))
        shutil.copy2(src_gt_xosc, os.path.join(gt_dst, f"{sid}_gt.xosc"))
        staged_gt.append(sid)

    if strict and missing:
        raise RuntimeError(
            (
                "Missing %d scene dirs/files.\n"
                "  - gt_scene_missing_for_bv(%d): %s\n"
                "  - gt_files_incomplete(%d): %s\n"
                "  - gt_scene_missing_for_av(%d): %s"
            )
            % (
                len(missing),
                len(missing_gt_scene),
                ",".join(missing_gt_scene) if missing_gt_scene else "-",
                len(missing_gt_files),
                ",".join(missing_gt_files) if missing_gt_files else "-",
                len(missing_av_gt_scene),
                ",".join(missing_av_gt_scene) if missing_av_gt_scene else "-",
            )
        )

    if not staged_bv:
        raise RuntimeError("No BV scenes staged from bv_scene_root.")
    if not staged_gt:
        raise RuntimeError("No GT scenes staged from gt_scene_root.")

    return stage_base, raw_root, gt_root, staged_bv, staged_gt, missing


def _run_cmd(cmd):
    print("[RUN]", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def main():
    parser = argparse.ArgumentParser(description="Offline BV+AV integrated scoring from trajectory CSVs")
    parser.add_argument("traj_dir", help="Trajectory folder path, e.g. .../traj_output/test")
    parser.add_argument("--split", default="A", choices=["A", "B", "C"], help="Split tag for scoring output")
    parser.add_argument(
        "--bv-scene-root",
        default="bv_generated_scenario_scoring/scenario",
        help="BV candidate xosc source root (flat or recursive).",
    )
    parser.add_argument(
        "--gt-scene-root",
        default="groundtruth31_exam_gt_release_final_A200T50_20260520",
        help="GT scene root for locating gt.xosc and xodr by scene id.",
    )
    parser.add_argument("--output-root", default="outputs", help="Final scoring output root")
    parser.add_argument("--av-source-min-data-rows", type=int, default=1, help="Min rows to accept one trajectory csv")
    parser.add_argument("--num-workers-bv", type=int, default=4, help="BV worker count")
    parser.add_argument("--num-workers-av", type=int, default=4, help="AV worker count")
    parser.add_argument("--scene-limit", type=int, default=0, help="Optional scene limit for BV replay")
    parser.add_argument("--allow-missing-scenes", action="store_true", help="Do not fail when some scenes cannot be mapped")
    parser.add_argument("--keep-stage-root", action="store_true", help="Keep temporary staged AV gt root")
    args = parser.parse_args()

    traj_dir = _abs_from_root(args.traj_dir)
    bv_scene_root = _abs_from_root(args.bv_scene_root)
    gt_scene_root = _abs_from_root(args.gt_scene_root)
    output_root = _abs_from_root(args.output_root)

    if not os.path.isdir(traj_dir):
        raise RuntimeError("traj_dir not found: %s" % traj_dir)
    if not os.path.isdir(bv_scene_root):
        raise RuntimeError("bv_scene_root not found: %s" % bv_scene_root)
    if not os.path.isdir(gt_scene_root):
        raise RuntimeError("gt_scene_root not found: %s" % gt_scene_root)
    os.makedirs(output_root, exist_ok=True)

    av_scene_ids = _scene_ids_from_traj(traj_dir)
    if not av_scene_ids:
        raise RuntimeError("No scene id parsed from trajectory csv names under: %s" % traj_dir)
    bv_scene_sources = _collect_bv_scene_sources(bv_scene_root)
    if not bv_scene_sources:
        raise RuntimeError("No BV xosc scenes found under: %s" % bv_scene_root)

    print("[INFO] traj_dir=", traj_dir)
    print("[INFO] bv_scene_root=", bv_scene_root)
    print("[INFO] gt_scene_root=", gt_scene_root)
    print("[INFO] av_scene_count_from_csv=", len(av_scene_ids))
    print("[INFO] bv_scene_count_from_xosc=", len(bv_scene_sources))

    stage_base, staged_raw_root, staged_gt_root, staged_bv, staged_gt, missing = _stage_bv_raw_and_av_gt_roots(
        bv_scene_sources=bv_scene_sources,
        av_scene_ids=av_scene_ids,
        gt_scene_root=gt_scene_root,
        split=args.split,
        output_root=output_root,
        strict=(not args.allow_missing_scenes),
    )

    print("[INFO] stage_base=", stage_base)
    print("[INFO] staged_raw_root=", staged_raw_root)
    print("[INFO] staged_gt_root=", staged_gt_root)
    print("[INFO] staged_bv_scene_count=", len(staged_bv))
    print("[INFO] staged_gt_scene_count=", len(staged_gt))
    if missing:
        print("[WARN] missing_scene_count=", len(missing))
        print("[WARN] missing_scene_ids=", ",".join(missing))

    integrated_py = os.path.join(PROJECT_ROOT, "integrated_bv_av_scoring.py")
    cmd = [
        sys.executable,
        integrated_py,
        "--splits", args.split,
        "--raw-scene-root", staged_raw_root,
        "--gt-root", staged_gt_root,
        "--av-source-traj-dir", traj_dir,
        "--av-source-min-data-rows", str(int(args.av_source_min_data_rows)),
        "--num-workers-bv", str(int(args.num_workers_bv)),
        "--num-workers-av", str(int(args.num_workers_av)),
        "--final-output-root", output_root,
    ]
    if int(args.scene_limit) > 0:
        cmd.extend(["--scene-limit", str(int(args.scene_limit))])

    try:
        _run_cmd(cmd)
    finally:
        if (not args.keep_stage_root) and os.path.isdir(stage_base):
            shutil.rmtree(stage_base, ignore_errors=True)

    print("[DONE] integrated scoring finished. Only final report csv files are kept.")


if __name__ == "__main__":
    main()
