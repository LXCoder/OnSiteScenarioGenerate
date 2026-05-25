#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Generate replay visualization files for 2026 outputs.

Default behavior:
- read replay csvs from output_dir
- load scene info from --raw-scene-dir (xosc/xodr)
- export GIF files without vehicle head guide lines
"""

from __future__ import print_function

import argparse
import csv
import glob
import inspect
import multiprocessing
import os
import re
import sys

# 项目根目录（用于拼接默认输出路径）
ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG
from utils.visualizer import Visualizer


_WORKER_SCENE_MAP = None
_WORKER_VIS_DIR = None


def _parse_scene_name_from_csv(csv_path):
    name = os.path.basename(csv_path)
    match = re.search(r"(scenario_[0-9a-zA-Z]+)_result\.csv$", name)
    if match:
        return match.group(1)
    return None


def _load_scene_map(workspace_dir):
    scenario_dir = os.path.join(workspace_dir, "scenario")
    gt_dir = os.path.join(workspace_dir, "ground_truth")
    config = {
        "tasks": [],
        "warmup": 1,
        "skipExist": True,
        "visualize": False,
    }
    sm = ScenarioManagerForISG(scenario_dir, gt_dir, config)
    return dict((scene.name, scene) for scene in sm.tasks)


def _load_scene_map_from_raw(raw_scene_dir, warmup=1):
    # 复用 replay_run_2026 中的原始 xosc/xodr 单场景构造逻辑
    from replay_run_2026 import _discover_raw_scene_dirs, _load_single_raw_task

    scene_map = {}
    for scene_dir in _discover_raw_scene_dirs(raw_scene_dir):
        scene_name = os.path.basename(scene_dir.rstrip(os.sep))
        try:
            scene_map[scene_name] = _load_single_raw_task(raw_scene_dir, scene_name, warmup)
        except Exception:
            continue
    return scene_map


def _to_float_or_none(value):
    try:
        return float(value)
    except Exception:
        return None


def _load_allowed_scenes_by_score(score_csv_path, planner_avg_min=None, planner_avg_max=None):
    # 根据 score.csv 中的 Planner Avg Score 做可视化过滤
    if not score_csv_path or (planner_avg_min is None and planner_avg_max is None):
        return None
    if not os.path.exists(score_csv_path):
        print("[WARNING] score csv not found, skip score filter:", score_csv_path)
        return None

    allowed = set()
    with open(score_csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            scene_id = (row.get("Scene ID") or "").strip()
            if not scene_id or scene_id.lower() == "average":
                continue
            score_val = _to_float_or_none(row.get("Planner Avg Score"))
            if score_val is None:
                continue
            if planner_avg_min is not None and score_val < planner_avg_min:
                continue
            if planner_avg_max is not None and score_val > planner_avg_max:
                continue
            allowed.add(scene_id)
    return allowed


def _default_vis_dir(output_dir):
    output_name = os.path.basename(os.path.abspath(output_dir.rstrip(os.sep)))
    if not output_name:
        output_name = "default_output"
    return os.path.join(ROOT_DIR, "new_workflow", "visualizations_2026", output_name)


def _init_vis_worker(workspace_dir, raw_scene_dir, vis_dir):
    global _WORKER_SCENE_MAP, _WORKER_VIS_DIR
    if raw_scene_dir:
        _WORKER_SCENE_MAP = _load_scene_map_from_raw(raw_scene_dir, warmup=1)
    elif workspace_dir:
        _WORKER_SCENE_MAP = _load_scene_map(workspace_dir)
    else:
        _WORKER_SCENE_MAP = {}
    _WORKER_VIS_DIR = vis_dir


def _render_single_csv(csv_path):
    scene_name = _parse_scene_name_from_csv(csv_path)
    if not scene_name:
        return {"status": "skipped", "csv": csv_path, "reason": "scene_name_parse_failed"}
    if _WORKER_SCENE_MAP is None or scene_name not in _WORKER_SCENE_MAP:
        return {"status": "skipped", "csv": csv_path, "reason": "scene_not_found_in_workspace"}

    gif_name = os.path.splitext(os.path.basename(csv_path))[0] + ".gif"
    gif_path = os.path.join(_WORKER_VIS_DIR, gif_name)

    try:
        vis = Visualizer()
        _run_replay_result_compatible(
            vis=vis,
            result_path=csv_path,
            save_path=gif_path,
            scene_info=_WORKER_SCENE_MAP[scene_name],
        )
        return {"status": "generated", "csv": csv_path, "gif": gif_path}
    except Exception as e:
        return {"status": "failed", "csv": csv_path, "error": repr(e)}


def _run_replay_result_compatible(vis, result_path, save_path, scene_info):
    """
    Backward-compatible wrapper for different Visualizer.replay_result signatures.
    Some legacy versions do not support scene_info kwargs.
    """
    replay_fn = vis.replay_result
    param_names = []
    try:
        sig = inspect.signature(replay_fn)
        param_names = list(sig.parameters.keys())
    except Exception:
        pass

    patch_loader = False
    old_loader = None
    if "scene_info" not in param_names and hasattr(vis, "_load_result_scene"):
        # Legacy visualizer: force loading scene from caller-provided workspace scene.
        old_loader = vis._load_result_scene
        vis._load_result_scene = lambda mode, task: scene_info
        patch_loader = True

    kwargs = {}
    if "scene_info" in param_names:
        kwargs["scene_info"] = scene_info
    if "draw_trajectory_guide" in param_names:
        kwargs["draw_trajectory_guide"] = False

    try:
        if "result_path" in param_names or "save_path" in param_names:
            return replay_fn(result_path=result_path, save_path=save_path, **kwargs)
        return replay_fn(result_path, save_path, **kwargs)
    finally:
        if patch_loader:
            vis._load_result_scene = old_loader


def generate_visualizations(
    output_dir,
    workspace_dir="",
    raw_scene_dir="",
    vis_dir=None,
    max_files=0,
    num_workers=0,
    score_csv_path="",
    planner_avg_min=None,
    planner_avg_max=None,
):
    if vis_dir is None:
        vis_dir = _default_vis_dir(output_dir)
    os.makedirs(vis_dir, exist_ok=True)

    # 仅处理回放输出的 REPLAY_*_result.csv
    csv_files = sorted(glob.glob(os.path.join(output_dir, "REPLAY_*_result.csv")))
    if max_files and max_files > 0:
        csv_files = csv_files[:max_files]

    if not score_csv_path:
        score_csv_path = os.path.join(output_dir, "score", "score.csv")
    allowed_scenes = _load_allowed_scenes_by_score(
        score_csv_path=score_csv_path,
        planner_avg_min=planner_avg_min,
        planner_avg_max=planner_avg_max,
    )
    if allowed_scenes is not None:
        csv_files = [
            p for p in csv_files
            if (_parse_scene_name_from_csv(p) in allowed_scenes)
        ]

    generated = []
    skipped = []
    failed = []

    # 自动推断 worker 数量
    worker_count = int(num_workers) if num_workers is not None else 0
    if worker_count <= 0:
        worker_count = int(os.cpu_count() or 1)
    worker_count = max(1, min(worker_count, len(csv_files) if csv_files else 1))

    start_method = "sequential"
    if worker_count == 1:
        _init_vis_worker(workspace_dir, raw_scene_dir, vis_dir)
        result_iter = [_render_single_csv(csv_path) for csv_path in csv_files]
    else:
        try:
            ctx = multiprocessing.get_context("fork")
            start_method = "fork"
        except ValueError:
            ctx = multiprocessing.get_context("spawn")
            start_method = "spawn"
        with ctx.Pool(
            processes=worker_count,
            initializer=_init_vis_worker,
            initargs=(workspace_dir, raw_scene_dir, vis_dir),
        ) as pool:
            result_iter = pool.imap_unordered(_render_single_csv, csv_files)
            result_iter = list(result_iter)

    for idx, item in enumerate(result_iter):
        status = item.get("status")
        if status == "generated":
            generated.append(item["gif"])
            print("[INFO] (%d/%d) generated: %s" % (idx + 1, len(csv_files), item["gif"]))
        elif status == "skipped":
            skipped.append(item.get("csv"))
            print("[INFO] (%d/%d) skipped: %s (%s)" % (idx + 1, len(csv_files), item.get("csv"), item.get("reason", "")))
        else:
            failed.append({"csv": item.get("csv"), "error": item.get("error", "unknown_error")})
            print("[WARNING] failed: %s | %s" % (item.get("csv"), item.get("error", "unknown_error")))

    return {
        "output_dir": output_dir,
        "vis_dir": vis_dir,
        "num_workers": worker_count,
        "mp_start_method": start_method,
        "input_csv_count": len(csv_files),
        "generated_count": len(generated),
        "skipped_count": len(skipped),
        "failed_count": len(failed),
        "generated_files": generated,
        "skipped_files": skipped,
        "failed_files": failed,
        "score_csv_path": score_csv_path,
        "planner_avg_min": planner_avg_min,
        "planner_avg_max": planner_avg_max,
    }


def main():
    parser = argparse.ArgumentParser(description="Generate 2026 replay visualization GIF files")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(ROOT_DIR, "new_workflow", "outputs_2026A"),
        help="directory containing replay result csv files",
    )
    parser.add_argument(
        "--raw-scene-dir",
        default="",
        help="optional raw scene root containing per-scene xosc/xodr folders (e.g. new_workflow/ground_truth/A)",
    )
    parser.add_argument(
        "--vis-dir",
        default="",
        help="directory to write visualization gif files (default: new_workflow/visualizations_2026/<output-dir-name>)",
    )
    parser.add_argument(
        "--max-files",
        type=int,
        default=0,
        help="limit number of replay csv files to process (0 means all)",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=0,
        help="visualization workers; <=0 means auto(cpu_count)",
    )
    parser.add_argument(
        "--score-csv",
        default="",
        help="score csv path used for filtering (default: <output-dir>/score/score.csv)",
    )
    parser.add_argument(
        "--planner-avg-min",
        type=float,
        default=None,
        help="only visualize scenes with Planner Avg Score >= this value",
    )
    parser.add_argument(
        "--planner-avg-max",
        type=float,
        default=None,
        help="only visualize scenes with Planner Avg Score <= this value",
    )
    args = parser.parse_args()

    info = generate_visualizations(
        workspace_dir="",
        output_dir=args.output_dir,
        raw_scene_dir=args.raw_scene_dir,
        vis_dir=(args.vis_dir if args.vis_dir else None),
        max_files=int(args.max_files),
        num_workers=int(args.num_workers),
        score_csv_path=args.score_csv,
        planner_avg_min=args.planner_avg_min,
        planner_avg_max=args.planner_avg_max,
    )

    print("[INFO] input_csv_count:", info["input_csv_count"])
    print("[INFO] generated_count:", info["generated_count"])
    print("[INFO] skipped_count:", info["skipped_count"])
    print("[INFO] failed_count:", info["failed_count"])
    print("[INFO] num_workers:", info["num_workers"])
    print("[INFO] mp_start_method:", info["mp_start_method"])
    print("[INFO] score_csv_path:", info["score_csv_path"])
    print("[INFO] planner_avg_min:", info["planner_avg_min"])
    print("[INFO] planner_avg_max:", info["planner_avg_max"])
    print("[INFO] vis_dir:", info["vis_dir"])


if __name__ == "__main__":
    main()
