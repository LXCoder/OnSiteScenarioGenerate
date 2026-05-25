#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
一体化评分脚本：
1) 调用 BV 回放评分（replay_run_2026.py）
2) 调用 AV 主车算法评分（score_ego_effectiveness.py）
3) 合并成每场景 100 分制结果（BV60 + AV40）
4) 输出两个 CSV：
   - per_scene_detailed_100.csv
   - summary_averages_100.csv

输出目录：<project_root>/outputs/<timestamp>_<split>/
"""

from __future__ import print_function

import argparse
import csv
import datetime
import glob
import os
import re
import shutil
import subprocess
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def _abs_from_root(path_value):
    if os.path.isabs(path_value):
        return path_value
    return os.path.abspath(os.path.join(PROJECT_ROOT, path_value))


def _safe_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def _list_subdirs(path):
    if not os.path.isdir(path):
        return set()
    return set(
        os.path.join(path, d)
        for d in os.listdir(path)
        if os.path.isdir(os.path.join(path, d))
    )


def _extract_scene_id_from_name(name):
    m = re.search(r"(scenario_[0-9a-fA-F]+)", str(name))
    if not m:
        return ""
    return m.group(1)


def _iter_traj_csv_files(source_traj_dir):
    """
    统一兼容两类轨迹文件命名：
    1) inference_data*.csv (历史)
    2) *_scenario_xxxxxxxx_output_*.csv (当前 tess_auto)
    """
    patterns = [
        "inference_data*.csv",
        "*scenario_*_output_*.csv",
    ]
    out = []
    seen = set()
    for pat in patterns:
        for p in glob.glob(os.path.join(source_traj_dir, pat)):
            ap = os.path.abspath(p)
            if ap in seen:
                continue
            seen.add(ap)
            out.append(ap)
    out.sort(key=lambda p: os.path.getmtime(p))
    return out


def _is_stable_file(path, stable_seconds):
    try:
        st = os.stat(path)
    except OSError:
        return False
    if st.st_size <= 0:
        return False
    return (time.time() - st.st_mtime) >= float(stable_seconds)


def _count_data_rows(csv_path):
    try:
        with open(csv_path, "r", encoding="utf-8", newline="") as f:
            n = sum(1 for _ in f)
    except Exception:
        return 0
    return max(0, n - 1)


def _pick_run_dir(output_root, split, before_dirs):
    """
    优先找“新产生”的目录；若没有，再找最近的 *_<split> 目录。
    """
    current_dirs = _list_subdirs(output_root)
    candidates = []
    pattern = re.compile(r"^\d{8}_\d{6}_" + re.escape(split) + r"$", re.I)

    for d in current_dirs:
        name = os.path.basename(d)
        if pattern.match(name):
            candidates.append(d)

    new_dirs = [d for d in candidates if d not in before_dirs]
    if new_dirs:
        new_dirs.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return new_dirs[0]

    if candidates:
        candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return candidates[0]

    return ""


def _run_cmd(cmd):
    print("[RUN]", " ".join(cmd))
    subprocess.run(cmd, check=True)


def _collect_gt_scene_ids(gt_split_dir):
    """
    收集 GT 赛题目录的全部场景ID（如 scenario_xxxxxxxx）。
    支持：
    - 子目录场景：gt_split_dir/scenario_xxx/
    - 平铺文件：gt_split_dir/scenario_xxx.xosc 或 scenario_xxx_gt.xosc
    """
    scene_ids = set()
    if not os.path.isdir(gt_split_dir):
        return []

    for name in os.listdir(gt_split_dir):
        if name.startswith("."):
            continue
        p = os.path.join(gt_split_dir, name)
        if os.path.isdir(p):
            has_xosc = any(f.lower().endswith(".xosc") for f in os.listdir(p))
            if has_xosc:
                scene_ids.add(name)
            continue
        if os.path.isfile(p) and name.lower().endswith(".xosc"):
            sid = os.path.splitext(name)[0]
            if sid.endswith("_gt"):
                sid = sid[:-3]
            scene_ids.add(sid)
    return sorted(scene_ids)


def _read_bv_scene_scores(scene_scores_csv):
    """
    读取 BV 的 per-scene 评分：
    Scene ID,Safety Score,Comfort Score,Test Score,Final Score
    """
    data = {}
    with open(scene_scores_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            scene = str(row.get("Scene ID", "")).strip()
            if not scene or scene.upper() == "AVERAGE":
                continue
            data[scene] = {
                "bv_safety_20": _safe_float(row.get("Safety Score", 0.0)),
                "bv_comfort_10": _safe_float(row.get("Comfort Score", 0.0)),
                "bv_test_30": _safe_float(row.get("Test Score", 0.0)),
                "bv_total_60": _safe_float(row.get("Final Score", 0.0)),
            }
    return data


def _read_av_scene_scores(per_scene_csv):
    """
    读取 AV 的 per-scene 四指标：
    csv,scene,safety_10,efficiency_10,comfort_10,compliance_10,total_40,status,error
    """
    data = {}
    with open(per_scene_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            scene = str(row.get("scene", "")).strip()
            if not scene or scene.upper() == "AVERAGE":
                continue
            data[scene] = {
                "av_safety_10": _safe_float(row.get("safety_10", 0.0)),
                "av_efficiency_10": _safe_float(row.get("efficiency_10", 0.0)),
                "av_comfort_10": _safe_float(row.get("comfort_10", 0.0)),
                "av_compliance_10": _safe_float(row.get("compliance_10", 0.0)),
                "av_total_40": _safe_float(row.get("total_40", 0.0)),
                "av_status": str(row.get("status", "")).strip(),
                "av_error": str(row.get("error", "")).strip(),
            }
    return data


def _prepare_av_csv_from_onsite_traj(source_traj_dir, target_csv_dir, min_data_rows):
    """
    将 OnSite 产出的 inference_data*.csv 转为 AV 评分器输入格式：
    replay_scenario_xxxxxxxx_result.csv

    规则：
    - 仅保留数据行 >= min_data_rows 的文件
    - 同一 scene 若有多份，保留数据行更多的；若相同行数，保留较新的
    """
    files = _iter_traj_csv_files(source_traj_dir)

    best_by_scene = {}
    skipped_empty = 0
    skipped_no_scene = 0

    for path in files:
        rows = _count_data_rows(path)
        if rows < int(min_data_rows):
            skipped_empty += 1
            continue

        scene_id = _extract_scene_id_from_name(os.path.basename(path))
        if not scene_id:
            skipped_no_scene += 1
            continue

        mtime = os.path.getmtime(path)
        prev = best_by_scene.get(scene_id)
        if (prev is None) or (rows > prev["rows"]) or (rows == prev["rows"] and mtime > prev["mtime"]):
            best_by_scene[scene_id] = {"path": path, "rows": rows, "mtime": mtime}

    if not os.path.isdir(target_csv_dir):
        os.makedirs(target_csv_dir)

    prepared = []
    for scene_id in sorted(best_by_scene.keys()):
        src = best_by_scene[scene_id]["path"]
        dst = os.path.join(target_csv_dir, "replay_%s_result.csv" % scene_id)
        shutil.copy2(src, dst)
        prepared.append(dst)

    return {
        "source_file_count": len(files),
        "prepared_scene_count": len(prepared),
        "skipped_empty_count": skipped_empty,
        "skipped_no_scene_count": skipped_no_scene,
    }


def _read_first_av_scene_row(per_scene_csv):
    """
    读取 AV 单次输出中的首个有效场景行（非 AVERAGE）。
    返回 dict；若无有效行返回 {}。
    """
    if not os.path.exists(per_scene_csv):
        return {}
    with open(per_scene_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            scene = str(row.get("scene", "")).strip()
            if not scene or scene.upper() == "AVERAGE":
                continue
            return dict(row)
    return {}


def _write_realtime_av_aggregate(per_scene_csv_path, av_scene_map):
    """
    将实时累计的 AV 场景结果写成 per_scene_four_metrics.csv 同结构文件，
    供后续 _build_reports 直接复用。
    """
    out_dir = os.path.dirname(os.path.abspath(per_scene_csv_path))
    if out_dir and (not os.path.isdir(out_dir)):
        os.makedirs(out_dir)

    fields = [
        "csv",
        "scene",
        "safety_10",
        "efficiency_10",
        "comfort_10",
        "compliance_10",
        "total_40",
        "status",
        "error",
    ]

    rows = []
    for scene in sorted(av_scene_map.keys()):
        row = dict(av_scene_map.get(scene) or {})
        rows.append(
            {
                "csv": row.get("csv", ""),
                "scene": scene,
                "safety_10": row.get("safety_10", "0.000"),
                "efficiency_10": row.get("efficiency_10", "0.000"),
                "comfort_10": row.get("comfort_10", "0.000"),
                "compliance_10": row.get("compliance_10", "0.000"),
                "total_40": row.get("total_40", "0.000"),
                "status": row.get("status", ""),
                "error": row.get("error", ""),
            }
        )

    ok_rows = [r for r in rows if str(r.get("status", "")).strip().lower() == "ok"]
    if ok_rows:
        n = float(len(ok_rows))
        avg_row = {
            "csv": "AVERAGE",
            "scene": "AVERAGE",
            "safety_10": round(sum(_safe_float(r.get("safety_10", 0.0)) for r in ok_rows) / n, 3),
            "efficiency_10": round(sum(_safe_float(r.get("efficiency_10", 0.0)) for r in ok_rows) / n, 3),
            "comfort_10": round(sum(_safe_float(r.get("comfort_10", 0.0)) for r in ok_rows) / n, 3),
            "compliance_10": round(sum(_safe_float(r.get("compliance_10", 0.0)) for r in ok_rows) / n, 3),
            "total_40": round(sum(_safe_float(r.get("total_40", 0.0)) for r in ok_rows) / n, 3),
            "status": "ok",
            "error": "",
        }
    else:
        avg_row = {
            "csv": "AVERAGE",
            "scene": "AVERAGE",
            "safety_10": 0.0,
            "efficiency_10": 0.0,
            "comfort_10": 0.0,
            "compliance_10": 0.0,
            "total_40": 0.0,
            "status": "none",
            "error": "",
        }
    rows.append(avg_row)

    with open(per_scene_csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def _write_csv(path, fieldnames, rows):
    out_dir = os.path.dirname(os.path.abspath(path))
    if out_dir and (not os.path.isdir(out_dir)):
        os.makedirs(out_dir)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def _build_reports(split, bv_scene_scores_csv, av_per_scene_csv, final_out_dir, gt_scene_ids):
    bv_map = _read_bv_scene_scores(bv_scene_scores_csv)
    av_map = _read_av_scene_scores(av_per_scene_csv)

    # 评分基准：BV 场景与 AV 场景的并集（任一侧缺失即补 0 分）。
    all_scenes = sorted(set(list(bv_map.keys()) + list(av_map.keys())))
    if not all_scenes:
        # 兜底：当 BV/AV 都为空时，才退回 GT 场景列表（若有）。
        all_scenes = list(gt_scene_ids or [])
    detail_rows = []

    for scene in all_scenes:
        bv = bv_map.get(scene, {})
        av = av_map.get(scene, {})
        bv_safety_20 = _safe_float(bv.get("bv_safety_20", 0.0))
        bv_comfort_10 = _safe_float(bv.get("bv_comfort_10", 0.0))
        bv_test_30 = _safe_float(bv.get("bv_test_30", 0.0))
        bv_total_60 = _safe_float(bv.get("bv_total_60", 0.0))

        av_safety_10 = _safe_float(av.get("av_safety_10", 0.0))
        av_efficiency_10 = _safe_float(av.get("av_efficiency_10", 0.0))
        av_comfort_10 = _safe_float(av.get("av_comfort_10", 0.0))
        av_compliance_10 = _safe_float(av.get("av_compliance_10", 0.0))
        av_total_40 = _safe_float(av.get("av_total_40", 0.0))

        total_100 = round(bv_total_60 + av_total_40, 3)
        detail_rows.append(
            {
                "topic": split,
                "scene": scene,
                "bv_safety_20": round(bv_safety_20, 3),
                "bv_comfort_10": round(bv_comfort_10, 3),
                "bv_test_30": round(bv_test_30, 3),
                "bv_total_60": round(bv_total_60, 3),
                "av_safety_10": round(av_safety_10, 3),
                "av_efficiency_10": round(av_efficiency_10, 3),
                "av_comfort_10": round(av_comfort_10, 3),
                "av_compliance_10": round(av_compliance_10, 3),
                "av_total_40": round(av_total_40, 3),
                "total_100": total_100,
                "av_status": av.get("av_status", ""),
                "av_error": av.get("av_error", ""),
            }
        )

    n = float(len(all_scenes)) if all_scenes else 1.0
    summary_row = {
        "topic": split,
        "scene_count": len(all_scenes),
        "bv_safety_20_avg": round(sum(r["bv_safety_20"] for r in detail_rows) / n, 3),
        "bv_comfort_10_avg": round(sum(r["bv_comfort_10"] for r in detail_rows) / n, 3),
        "bv_test_30_avg": round(sum(r["bv_test_30"] for r in detail_rows) / n, 3),
        "bv_total_60_avg": round(sum(r["bv_total_60"] for r in detail_rows) / n, 3),
        "av_safety_10_avg": round(sum(r["av_safety_10"] for r in detail_rows) / n, 3),
        "av_efficiency_10_avg": round(sum(r["av_efficiency_10"] for r in detail_rows) / n, 3),
        "av_comfort_10_avg": round(sum(r["av_comfort_10"] for r in detail_rows) / n, 3),
        "av_compliance_10_avg": round(sum(r["av_compliance_10"] for r in detail_rows) / n, 3),
        "av_total_40_avg": round(sum(r["av_total_40"] for r in detail_rows) / n, 3),
        "total_100_avg": round(sum(r["total_100"] for r in detail_rows) / n, 3),
    }

    # 在明细文件末尾追加平均行，便于直接在单表查看整体均值。
    detail_rows.append(
        {
            "topic": split,
            "scene": "AVERAGE",
            "bv_safety_20": summary_row["bv_safety_20_avg"],
            "bv_comfort_10": summary_row["bv_comfort_10_avg"],
            "bv_test_30": summary_row["bv_test_30_avg"],
            "bv_total_60": summary_row["bv_total_60_avg"],
            "av_safety_10": summary_row["av_safety_10_avg"],
            "av_efficiency_10": summary_row["av_efficiency_10_avg"],
            "av_comfort_10": summary_row["av_comfort_10_avg"],
            "av_compliance_10": summary_row["av_compliance_10_avg"],
            "av_total_40": summary_row["av_total_40_avg"],
            "total_100": summary_row["total_100_avg"],
            "av_status": "AVERAGE",
            "av_error": "",
        }
    )

    detail_csv = os.path.join(final_out_dir, "per_scene_detailed_100.csv")
    summary_csv = os.path.join(final_out_dir, "summary_averages_100.csv")

    detail_fields = [
        "topic",
        "scene",
        "bv_safety_20",
        "bv_comfort_10",
        "bv_test_30",
        "bv_total_60",
        "av_safety_10",
        "av_efficiency_10",
        "av_comfort_10",
        "av_compliance_10",
        "av_total_40",
        "total_100",
        "av_status",
        "av_error",
    ]
    summary_fields = [
        "topic",
        "scene_count",
        "bv_safety_20_avg",
        "bv_comfort_10_avg",
        "bv_test_30_avg",
        "bv_total_60_avg",
        "av_safety_10_avg",
        "av_efficiency_10_avg",
        "av_comfort_10_avg",
        "av_compliance_10_avg",
        "av_total_40_avg",
        "total_100_avg",
    ]

    _write_csv(detail_csv, detail_fields, detail_rows)
    _write_csv(summary_csv, summary_fields, [summary_row])
    return detail_csv, summary_csv, len(detail_rows), len(bv_map), len(av_map)


def _run_realtime_av_mode(args, split, gt_scene_ids, bv_scene_scores_csv, final_out_dir, av_output_root):
    source_traj_dir = _abs_from_root(args.realtime_source_traj_dir)
    csv_dir = os.path.join(args.traj_root, split)
    if not os.path.isdir(source_traj_dir):
        raise RuntimeError("realtime source traj dir not found: %s" % source_traj_dir)
    if not os.path.isdir(csv_dir):
        os.makedirs(csv_dir)

    processed = set()
    av_scene_map = {}
    agg_per_scene_csv = os.path.join(final_out_dir, "_tmp", "av_outputs", "realtime_per_scene_four_metrics.csv")
    print("[实时] source_traj_dir=%s" % source_traj_dir)
    print("[实时] av_traj_dir=%s" % csv_dir)
    print("[实时] final_out_dir=%s" % final_out_dir)
    print("[实时] 监听中，Ctrl+C 停止。")

    while True:
        files = _iter_traj_csv_files(source_traj_dir)
        handled = 0

        for path in files:
            name = os.path.basename(path)
            if name in processed:
                continue
            if float(args.realtime_start_epoch) > 0:
                try:
                    if os.path.getmtime(path) < float(args.realtime_start_epoch):
                        processed.add(name)
                        continue
                except Exception:
                    processed.add(name)
                    continue
            if not _is_stable_file(path, args.realtime_stable_seconds):
                continue

            rows = _count_data_rows(path)
            if rows < int(args.realtime_min_data_rows):
                processed.add(name)
                try:
                    os.remove(path)
                    print("[实时] 跳过并剔除空轨迹: %s rows=%d" % (name, rows))
                except Exception:
                    print("[实时] 跳过空轨迹: %s rows=%d" % (name, rows))
                continue

            scene_id = _extract_scene_id_from_name(name)
            if not scene_id:
                processed.add(name)
                print("[实时] 跳过无场景ID文件: %s" % name)
                continue

            # 评测器内部会从文件名反推地图目录，必须用简洁标准名：
            # replay_scenario_xxxxxxxx_result.csv
            target_name = "replay_%s_result.csv" % (scene_id,)
            target_path = os.path.join(csv_dir, target_name)
            shutil.copy2(path, target_path)
            print("[实时] 新轨迹 -> %s" % target_path)

            av_before = _list_subdirs(av_output_root)
            av_cmd = [
                sys.executable,
                args.av_script,
                "--csv",
                target_path,
                "--scene-root",
                args.gt_root,
                "--eval-root",
                args.eval_root,
                "--output-dir",
                av_output_root,
                "--topic",
                split,
                "--workers",
                "1",
                "--keep-workdir",
            ]
            _run_cmd(av_cmd)

            av_run_dir = _pick_run_dir(av_output_root, split, av_before)
            if not av_run_dir:
                raise RuntimeError("Cannot find AV output dir for split %s in %s" % (split, av_output_root))
            av_per_scene_csv = os.path.join(av_run_dir, "per_scene_four_metrics.csv")
            if not os.path.exists(av_per_scene_csv):
                raise RuntimeError("AV per_scene_four_metrics.csv not found: %s" % av_per_scene_csv)

            # 读取本次单文件结果并累计（按 scene 覆盖更新）
            single_row = _read_first_av_scene_row(av_per_scene_csv)
            if single_row:
                current_scene = str(single_row.get("scene", "")).strip() or scene_id
                av_scene_map[current_scene] = single_row
            else:
                av_scene_map[scene_id] = {
                    "csv": os.path.basename(target_path),
                    "scene": scene_id,
                    "safety_10": "0.000",
                    "efficiency_10": "0.000",
                    "comfort_10": "0.000",
                    "compliance_10": "0.000",
                    "total_40": "0.000",
                    "status": "failed",
                    "error": "empty av result",
                }

            _write_realtime_av_aggregate(agg_per_scene_csv, av_scene_map)
            detail_csv, summary_csv, _, _, _ = _build_reports(
                split=split,
                bv_scene_scores_csv=bv_scene_scores_csv,
                av_per_scene_csv=agg_per_scene_csv,
                final_out_dir=final_out_dir,
                gt_scene_ids=gt_scene_ids,
            )
            print("[实时] 已刷新一体化结果: %s | %s" % (detail_csv, summary_csv))

            processed.add(name)
            handled += 1

        if handled == 0:
            if args.realtime_once:
                print("[实时] once 模式结束（本轮无新增可处理文件）。")
                return
            time.sleep(float(args.realtime_poll_seconds))
        elif args.realtime_once:
            print("[实时] once 模式结束（完成一轮处理）。")
            return


def main():
    parser = argparse.ArgumentParser(description="Run BV+AV integrated scoring and export 100-point reports.")
    parser.add_argument("--splits", default="A,B,C", help="Comma-separated splits, e.g. A,B,C")
    parser.add_argument("--scene-limit", type=int, default=0, help="Replay scene limit per split; 0 means all")
    parser.add_argument("--num-workers-bv", type=int, default=4, help="Worker count for replay_run_2026.py")
    parser.add_argument("--num-workers-av", type=int, default=4, help="Worker count for score_ego_effectiveness.py")

    parser.add_argument(
        "--bv-script",
        default="bv_generated_scenario_scoring/replay_run_2026.py",
    )
    parser.add_argument(
        "--av-script",
        default="av_ego_scoring/score_ego_effectiveness.py",
    )
    parser.add_argument(
        "--raw-scene-root",
        default="bv_generated_scenario_scoring/scenario",
    )
    parser.add_argument(
        "--gt-root",
        default="groundtruth31_exam_gt_merged_20260509_163645",
    )
    parser.add_argument(
        "--traj-root",
        default="av_ego_scoring/traj",
    )
    parser.add_argument(
        "--av-source-traj-dir",
        default="",
        help="可选：直接从 OnSite traj_output 目录提取 inference_data*.csv 并生成 AV 输入",
    )
    parser.add_argument(
        "--av-source-min-data-rows",
        type=int,
        default=2,
        help="当使用 --av-source-traj-dir 时，最少轨迹数据行数",
    )
    parser.add_argument(
        "--eval-root",
        default="av_ego_scoring/EvaluationSystem-onsite2025-116e55243d4a177fe72a20f81da9dfd43f78f029",
    )
    parser.add_argument(
        "--final-output-root",
        default="outputs",
        help="Integrated final output root",
    )
    parser.add_argument(
        "--realtime-av",
        action="store_true",
        help="(deprecated) 实时模式已废弃，请使用离线一体化流程",
    )
    parser.add_argument(
        "--realtime-source-traj-dir",
        default="tess_auto/OnSiteScenarioGenerate/traj_output",
        help="OnSiteScenarioGenerate 轨迹输出目录",
    )
    parser.add_argument("--realtime-poll-seconds", type=float, default=2.0, help="实时扫描间隔秒")
    parser.add_argument("--realtime-stable-seconds", type=float, default=2.0, help="文件稳定时间秒")
    parser.add_argument("--realtime-min-data-rows", type=int, default=1, help="最少轨迹数据行")
    parser.add_argument("--realtime-start-epoch", type=float, default=0.0, help="仅处理 mtime>=该时间戳(秒)的轨迹文件")
    parser.add_argument("--realtime-once", action="store_true", help="实时模式只扫描/处理一轮后退出")
    args = parser.parse_args()

    if args.realtime_av:
        raise RuntimeError("realtime mode is deprecated in this deployment. Use offline pipeline only.")

    # 统一转换为绝对路径（相对路径按脚本所在目录解析，便于部署）
    args.bv_script = _abs_from_root(args.bv_script)
    args.av_script = _abs_from_root(args.av_script)
    args.raw_scene_root = _abs_from_root(args.raw_scene_root)
    args.gt_root = _abs_from_root(args.gt_root)
    args.traj_root = _abs_from_root(args.traj_root)
    args.eval_root = _abs_from_root(args.eval_root)
    args.final_output_root = _abs_from_root(args.final_output_root)
    args.realtime_source_traj_dir = _abs_from_root(args.realtime_source_traj_dir)
    if args.av_source_traj_dir:
        args.av_source_traj_dir = _abs_from_root(args.av_source_traj_dir)

    splits = []
    for part in str(args.splits).split(","):
        s = str(part).strip().upper()
        if s in ("A", "B", "C") and s not in splits:
            splits.append(s)
    if not splits:
        raise RuntimeError("No valid split in --splits. Use A/B/C.")

    run_ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    for split in splits:
        raw_scene_dir = os.path.join(args.raw_scene_root, split)
        gt_split_dir = os.path.join(args.gt_root, split)
        final_out_dir = os.path.join(args.final_output_root, run_ts + "_" + split)
        tmp_root = os.path.join(final_out_dir, "_tmp")
        bv_output_root = os.path.join(tmp_root, "bv_outputs")
        av_output_root = os.path.join(tmp_root, "av_outputs")
        csv_dir = os.path.join(args.traj_root, split)

        if not os.path.isdir(raw_scene_dir):
            raise RuntimeError("raw scene dir not found: %s" % raw_scene_dir)
        if not os.path.isdir(gt_split_dir):
            raise RuntimeError("gt split dir not found: %s" % gt_split_dir)
        if not args.realtime_av and args.av_source_traj_dir:
            if not os.path.isdir(args.av_source_traj_dir):
                raise RuntimeError("av source traj dir not found: %s" % args.av_source_traj_dir)
            staged_root = os.path.join(tmp_root, "offline_av_input")
            csv_dir = os.path.join(staged_root, split)
            stats = _prepare_av_csv_from_onsite_traj(
                source_traj_dir=args.av_source_traj_dir,
                target_csv_dir=csv_dir,
                min_data_rows=args.av_source_min_data_rows,
            )
            print(
                "[离线AV预处理] split=%s source_files=%d prepared_scenes=%d skipped_empty=%d skipped_no_scene=%d"
                % (
                    split,
                    stats["source_file_count"],
                    stats["prepared_scene_count"],
                    stats["skipped_empty_count"],
                    stats["skipped_no_scene_count"],
                )
            )
            if int(stats["prepared_scene_count"]) <= 0:
                raise RuntimeError(
                    "No valid AV input CSV prepared from source dir: %s (min rows=%d)"
                    % (args.av_source_traj_dir, int(args.av_source_min_data_rows))
                )
        else:
            if not os.path.isdir(csv_dir):
                raise RuntimeError("traj csv dir not found: %s" % csv_dir)
        if not os.path.isdir(final_out_dir):
            os.makedirs(final_out_dir)

        gt_scene_ids = _collect_gt_scene_ids(gt_split_dir)

        # 1) BV
        bv_before = _list_subdirs(bv_output_root)
        bv_cmd = [
            sys.executable,
            args.bv_script,
            "--raw-scene-dir", raw_scene_dir,
            "--gt-raw-scene-dir", gt_split_dir,
            "--num-workers", str(int(args.num_workers_bv)),
            "--output-dir", bv_output_root,
        ]
        if int(args.scene_limit) > 0:
            bv_cmd.extend(["--scene-limit", str(int(args.scene_limit))])
        _run_cmd(bv_cmd)

        bv_run_dir = _pick_run_dir(bv_output_root, split, bv_before)
        if not bv_run_dir:
            raise RuntimeError("Cannot find BV output dir for split %s in %s" % (split, bv_output_root))
        bv_scene_scores_csv = os.path.join(bv_run_dir, "scene_scores.csv")
        if not os.path.exists(bv_scene_scores_csv):
            raise RuntimeError("BV scene_scores.csv not found: %s" % bv_scene_scores_csv)

        # 2) AV
        av_before = _list_subdirs(av_output_root)
        av_cmd = [
            sys.executable,
            args.av_script,
            "--csv-dir", csv_dir,
            "--scene-root", args.gt_root,
            "--eval-root", args.eval_root,
            "--output-dir", av_output_root,
            "--topic", split,
            "--workers", str(int(args.num_workers_av)),
        ]
        _run_cmd(av_cmd)

        av_run_dir = _pick_run_dir(av_output_root, split, av_before)
        if not av_run_dir:
            raise RuntimeError("Cannot find AV output dir for split %s in %s" % (split, av_output_root))
        av_per_scene_csv = os.path.join(av_run_dir, "per_scene_four_metrics.csv")
        if not os.path.exists(av_per_scene_csv):
            raise RuntimeError("AV per_scene_four_metrics.csv not found: %s" % av_per_scene_csv)

        # 3) 合并输出（100分制）
        detail_csv, summary_csv, merged_scene_count, bv_scored_scene_count, av_scored_scene_count = _build_reports(
            split=split,
            bv_scene_scores_csv=bv_scene_scores_csv,
            av_per_scene_csv=av_per_scene_csv,
            final_out_dir=final_out_dir,
            gt_scene_ids=gt_scene_ids,
        )

        # 一体化运行完成后，删除 BV/AV 中间文件，仅保留 final_out_dir 的最终 CSV。
        if os.path.isdir(tmp_root):
            shutil.rmtree(tmp_root, ignore_errors=True)

        print("[OK] split=%s" % split)
        print("[OK] detail_csv=%s" % detail_csv)
        print("[OK] summary_csv=%s" % summary_csv)


if __name__ == "__main__":
    main()
