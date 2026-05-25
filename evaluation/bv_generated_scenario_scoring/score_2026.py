#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
onsite-test 2026 评分入口。

设计目标：
1) 兼容 replay_run_2026.py 的内存调用与文件调用；
2) 兼容历史 score.py 接口，便于老脚本平滑迁移；
3) 对官方场景清单执行“缺失即0分”策略，保证评测口径稳定。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
from datetime import datetime


ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))

# 评分模板：default 用于完整分数，bg_only 用于2026精简输出。
SCORE_PROFILE_CONFIG = {
    "default": {
        "weights": {"safety": 30.0, "realism": 20.0, "comfort": 10.0, "test": 40.0},
        "fieldnames": [
            "Scene ID",
            "Is Collision",
            "Is Out Dynamic",
            "Realism Raw",
            "Comfort Raw",
            "Safety Score",
            "Realism Score",
            "Comfort Score",
            "Final Score",
            "Planner Avg Score",
            "GT Planner Avg Score",
            "Planner Count",
        ],
    },
    "bg_only": {
        "weights": {"safety": 20.0, "realism": 0.0, "comfort": 10.0, "test": 30.0},
        "fieldnames": [
            "Scene ID",
            "Is Collision",
            "Is Out Dynamic",
            "Comfort Raw",
            "Safety Score",
            "Comfort Score",
            "Final Score",
            "Planner Avg Score",
            "GT Planner Avg Score",
            "Planner Count",
        ],
    },
}


# -----------------------------------------------------------------------------
# Basic helpers
# -----------------------------------------------------------------------------

def _safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _resolve_path(path, default_path=""):
    """Resolve a possibly relative project path in a robust way."""
    candidate = str(path or "").strip()
    if not candidate:
        candidate = str(default_path or "").strip()
    if not candidate:
        return ""

    candidates = []
    if os.path.isabs(candidate):
        candidates.append(candidate)
    else:
        # Prefer script-relative paths because this script is usually imported dynamically
        # from replay_run_2026.py, and cwd is not always onsite-test/.
        candidates.append(os.path.join(SCRIPT_DIR, candidate))
        candidates.append(os.path.join(ROOT_DIR, candidate))
        candidates.append(os.path.abspath(candidate))

    for item in candidates:
        if os.path.exists(item):
            return os.path.abspath(item)
    # Return the first normalized path even if it does not exist, so error messages are clear.
    return os.path.abspath(candidates[0]) if candidates else ""


def _load_json(path):
    with open(path, "r") as f:
        return json.load(f)


# -----------------------------------------------------------------------------
# Historical test-value formula and GT score loading
# -----------------------------------------------------------------------------

def _calculate_scene_score_v2(gen_score, gt_score):
    """
    Historical test-value scoring formula.

    Return range: [0, 1]. The caller multiplies it by the configured test-score
    weight, e.g. 40 in default mode or 30 in bg_only mode.
    """
    split_score = 0.5
    gen_score = _safe_float(gen_score, 0.0)
    gt_score = _safe_float(gt_score, 0.0)
    if gt_score <= 0:
        return 0.0
    if gen_score < gt_score:
        return ((gt_score - gen_score) / gt_score) * (1 - split_score) + split_score
    if gt_score >= 80:
        return 0.0
    interval = int(gt_score // 8)
    return split_score - interval * split_score / 10


# Public alias kept for compatibility with legacy score.py imports.
calculate_scene_score_v2 = _calculate_scene_score_v2


def _extract_planner_avg(scene_result):
    if not isinstance(scene_result, dict):
        return None
    if "GT Planner Avg Score" in scene_result:
        return _safe_float(scene_result.get("GT Planner Avg Score", 0.0), 0.0)
    if "Planner Avg Score" in scene_result:
        return _safe_float(scene_result.get("Planner Avg Score", 0.0), 0.0)

    planner_totals = []
    for key, value in scene_result.items():
        if not str(key).startswith("planner_") or not isinstance(value, dict):
            continue
        planner_totals.append(
            _safe_float(value.get("safe", 0.0), 0.0)
            + _safe_float(value.get("efficiency", 0.0), 0.0)
            + _safe_float(value.get("comfortable", 0.0), 0.0)
        )
    if planner_totals:
        return sum(planner_totals) / float(len(planner_totals))
    return None


def _select_split_dict(data, split):
    """Select a split dict from several common json layouts."""
    if not isinstance(data, dict):
        return {}

    split_key = str(split or "").strip().upper()
    if split_key and isinstance(data.get(split_key), dict):
        split_data = data.get(split_key)
        # Support {"A": {"scores": {...}}} and {"A": {"scenes": [...], "scores": {...}}}.
        if isinstance(split_data.get("scores"), dict):
            return split_data.get("scores")
        # Support {"A": {"scenario_x": {...}}}.
        return split_data

    # Support flat {"scenario_x": {...}}.
    if any(str(k).startswith("scenario_") for k in data.keys()):
        return data

    return {}


def _default_gt_score_paths():
    return [
        os.path.join("src", "gt_score_ABC.json"),
        os.path.join("src", "gt_score.json"),
    ]


def _build_gt_planner_avg_map_from_src(gt_score_json="", split="", allow_scene_ids=None):
    """
    Load GT Planner Avg Score map.

    Supports:
    - src/gt_score_ABC.json: {"A": {"scenario_x": {...}}, "B": {...}}
    - src/gt_score_ABC.json: {"A": {"scores": {"scenario_x": {...}}}}
    - legacy src/gt_score.json: {"scenario_x": {"planner_0": {...}}}
    """
    candidate_paths = []
    if str(gt_score_json or "").strip():
        candidate_paths.append(gt_score_json)
    else:
        candidate_paths.extend(_default_gt_score_paths())

    data = None
    for path in candidate_paths:
        resolved = _resolve_path(path)
        if not resolved or not os.path.exists(resolved):
            continue
        try:
            data = _load_json(resolved)
            break
        except Exception:
            data = None

    if not isinstance(data, dict):
        return {}

    scene_dict = _select_split_dict(data, split)
    if not scene_dict:
        return {}

    allow = None
    if allow_scene_ids:
        allow = set(str(x) for x in allow_scene_ids if str(x).strip())

    out = {}
    for scene_id, scene_result in scene_dict.items():
        scene_id = str(scene_id)
        if allow is not None and scene_id not in allow:
            continue
        avg = _extract_planner_avg(scene_result)
        if avg is not None:
            out[scene_id] = float(avg)

    return out


def read_gt_safety_scores(competition_type=None, gt_score_json=""):
    """Legacy public API: return {scene_id: GT planner average score}."""
    return _build_gt_planner_avg_map_from_src(
        gt_score_json=gt_score_json,
        split=competition_type,
    )


# -----------------------------------------------------------------------------
# Official scene list support from legacy score.py
# -----------------------------------------------------------------------------

def _load_official_scene_ids(abc_scenes_json="", split=""):
    split_key = str(split or "").strip().upper()
    if not split_key:
        return []

    path = _resolve_path(abc_scenes_json, os.path.join("src", "ABC_scenes.json"))
    if not path or not os.path.exists(path):
        return []

    try:
        data = _load_json(path)
    except Exception:
        return []

    split_data = data.get(split_key) if isinstance(data, dict) else None
    if isinstance(split_data, dict):
        scenes = split_data.get("scenes", [])
    elif isinstance(split_data, list):
        scenes = split_data
    else:
        scenes = []

    return [str(x) for x in scenes if str(x).strip()]


def _infer_split_from_output_dir(output_dir, abc_scenes_json=""):
    path = _resolve_path(abc_scenes_json, os.path.join("src", "ABC_scenes.json"))
    keys = ["A", "B", "C"]
    if os.path.exists(path):
        try:
            data = _load_json(path)
            if isinstance(data, dict) and data:
                keys = [str(k).upper() for k in data.keys()]
        except Exception:
            pass

    leaf = os.path.basename(os.path.abspath(output_dir or ".").rstrip(os.sep)).upper()
    for key in keys:
        if leaf == key or leaf.startswith(key + "_") or leaf.startswith(key + "-"):
            return key
    return ""


def _official_scene_pack(results, split="", abc_scenes_json="", official_scene_ids=None, include_invalid_generated=True):
    """
    Return (scene_ids_for_rows, official_ids_set, official_ids_for_average).
    If official_scene_ids exists, missing official scenes are explicitly included as zero rows.
    """
    generated_ids = [str(x) for x in sorted(results.keys())]

    if official_scene_ids is None:
        official_scene_ids = _load_official_scene_ids(abc_scenes_json=abc_scenes_json, split=split)

    official_scene_ids = [str(x) for x in (official_scene_ids or []) if str(x).strip()]
    if not official_scene_ids:
        return generated_ids, set(), []

    official_set = set(official_scene_ids)
    row_ids = list(official_scene_ids)
    if include_invalid_generated:
        invalid_generated = [sid for sid in generated_ids if sid not in official_set]
        row_ids.extend(invalid_generated)
    return row_ids, official_set, list(official_scene_ids)


# -----------------------------------------------------------------------------
# Planner result aggregation and per-scene scoring
# -----------------------------------------------------------------------------

def _planner_sort_key(key):
    if str(key).startswith("planner_"):
        try:
            return (0, int(str(key).split("_", 1)[1]))
        except Exception:
            return (0, str(key))
    return (1, str(key))


def _collect_planner_keys(results_dict):
    keys = set()
    for _, scene_result in results_dict.items():
        if not isinstance(scene_result, dict):
            continue
        for key, value in scene_result.items():
            if str(key).startswith("planner_") and isinstance(value, dict):
                keys.add(str(key))
    return sorted(keys, key=_planner_sort_key)


def _planner_stats(scene_result, planner_keys):
    planner_values = {}
    planner_totals = []

    for planner_key in planner_keys:
        planner_data = scene_result.get(planner_key, {}) if isinstance(scene_result, dict) else {}
        if not isinstance(planner_data, dict):
            planner_data = {}

        p_safe = _safe_float(planner_data.get("safe", 0.0), 0.0)
        p_eff = _safe_float(planner_data.get("efficiency", 0.0), 0.0)
        p_comf = _safe_float(planner_data.get("comfortable", 0.0), 0.0)
        p_total = p_safe + p_eff + p_comf

        planner_values[planner_key + "_safe"] = round(p_safe, 4)
        planner_values[planner_key + "_efficiency"] = round(p_eff, 4)
        planner_values[planner_key + "_comfortable"] = round(p_comf, 4)
        planner_values[planner_key + "_total"] = round(p_total, 4)
        planner_totals.append(p_total)

    planner_avg_score = sum(planner_totals) / float(len(planner_totals)) if planner_totals else 0.0
    return planner_values, planner_avg_score, len(planner_totals)


def _get_score_profile(profile, include_test_score, include_valid_submit=False):
    """
    解析评分模板，返回：模板名、权重配置、CSV字段。
    这里把模板定义集中到 SCORE_PROFILE_CONFIG，减少重复与改错风险。
    """
    profile = str(profile or "default").strip().lower()
    if profile not in SCORE_PROFILE_CONFIG:
        profile = "default"
    weights = dict(SCORE_PROFILE_CONFIG[profile]["weights"])
    fieldnames = list(SCORE_PROFILE_CONFIG[profile]["fieldnames"])

    if include_test_score:
        fieldnames.insert(fieldnames.index("Final Score"), "Test Score")
    if include_valid_submit:
        fieldnames.append("Is Valid Submit")
    return profile, weights, fieldnames


def _clamp_score(value):
    # Keep the historical formula but avoid negative values caused by unusual raw metrics.
    return max(0.0, float(value))


def _zero_score_row(scene_id, planner_keys, include_test_score=True, include_valid_submit=False):
    row = {
        "Scene ID": scene_id,
        "Is Collision": 1,
        "Is Out Dynamic": 0,
        "Realism Raw": 0.0,
        "Comfort Raw": 5.0,
        "Safety Score": 0.0,
        "Realism Score": 0.0,
        "Comfort Score": 0.0,
        "Final Score": 0.0,
        "Planner Avg Score": 0.0,
        "GT Planner Avg Score": 0.0,
        "Planner Count": 0,
    }
    if include_test_score:
        row["Test Score"] = 0.0
    if include_valid_submit:
        row["Is Valid Submit"] = 0
    for planner_key in planner_keys:
        row[planner_key + "_safe"] = 0.0
        row[planner_key + "_efficiency"] = 0.0
        row[planner_key + "_comfortable"] = 0.0
        row[planner_key + "_total"] = 0.0
    return row


def score_scene(
    scene_id,
    scene_result,
    planner_keys,
    include_test_score=True,
    gt_planner_avg=0.0,
    score_weights=None,
    include_valid_submit=False,
    is_valid_submit=1,
    force_zero=False,
):
    """
    计算单场景分数并返回一行CSV字典。

    口径说明：
    - is_out_dynamic=1 时整场直接置零；
    - Test Score 只在“无碰撞且有GT基准分”时生效；
    - force_zero=True 用于官方场景缺失/无效补零。
    """
    if force_zero:
        return _zero_score_row(
            scene_id=scene_id,
            planner_keys=planner_keys,
            include_test_score=include_test_score,
            include_valid_submit=include_valid_submit,
        )

    scene_result = scene_result if isinstance(scene_result, dict) else {}

    is_collision = _safe_int(scene_result.get("is_collision", 1), 1)
    is_out_dynamic = _safe_int(scene_result.get("is_out_dynamic", 0), 0)
    realism_raw = _safe_float(scene_result.get("realism_score", 0.0), 0.0)
    comfort_raw = _safe_float(scene_result.get("comfort_score", 5.0), 5.0)

    planner_values, planner_avg_score, planner_count = _planner_stats(scene_result, planner_keys)
    gt_planner_avg = _safe_float(gt_planner_avg, 0.0)
    test_score = 0.0

    weights = score_weights or {
        "safety": 30.0,
        "realism": 20.0,
        "comfort": 10.0,
        "test": 40.0,
    }

    if is_out_dynamic == 1:
        safety_score = 0.0
        realism_score = 0.0
        comfort_score = 0.0
        final_score = 0.0
    else:
        safety_score = float(weights.get("safety", 0.0)) if is_collision == 0 else 0.0
        realism_weight = float(weights.get("realism", 0.0))
        realism_score = (1.0 - math.sqrt(max(0.0, realism_raw))) * realism_weight if realism_weight > 0 else 0.0
        comfort_weight = float(weights.get("comfort", 0.0))
        comfort_score = comfort_weight - (comfort_raw / 5.0) * comfort_weight if comfort_weight > 0 else 0.0
        if include_test_score and is_collision == 0 and gt_planner_avg > 0:
            test_score = _calculate_scene_score_v2(planner_avg_score, gt_planner_avg) * float(weights.get("test", 0.0))
        safety_score = _clamp_score(safety_score)
        realism_score = _clamp_score(realism_score)
        comfort_score = _clamp_score(comfort_score)
        test_score = _clamp_score(test_score)
        final_score = safety_score + realism_score + comfort_score + test_score

    row = {
        "Scene ID": scene_id,
        "Is Collision": is_collision,
        "Is Out Dynamic": is_out_dynamic,
        "Realism Raw": round(realism_raw, 6),
        "Comfort Raw": round(comfort_raw, 6),
        "Safety Score": round(safety_score, 2),
        "Realism Score": round(realism_score, 2),
        "Comfort Score": round(comfort_score, 2),
        "Final Score": round(final_score, 2),
        "Planner Avg Score": round(planner_avg_score, 2),
        "GT Planner Avg Score": round(gt_planner_avg, 2),
        "Planner Count": planner_count,
    }
    if include_test_score:
        row["Test Score"] = round(test_score, 2)
    if include_valid_submit:
        row["Is Valid Submit"] = int(is_valid_submit)
    row.update(planner_values)
    return row


# -----------------------------------------------------------------------------
# CSV output
# -----------------------------------------------------------------------------

def _mean(rows, key, denominator=None):
    if not rows:
        return 0.0
    values = [_safe_float(row.get(key, 0.0), 0.0) for row in rows]
    if denominator is None:
        denominator = len(rows)
    if int(denominator) <= 0:
        return 0.0
    return sum(values) / float(denominator)


def _average_source_rows(rows, average_scene_ids=None):
    if not average_scene_ids:
        return list(rows)
    allowed = set(str(x) for x in average_scene_ids)
    return [row for row in rows if str(row.get("Scene ID", "")) in allowed]


def build_average_row(rows, fieldnames, denominator=None, average_scene_ids=None):
    avg_rows = _average_source_rows(rows, average_scene_ids=average_scene_ids)
    if denominator is None:
        denominator = len(avg_rows)

    avg_row = dict((key, "") for key in fieldnames)
    avg_row["Scene ID"] = "Average"

    avg_keys = [
        "Safety Score",
        "Realism Score",
        "Comfort Score",
        "Test Score",
        "Final Score",
        "Planner Avg Score",
        "GT Planner Avg Score",
        "Is Collision",
        "Is Out Dynamic",
        "Realism Raw",
        "Comfort Raw",
        "Planner Count",
        "Is Valid Submit",
    ]

    for key in fieldnames:
        if key in avg_keys or key.endswith("_safe") or key.endswith("_efficiency") or key.endswith("_comfortable") or key.endswith("_total"):
            avg_row[key] = round(_mean(avg_rows, key, denominator=denominator), 2)
    return avg_row


def _build_simple_average_row(rows, fieldnames, denominator=None, average_scene_ids=None):
    avg_rows = _average_source_rows(rows, average_scene_ids=average_scene_ids)
    if denominator is None:
        denominator = len(avg_rows)

    avg_row = dict((key, "") for key in fieldnames)
    avg_row["Scene ID"] = "Average"
    for key in fieldnames:
        if key in ("Safety Score", "Realism Score", "Comfort Score", "Test Score", "Final Score", "Is Valid Submit"):
            avg_row[key] = round(_mean(avg_rows, key, denominator=denominator), 2)
    return avg_row


def _score_detail_from_average(avg_row):
    return (
        _safe_float(avg_row.get("Safety Score", 0.0), 0.0),
        _safe_float(avg_row.get("Realism Score", 0.0), 0.0),
        _safe_float(avg_row.get("Comfort Score", 0.0), 0.0),
        _safe_float(avg_row.get("Test Score", 0.0), 0.0),
        _safe_float(avg_row.get("Final Score", 0.0), 0.0),
    )


def _write_csv(csv_path, fieldnames, rows, avg_row=None):
    out_dir = os.path.dirname(os.path.abspath(csv_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})
        if avg_row is not None:
            writer.writerow({key: avg_row.get(key, "") for key in fieldnames})


def load_results(results_json):
    if not os.path.exists(results_json):
        raise RuntimeError("results.json not found: %s" % results_json)
    data = _load_json(results_json)
    if not isinstance(data, dict):
        raise RuntimeError("results.json format error: top-level must be dict")
    return data


def score_results(
    results_json,
    output_dir,
    include_test_score=True,
    gt_planner_avg_map=None,
    strict_gt_score=False,
    score_profile="default",
    simple_output=True,
    simple_output_dir=None,
    split="",
    abc_scenes_json="",
    official_scene_ids=None,
    include_invalid_generated=True,
):
    results = load_results(results_json)
    return score_results_from_dict(
        results=results,
        output_dir=output_dir,
        include_test_score=include_test_score,
        gt_planner_avg_map=gt_planner_avg_map,
        strict_gt_score=strict_gt_score,
        score_profile=score_profile,
        simple_output=simple_output,
        simple_output_dir=simple_output_dir,
        split=split,
        abc_scenes_json=abc_scenes_json,
        official_scene_ids=official_scene_ids,
        include_invalid_generated=include_invalid_generated,
    )


def score_results_from_dict(
    results,
    output_dir,
    include_test_score=True,
    gt_planner_avg_map=None,
    strict_gt_score=False,
    score_profile="default",
    simple_output=True,
    simple_output_dir=None,
    split="",
    abc_scenes_json="",
    official_scene_ids=None,
    include_invalid_generated=True,
):
    """
    内存字典评分主流程（replay_run_2026.py 主要走这个接口）。
    """
    if not isinstance(results, dict):
        raise RuntimeError("results format error: top-level must be dict")

    if not split:
        split = _infer_split_from_output_dir(output_dir, abc_scenes_json=abc_scenes_json)

    planner_keys = _collect_planner_keys(results)

    row_scene_ids, official_set, average_scene_ids = _official_scene_pack(
        results=results,
        split=split,
        abc_scenes_json=abc_scenes_json,
        official_scene_ids=official_scene_ids,
        include_invalid_generated=include_invalid_generated,
    )
    include_valid_submit = bool(official_set)

    if gt_planner_avg_map is None:
        gt_planner_avg_map = _build_gt_planner_avg_map_from_src(
            gt_score_json="",
            split=split,
            allow_scene_ids=average_scene_ids or None,
        )
    else:
        gt_planner_avg_map = dict(gt_planner_avg_map or {})

    check_scene_ids = average_scene_ids if average_scene_ids else row_scene_ids
    if include_test_score and strict_gt_score:
        missing = [sid for sid in check_scene_ids if sid not in gt_planner_avg_map]
        if missing:
            sample = ", ".join(missing[:10])
            raise RuntimeError(
                "Missing GT Planner Avg Score for %d scenes. Samples: %s"
                % (len(missing), sample)
            )

    profile_name, score_weights, fieldnames = _get_score_profile(
        score_profile,
        include_test_score,
        include_valid_submit=include_valid_submit,
    )

    rows = []
    for scene_id in row_scene_ids:
        scene_id = str(scene_id)
        is_official_mode = bool(official_set)
        is_official_scene = (not is_official_mode) or (scene_id in official_set)
        has_generated_result = scene_id in results
        is_valid_submit = 1 if (is_official_scene and has_generated_result) else 0
        force_zero = not (is_official_scene and has_generated_result)

        rows.append(
            score_scene(
                scene_id,
                results.get(scene_id, {}),
                planner_keys,
                include_test_score=bool(include_test_score),
                gt_planner_avg=gt_planner_avg_map.get(scene_id, 0.0),
                score_weights=score_weights,
                include_valid_submit=include_valid_submit,
                is_valid_submit=is_valid_submit,
                force_zero=force_zero,
            )
        )

    planner_fields = []
    for planner_key in planner_keys:
        planner_fields.extend(
            [
                planner_key + "_safe",
                planner_key + "_efficiency",
                planner_key + "_comfortable",
                planner_key + "_total",
            ]
        )
    if profile_name == "default":
        fieldnames = fieldnames + planner_fields

    average_denominator = len(average_scene_ids) if average_scene_ids else None
    full_avg_row = build_average_row(
        rows,
        fieldnames,
        denominator=average_denominator,
        average_scene_ids=average_scene_ids,
    )
    score_detail = _score_detail_from_average(full_avg_row)

    if simple_output:
        base_dir = os.path.abspath(simple_output_dir) if simple_output_dir else ""
        if base_dir:
            simple_dir = base_dir
        else:
            base_dir = os.path.join(SCRIPT_DIR, "outputs")
            timestamp_dir = datetime.now().strftime("%Y%m%d_%H%M%S")
            simple_dir = os.path.join(base_dir, timestamp_dir)
        os.makedirs(simple_dir, exist_ok=True)

        simple_fields = [
            "Scene ID",
            "Safety Score",
            "Comfort Score",
            "Final Score",
        ]
        if profile_name == "default" and "Realism Score" in fieldnames:
            # Keep simple output compact but still useful for default full score.
            simple_fields.insert(simple_fields.index("Comfort Score"), "Realism Score")
        if include_test_score:
            simple_fields.insert(simple_fields.index("Final Score"), "Test Score")
        if include_valid_submit:
            simple_fields.append("Is Valid Submit")

        simple_avg_row = _build_simple_average_row(
            rows,
            simple_fields,
            denominator=average_denominator,
            average_scene_ids=average_scene_ids,
        )

        detail_csv = os.path.join(simple_dir, "scene_scores.csv")
        summary_csv = os.path.join(simple_dir, "summary_scores.csv")
        _write_csv(detail_csv, simple_fields, rows, avg_row=None)
        _write_csv(summary_csv, simple_fields, [], avg_row=simple_avg_row)

        return {
            "scene_count": len(rows),
            "official_scene_count": len(average_scene_ids),
            "planner_count": len(planner_keys),
            "average_final_score": round(_safe_float(simple_avg_row.get("Final Score", 0.0), 0.0), 2),
            "score_detail": _score_detail_from_average(simple_avg_row),
            "test_value_enabled": bool(include_test_score),
            "gt_planner_avg_count": int(len(gt_planner_avg_map)),
            "summary_csv": summary_csv,
            "score_csv": detail_csv,
            "legacy_scores_csv": "",
        }

    score_dir = os.path.join(output_dir, "score")
    os.makedirs(score_dir, exist_ok=True)

    summary_csv = os.path.join(score_dir, "all_scenes_summary.csv")
    score_csv = os.path.join(score_dir, "score.csv")
    legacy_scores_csv = os.path.join(score_dir, "scores.csv")

    _write_csv(summary_csv, fieldnames, rows, avg_row=full_avg_row)
    _write_csv(score_csv, fieldnames, rows, avg_row=full_avg_row)
    # Legacy alias from score.py; some scripts may still look for scores.csv.
    _write_csv(legacy_scores_csv, fieldnames, rows, avg_row=full_avg_row)

    return {
        "scene_count": len(rows),
        "official_scene_count": len(average_scene_ids),
        "planner_count": len(planner_keys),
        "average_final_score": round(_safe_float(full_avg_row.get("Final Score", 0.0), 0.0), 2),
        "score_detail": score_detail,
        "test_value_enabled": bool(include_test_score),
        "gt_planner_avg_count": int(len(gt_planner_avg_map)),
        "summary_csv": summary_csv,
        "score_csv": score_csv,
        "legacy_scores_csv": legacy_scores_csv,
    }


# -----------------------------------------------------------------------------
# Legacy score.py compatible APIs
# -----------------------------------------------------------------------------

def read_scores_from_json(json_path, competition_type=None):
    results = load_results(json_path)
    official_scene_ids = _load_official_scene_ids(split=competition_type)
    if not official_scene_ids:
        official_scene_ids = sorted(results.keys())
    official_set = set(str(x) for x in official_scene_ids)
    generated_set = set(str(x) for x in results.keys())

    invalid_scene_ids = sorted(generated_set - official_set)
    missing_scene_ids = [sid for sid in official_scene_ids if sid not in generated_set]
    all_invalid_scene_ids = invalid_scene_ids + missing_scene_ids

    gt_avg_planner_scores = read_gt_safety_scores(competition_type)
    gt_scenes_num = len(official_scene_ids)

    avg_planner_scores = {}
    comfort_scores = []
    realism_scores = []
    sv_safety_scores = []
    collision_free_scenes = {}

    planner_keys = _collect_planner_keys(results)
    for scene_id, scene_data in results.items():
        scene_id = str(scene_id)
        if scene_id not in official_set:
            continue
        _, planner_avg_score, _ = _planner_stats(scene_data, planner_keys)
        avg_planner_scores[scene_id] = planner_avg_score
        comfort_scores.append(_safe_float(scene_data.get("comfort_score", 5.0), 5.0))
        realism_scores.append(math.sqrt(max(0.0, _safe_float(scene_data.get("realism_score", 0.0), 0.0))))
        is_collision = _safe_int(scene_data.get("is_collision", 1), 1)
        sv_safety_scores.append(is_collision)
        collision_free_scenes[scene_id] = is_collision == 0

    avg_comfort_score = sum(comfort_scores) / float(gt_scenes_num) if gt_scenes_num and comfort_scores else 0.0
    avg_realism_score = sum(realism_scores) / float(gt_scenes_num) if gt_scenes_num and realism_scores else 0.0
    avg_sv_safety_score = sum(sv_safety_scores) / float(gt_scenes_num) if gt_scenes_num and sv_safety_scores else 0.0

    return (
        avg_planner_scores,
        avg_comfort_score,
        avg_realism_score,
        avg_sv_safety_score,
        collision_free_scenes,
        gt_avg_planner_scores,
        gt_scenes_num,
        all_invalid_scene_ids,
    )


def calculate_test_score(avg_planner_scores, collision_free_scenes, gt_avg_planner_scores, gt_scenes_num):
    total_score = 0.0
    if _safe_int(gt_scenes_num, 0) <= 0:
        return 0.0
    for scenario_id, gen_score in avg_planner_scores.items():
        if scenario_id in gt_avg_planner_scores and collision_free_scenes.get(scenario_id, False):
            total_score += _calculate_scene_score_v2(gen_score, gt_avg_planner_scores[scenario_id])
    return total_score / float(gt_scenes_num)


def calculate_and_save_scene_scores(json_path, output_dir, gt_avg_planner_scores=None, invalid_scene_ids=None):
    """Legacy compatible wrapper around score_results()."""
    split = _infer_split_from_output_dir(output_dir)
    official_scene_ids = _load_official_scene_ids(split=split)
    info = score_results(
        results_json=json_path,
        output_dir=output_dir,
        include_test_score=True,
        gt_planner_avg_map=gt_avg_planner_scores or None,
        strict_gt_score=False,
        score_profile="default",
        simple_output=False,
        split=split,
        official_scene_ids=official_scene_ids or None,
        include_invalid_generated=True,
    )
    return info["average_final_score"], info.get("score_detail", ())


def calculate_final_score(json_path, output_dir=None, competition_type=None, **kwargs):
    """
    历史兼容入口。

    兼容旧版调用签名：
    `calculate_final_score(json_path, output_dir, competition_type)`
    返回值保持 `(final_score, score_detail)`。
    """
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(json_path)) or os.getcwd()

    split = str(kwargs.get("split", competition_type or "") or "").strip().upper()
    if not split:
        split = _infer_split_from_output_dir(output_dir, abc_scenes_json=kwargs.get("abc_scenes_json", ""))

    abc_scenes_json = kwargs.get("abc_scenes_json", "")
    official_scene_ids = kwargs.get("official_scene_ids", None)
    if official_scene_ids is None:
        official_scene_ids = _load_official_scene_ids(abc_scenes_json=abc_scenes_json, split=split)

    gt_score_json = kwargs.get("gt_score_json", "")
    gt_planner_avg_map = kwargs.get("gt_planner_avg_map", None)
    if gt_planner_avg_map is None:
        gt_planner_avg_map = _build_gt_planner_avg_map_from_src(
            gt_score_json=gt_score_json,
            split=split,
            allow_scene_ids=official_scene_ids or None,
        )

    info = score_results(
        results_json=json_path,
        output_dir=output_dir,
        include_test_score=not bool(kwargs.get("disable_test_score", False)),
        gt_planner_avg_map=gt_planner_avg_map,
        strict_gt_score=bool(kwargs.get("strict_gt_score", False)),
        score_profile=kwargs.get("score_profile", "default"),
        simple_output=bool(kwargs.get("simple_output", False)),
        simple_output_dir=kwargs.get("simple_output_dir", None),
        split=split,
        abc_scenes_json=abc_scenes_json,
        official_scene_ids=official_scene_ids or None,
        include_invalid_generated=True,
    )
    return info["average_final_score"], info.get("score_detail", ())


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------

def _resolve_paths(args):
    default_output_dir = os.path.join(ROOT_DIR, "new_workflow", "outputs_2026A")

    output_dir = args.output_dir if args.output_dir else ""
    results_json = args.results_json if args.results_json else ""

    if not results_json and not output_dir:
        output_dir = default_output_dir
        results_json = os.path.join(output_dir, "results.json")
    elif not results_json and output_dir:
        results_json = os.path.join(output_dir, "results.json")
    elif results_json and not output_dir:
        output_dir = os.path.dirname(os.path.abspath(results_json))

    return os.path.abspath(results_json), os.path.abspath(output_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Score 2026 replay results and export scene score CSVs"
    )
    parser.add_argument("--results-json", default="", help="path to replay results.json")
    parser.add_argument("--output-dir", default="", help="output directory containing results.json")
    parser.add_argument(
        "--enable-test-score",
        action="store_true",
        help="deprecated; test score is enabled by default; use --disable-test-score to turn off",
    )
    parser.add_argument("--disable-test-score", action="store_true", help="disable Test Score computation")
    parser.add_argument(
        "--gt-score-json",
        default=os.path.join("src", "gt_score_ABC.json"),
        help="GT score json; relative paths are resolved against onsite-test/",
    )
    parser.add_argument("--split", "--competition-type", dest="split", default="", help="A/B/C split key")
    parser.add_argument(
        "--abc-scenes-json",
        default=os.path.join("src", "ABC_scenes.json"),
        help="official scene list json; used to zero missing/invalid scenes when split is known",
    )
    parser.add_argument(
        "--strict-gt-score",
        action="store_true",
        help="fail if any official scene misses GT Planner Avg Score when test score is enabled",
    )
    parser.add_argument(
        "--score-profile",
        default="default",
        choices=["default", "bg_only"],
        help="default: 30+20+10+40; bg_only: 20+10+30",
    )
    parser.add_argument(
        "--full-output",
        action="store_true",
        help="write full output_dir/score CSVs instead of compact simple output",
    )
    parser.add_argument(
        "--simple-output-dir",
        default="",
        help="directory for simple CSV outputs; default uses onsite-test/outputs/<timestamp>/",
    )

    args = parser.parse_args()
    results_json, output_dir = _resolve_paths(args)

    split = str(args.split or "").strip().upper()
    if not split:
        split = _infer_split_from_output_dir(output_dir, abc_scenes_json=args.abc_scenes_json)

    enable_test_score = not bool(args.disable_test_score)
    official_scene_ids = _load_official_scene_ids(args.abc_scenes_json, split) if split else []

    gt_planner_avg_map = {}
    if enable_test_score:
        gt_planner_avg_map = _build_gt_planner_avg_map_from_src(
            gt_score_json=args.gt_score_json,
            split=split,
            allow_scene_ids=official_scene_ids or None,
        )

    info = score_results(
        results_json=results_json,
        output_dir=output_dir,
        include_test_score=bool(enable_test_score),
        gt_planner_avg_map=gt_planner_avg_map,
        strict_gt_score=bool(args.strict_gt_score),
        score_profile=args.score_profile,
        simple_output=not bool(args.full_output),
        simple_output_dir=args.simple_output_dir,
        split=split,
        abc_scenes_json=args.abc_scenes_json,
        official_scene_ids=official_scene_ids or None,
        include_invalid_generated=True,
    )

    print("[INFO] split:", split or "<none>")
    print("[INFO] scene_count:", info["scene_count"])
    print("[INFO] official_scene_count:", info.get("official_scene_count", 0))
    print("[INFO] planner_count:", info["planner_count"])
    print("[INFO] average_final_score:", info["average_final_score"])
    print("[INFO] test_value_enabled:", info["test_value_enabled"])
    print("[INFO] gt_planner_avg_count:", info.get("gt_planner_avg_count", 0))
    print("[INFO] summary_csv:", info["summary_csv"])
    print("[INFO] score_csv:", info["score_csv"])
    if info.get("legacy_scores_csv"):
        print("[INFO] legacy_scores_csv:", info["legacy_scores_csv"])


if __name__ == "__main__":
    main()
