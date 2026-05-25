#!/usr/bin/env python3
"""
批量轨迹打分脚本（使用 gt.xosc + xodr）。

目标：
1. 自动从场景库中按 scene_id 匹配对应的 xodr 与 *_gt.xosc。
2. 调用 EvaluationSystem 的离线评分。
3. 仅保留两个最终结果文件：
   - per_scene_four_metrics.csv（每场景四项 + 总分 + 最后一行均值）
   - four_metrics_averages.csv（仅四项均值 + 总分均值）
4. 输出目录按“时间戳 + 赛题标识（A/B/C）”新建子目录。
5. 不保留 map_stage / trj_stage 临时目录。
"""

import argparse
import csv
import math
import os
import re
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from tempfile import mkdtemp
from typing import Dict, List, NamedTuple, Optional, Set, Tuple


class SceneFiles(NamedTuple):
    """场景文件三元组：场景目录、xodr、gt_xosc。"""

    scene_dir: Path
    scene_name: str
    xodr: Path
    gt_xosc: Path


# =========================
# 参数与路径解析
# =========================

def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    p = argparse.ArgumentParser(description="批量评分 *_result.csv，并输出两份聚合结果。")
    p.add_argument(
        "--csv-dir",
        default="/home/zd/Program/onsite-test/onsite_ego_scoring",
        help="待评分 CSV 所在目录（默认扫描 *_result.csv）",
    )
    p.add_argument(
        "--csv",
        default="",
        help="可选：只评分一个 CSV 文件（优先于 --csv-dir）",
    )
    p.add_argument(
        "--scene-root",
        default="/home/zd/Program/onsite-test/groundtruth31_exam_gt_merged_20260509_163645",
        help="场景根目录（含 A/B/C 子目录）",
    )
    p.add_argument(
        "--eval-root",
        default="/home/zd/Program/onsite-test/onsite_ego_scoring/EvaluationSystem-onsite2025-116e55243d4a177fe72a20f81da9dfd43f78f029",
        help="EvaluationSystem 根目录",
    )
    p.add_argument(
        "--output-dir",
        default="/home/zd/Program/onsite-test/onsite_ego_scoring/outputs",
        help="输出根目录",
    )
    p.add_argument(
        "--topic",
        default="auto",
        choices=["auto", "A", "B", "C"],
        help="赛题标识。auto 自动识别；也可手动指定 A/B/C。",
    )
    p.add_argument(
        "--workers",
        type=int,
        default=1,
        help="外层并行进程数（按 CSV 并行）。1 表示串行，建议 2~8。",
    )
    p.add_argument("--keep-workdir", action="store_true", help="是否保留每个场景临时目录（默认删除）")
    p.add_argument("--dry-run", action="store_true", help="仅做匹配检查，不执行评分")
    return p.parse_args()


def extract_scene_name(csv_name: str) -> str:
    """从 CSV 文件名中提取 scene_xxxxxxxx。"""
    m = re.search(r"(scenario_[0-9a-fA-F]+)", csv_name)
    if not m:
        raise ValueError(f"无法从文件名提取场景ID: {csv_name}")
    return m.group(1)


def extract_scene_type(csv_name: str) -> str:
    """从 CSV 文件名前缀提取场景类型（如 replay/fragment/serial）。"""
    first = csv_name.split("_")[0].strip()
    if not first:
        raise ValueError(f"无法从文件名提取场景类型: {csv_name}")
    return first.lower()


def pick_csv_files(csv_dir: Path, single_csv: str) -> List[Path]:
    """选取待评分 CSV 列表。"""
    if single_csv:
        p = Path(single_csv).resolve()
        if not p.is_file():
            raise FileNotFoundError(f"CSV 不存在: {p}")
        return [p]

    files = sorted(csv_dir.glob("*_result.csv"))
    files = [f for f in files if not f.name.startswith("score") and not f.name.startswith("four_metrics")]
    if not files:
        raise FileNotFoundError(f"目录下未找到 *_result.csv: {csv_dir}")
    return files


# =========================
# 场景匹配与暂存目录构建
# =========================

def find_scene_files(scene_root: Path, scene_name: str) -> SceneFiles:
    """
    在 A/B/C 下查找 scene_name 对应目录，要求至少存在：
    - 1个 .xodr
    - 1个 *_gt.xosc
    """
    candidates = list(scene_root.glob(f"*/{scene_name}"))
    if not candidates:
        raise FileNotFoundError(f"场景目录未找到: {scene_name} (root={scene_root})")

    for scene_dir in candidates:
        xodrs = sorted(scene_dir.glob("*.xodr"))
        gtxoscs = sorted(scene_dir.glob("*_gt.xosc"))
        if xodrs and gtxoscs:
            return SceneFiles(scene_dir=scene_dir, scene_name=scene_name, xodr=xodrs[0], gt_xosc=gtxoscs[0])

    raise FileNotFoundError(
        f"已找到 {len(candidates)} 个目录，但缺少 .xodr 或 *_gt.xosc: {scene_name}"
    )


def stage_map_folder(scene: SceneFiles, scene_type: str, stage_root: Path) -> Path:
    """
    按评分工具要求，构造地图目录结构：
    <stage_root>/<scene_type>/<scene_name>/<scene_name>.xodr
    <stage_root>/<scene_type>/<scene_name>/<scene_name>.xosc  (由 *_gt.xosc 重命名复制)
    """
    dst_scene_dir = stage_root / scene_type / scene.scene_name
    dst_scene_dir.mkdir(parents=True, exist_ok=True)

    shutil.copy2(scene.xodr, dst_scene_dir / f"{scene.scene_name}.xodr")
    shutil.copy2(scene.gt_xosc, dst_scene_dir / f"{scene.scene_name}.xosc")

    # Compatibility for evaluator variants expecting folder name without "scenario_".
    if scene.scene_name.startswith("scenario_"):
        short_name = scene.scene_name[len("scenario_") :]
        if short_name:
            alias_dir = stage_root / scene_type / short_name
            alias_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(scene.xodr, alias_dir / f"{short_name}.xodr")
            shutil.copy2(scene.gt_xosc, alias_dir / f"{short_name}.xosc")
    return stage_root


def stage_single_csv(csv_path: Path, stage_root: Path) -> Path:
    """把单个待评分 CSV 复制到临时 trajectory 目录，并补齐评测器需要的字段。"""
    trj_dir = stage_root / "trajectory"
    trj_dir.mkdir(parents=True, exist_ok=True)
    dst = trj_dir / csv_path.name
    shutil.copy2(csv_path, dst)

    # Compatibility: evaluator expects "simuTime" column.
    # Our generated traj usually has "time" in seconds.
    try:
        import pandas as pd  # already a project dependency

        df = pd.read_csv(dst)
        if "simuTime" not in df.columns and "time" in df.columns:
            t = pd.to_numeric(df["time"], errors="coerce").fillna(0.0)
            df.insert(0, "simuTime", (t * 1000.0).round(3))
            df.to_csv(dst, index=False)
    except Exception:
        # Keep original file if conversion fails; upstream will report details.
        pass
    return trj_dir


def _chmod_readable_tree(root: Path) -> None:
    """Best-effort: make debug/work dirs readable from host user."""
    try:
        os.chmod(str(root), 0o755)
    except Exception:
        return
    for dirpath, dirnames, filenames in os.walk(str(root)):
        for d in dirnames:
            p = os.path.join(dirpath, d)
            try:
                os.chmod(p, 0o755)
            except Exception:
                pass
        for f in filenames:
            p = os.path.join(dirpath, f)
            try:
                os.chmod(p, 0o644)
            except Exception:
                pass


# =========================
# 评分工具导入兼容（缺包兜底）
# =========================

def import_offline_eval(eval_root: Path):
    """
    动态导入 offline_evaluate。
    评分工具在导入阶段会引用 kafka/redis/matlab；离线场景不一定需要，故做最小 stub 兜底。
    """
    sys.path.insert(0, str(eval_root))
    import types

    try:
        import kafka  # type: ignore  # noqa: F401
    except Exception:
        kafka_stub = types.ModuleType("kafka")

        class _KafkaConsumerStub:  # pragma: no cover
            def __init__(self, *args, **kwargs):
                pass

        class _TopicPartitionStub:  # pragma: no cover
            def __init__(self, *args, **kwargs):
                self.topic = args[0] if args else None
                self.partition = args[1] if len(args) > 1 else None

        kafka_stub.KafkaConsumer = _KafkaConsumerStub
        kafka_stub.TopicPartition = _TopicPartitionStub
        sys.modules["kafka"] = kafka_stub

    try:
        import redis  # type: ignore  # noqa: F401
    except Exception:
        redis_stub = types.ModuleType("redis")

        class _RedisClientStub:  # pragma: no cover
            def __init__(self, *args, **kwargs):
                pass

            def __getattr__(self, _):
                def _noop(*args, **kwargs):
                    return None

                return _noop

        redis_stub.Redis = _RedisClientStub
        redis_stub.StrictRedis = _RedisClientStub
        sys.modules["redis"] = redis_stub

    try:
        import matlab  # type: ignore  # noqa: F401
    except Exception:
        import math as _math
        import numpy as _np

        matlab_stub = types.ModuleType("matlab")
        matlab_stub.linspace = _np.linspace
        matlab_stub.sqrt = _math.sqrt
        sys.modules["matlab"] = matlab_stub

    import offline_evaluate  # type: ignore

    return offline_evaluate


def ensure_kb_files(eval_root: Path) -> None:
    """确保评分工具依赖的 testData/KnowledgeBase 下关键文件存在。"""
    src_kb = eval_root / "evaluateUtils" / "TrafficRule_compliance" / "KnowledgeBase"
    dst_kb = eval_root / "testData" / "KnowledgeBase"

    # 兼容符号链接或历史遗留路径
    if dst_kb.is_symlink():
        dst_kb = Path(os.path.realpath(dst_kb))
    elif dst_kb.exists() and not dst_kb.is_dir():
        dst_kb.unlink()

    dst_kb.mkdir(parents=True, exist_ok=True)

    for name in ("lefthand_flag.csv", "china.csv"):
        src = src_kb / name
        dst = dst_kb / name
        if src.is_file() and (not dst.exists()):
            shutil.copy2(src, dst)


# =========================
# 分数归一化与结果输出
# =========================

def clamp_score10(raw_value: Optional[float]) -> float:
    """将分数限制在 0~10，不做缩放。"""
    if raw_value is None:
        return 0.0
    try:
        v = float(raw_value)
    except Exception:
        return 0.0
    if math.isnan(v) or math.isinf(v):
        return 0.0
    v = max(0.0, min(10.0, v))
    return round(v, 3)


def normalize_four_metrics(score_dict: Dict[str, float]) -> Dict[str, float]:
    """提取四项并计算总分（40制，四项等权，每项满分10）。"""
    # 按你的口径：仅效率维度按 100 -> 10 缩放，其余维度直接按 10 分制截断。
    safety10 = clamp_score10(score_dict.get("安全"))
    comfort10 = clamp_score10(score_dict.get("舒适"))
    compliance10 = clamp_score10(score_dict.get("交规符合性"))

    raw_efficiency = score_dict.get("效率")
    if raw_efficiency is None:
        efficiency10 = 0.0
    else:
        try:
            efficiency10 = float(raw_efficiency) / 10.0
        except Exception:
            efficiency10 = 0.0
        if math.isnan(efficiency10) or math.isinf(efficiency10):
            efficiency10 = 0.0
        efficiency10 = round(max(0.0, min(10.0, efficiency10)), 3)

    total40 = round(safety10 + efficiency10 + comfort10 + compliance10, 3)
    return {
        "safety_10": safety10,
        "efficiency_10": efficiency10,
        "comfort_10": comfort10,
        "compliance_10": compliance10,
        "total_40": total40,
    }


def write_two_output_files(output_dir: Path, rows: List[Dict[str, str]]) -> Tuple[Path, Path]:
    """写入你要求的两份最终结果文件。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    per_scene_path = output_dir / "per_scene_four_metrics.csv"
    avg_only_path = output_dir / "four_metrics_averages.csv"

    valid_rows = [r for r in rows if r.get("status") == "ok"]
    if valid_rows:
        safety_avg = round(sum(float(r["safety_10"]) for r in valid_rows) / len(valid_rows), 3)
        efficiency_avg = round(sum(float(r["efficiency_10"]) for r in valid_rows) / len(valid_rows), 3)
        comfort_avg = round(sum(float(r["comfort_10"]) for r in valid_rows) / len(valid_rows), 3)
        compliance_avg = round(sum(float(r["compliance_10"]) for r in valid_rows) / len(valid_rows), 3)
        total_avg = round(sum(float(r["total_40"]) for r in valid_rows) / len(valid_rows), 3)
    else:
        safety_avg = efficiency_avg = comfort_avg = compliance_avg = total_avg = 0.0

    with per_scene_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "csv",
                "scene",
                "safety_10",
                "efficiency_10",
                "comfort_10",
                "compliance_10",
                "total_40",
                "status",
                "error",
            ],
        )
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

        # 末尾附加平均行
        writer.writerow(
            {
                "csv": "AVERAGE",
                "scene": "AVERAGE",
                "safety_10": safety_avg,
                "efficiency_10": efficiency_avg,
                "comfort_10": comfort_avg,
                "compliance_10": compliance_avg,
                "total_40": total_avg,
                "status": "ok" if valid_rows else "none",
                "error": "",
            }
        )

    with avg_only_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "safety_10_avg",
                "efficiency_10_avg",
                "comfort_10_avg",
                "compliance_10_avg",
                "total_40_avg",
                "valid_scene_count",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "safety_10_avg": safety_avg,
                "efficiency_10_avg": efficiency_avg,
                "comfort_10_avg": comfort_avg,
                "compliance_10_avg": compliance_avg,
                "total_40_avg": total_avg,
                "valid_scene_count": len(valid_rows),
            }
        )

    return per_scene_path, avg_only_path


# =========================
# 运行期目录与赛题识别
# =========================

def infer_topic(args_topic: str, csv_dir: Path, csv_files: List[Path], scene_root: Path) -> str:
    """
    赛题标识规则：
    - 手动指定：A / B / C
    - auto：优先从 csv_dir 名称判断；否则从场景所在分组(A/B/C)判断
    """
    if args_topic in {"A", "B", "C"}:
        return args_topic

    base = csv_dir.name.upper()
    if base in {"A", "B", "C"}:
        return base

    groups: Set[str] = set()
    for f in csv_files:
        try:
            scene_name = extract_scene_name(f.name)
            sf = find_scene_files(scene_root, scene_name)
            g = sf.scene_dir.parent.name.upper()
            if g in {"A", "B", "C"}:
                groups.add(g)
        except Exception:
            continue

    if len(groups) == 1:
        return list(groups)[0]
    if groups == {"B", "C"}:
        # 在混合目录中给出可读标记；建议实际使用时按B/C分开运行
        return "B_C"
    if not groups:
        return "UNKNOWN"
    return "MIXED"


def make_run_output_dir(output_root: Path, topic: str) -> Path:
    """创建本次运行输出目录：outputs/YYYYmmdd_HHMMSS_A(or B or C)。"""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = output_root / f"{ts}_{topic}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


# =========================
# 单场景评分
# =========================

def score_one_csv(
    csv_path: Path,
    scene_root: Path,
    offline_evaluate,
    run_output_dir: Path,
    keep_workdir: bool,
) -> Tuple[Dict[str, float], Optional[str]]:
    """对单个 CSV 执行评分，返回四项分与错误信息。"""
    scene_name = extract_scene_name(csv_path.name)
    scene_type = extract_scene_type(csv_path.name)
    scene_files = find_scene_files(scene_root, scene_name)

    # 每个场景独立临时目录，避免相互污染
    workdir = Path(mkdtemp(prefix="onsite_score_", dir=str(run_output_dir)))
    _chmod_readable_tree(workdir)
    map_stage_root = workdir / "map_stage"
    trj_stage_root = workdir / "trj_stage"

    stage_map_folder(scene_files, scene_type, map_stage_root)
    trj_dir = stage_single_csv(csv_path, trj_stage_root)

    print(f"[INFO] csv={csv_path.name} scene={scene_name} type={scene_type}")
    print(f"[INFO] using {scene_files.xodr.name} + {scene_files.gt_xosc.name}")

    try:
        raw_score = offline_evaluate.offline_evaluation(str(trj_dir), str(map_stage_root))
        metrics = normalize_four_metrics(raw_score)
        err = None
    except Exception as e:
        metrics = {
            "safety_10": 0.0,
            "efficiency_10": 0.0,
            "comfort_10": 0.0,
            "compliance_10": 0.0,
            "total_40": 0.0,
        }
        err = str(e)

    # 默认删除临时目录（你明确要求不保留 map_stage / trj_stage）
    if not keep_workdir:
        shutil.rmtree(workdir, ignore_errors=True)
    else:
        _chmod_readable_tree(workdir)

    return metrics, err


def _score_one_csv_worker(
    csv_path_str: str,
    scene_root_str: str,
    eval_root_str: str,
    run_output_dir_str: str,
    keep_workdir: bool,
) -> Dict[str, str]:
    """
    进程池 worker 入口（必须是顶层函数，便于多进程序列化）。
    每个任务独立导入评分器，避免跨进程对象传递问题。
    """
    csv_path = Path(csv_path_str)
    scene_root = Path(scene_root_str)
    eval_root = Path(eval_root_str)
    run_output_dir = Path(run_output_dir_str)
    scene_name = extract_scene_name(csv_path.name)

    try:
        ensure_kb_files(eval_root)
        offline_evaluate = import_offline_eval(eval_root)
        metrics, err = score_one_csv(
            csv_path=csv_path,
            scene_root=scene_root,
            offline_evaluate=offline_evaluate,
            run_output_dir=run_output_dir,
            keep_workdir=keep_workdir,
        )
        status = "ok" if not err else "failed"
        return {
            "csv": csv_path.name,
            "scene": scene_name,
            "safety_10": f"{metrics['safety_10']:.3f}",
            "efficiency_10": f"{metrics['efficiency_10']:.3f}",
            "comfort_10": f"{metrics['comfort_10']:.3f}",
            "compliance_10": f"{metrics['compliance_10']:.3f}",
            "total_40": f"{metrics['total_40']:.3f}",
            "status": status,
            "error": err or "",
        }
    except Exception as e:
        return {
            "csv": csv_path.name,
            "scene": scene_name,
            "safety_10": "0.000",
            "efficiency_10": "0.000",
            "comfort_10": "0.000",
            "compliance_10": "0.000",
            "total_40": "0.000",
            "status": "failed",
            "error": str(e),
        }


# =========================
# 主流程
# =========================

def main() -> int:
    args = parse_args()

    csv_dir = Path(args.csv_dir).resolve()
    scene_root = Path(args.scene_root).resolve()
    eval_root = Path(args.eval_root).resolve()
    output_root = Path(args.output_dir).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    if not csv_dir.is_dir() and not args.csv:
        raise FileNotFoundError(f"csv-dir 不存在: {csv_dir}")
    if not scene_root.is_dir():
        raise FileNotFoundError(f"scene-root 不存在: {scene_root}")
    if not eval_root.is_dir():
        raise FileNotFoundError(f"eval-root 不存在: {eval_root}")

    csv_files = pick_csv_files(csv_dir, args.csv)
    topic = infer_topic(args.topic, csv_dir, csv_files, scene_root)
    print(f"[INFO] Found {len(csv_files)} CSV file(s)")
    print(f"[INFO] topic={topic}")

    # dry-run：仅检查匹配，不评分、不写输出
    if args.dry_run:
        for f in csv_files:
            scene_name = extract_scene_name(f.name)
            sf = find_scene_files(scene_root, scene_name)
            print(f"[DRY-RUN] {f.name} -> {sf.scene_dir} ({sf.gt_xosc.name}, {sf.xodr.name})")
        return 0

    run_output_dir = make_run_output_dir(output_root, topic)
    print(f"[INFO] run_output_dir={run_output_dir}")
    print(f"[INFO] workers={args.workers}")

    rows: List[Dict[str, str]] = []
    # 串行模式：逻辑更直观，便于调试
    if args.workers <= 1:
        ensure_kb_files(eval_root)
        offline_evaluate = import_offline_eval(eval_root)
        for csv_file in csv_files:
            scene_name = extract_scene_name(csv_file.name)
            try:
                metrics, err = score_one_csv(
                    csv_path=csv_file,
                    scene_root=scene_root,
                    offline_evaluate=offline_evaluate,
                    run_output_dir=run_output_dir,
                    keep_workdir=args.keep_workdir,
                )
                status = "ok" if not err else "failed"
                rows.append(
                    {
                        "csv": csv_file.name,
                        "scene": scene_name,
                        "safety_10": f"{metrics['safety_10']:.3f}",
                        "efficiency_10": f"{metrics['efficiency_10']:.3f}",
                        "comfort_10": f"{metrics['comfort_10']:.3f}",
                        "compliance_10": f"{metrics['compliance_10']:.3f}",
                        "total_40": f"{metrics['total_40']:.3f}",
                        "status": status,
                        "error": err or "",
                    }
                )
            except Exception as e:
                rows.append(
                    {
                        "csv": csv_file.name,
                        "scene": scene_name,
                        "safety_10": "0.000",
                        "efficiency_10": "0.000",
                        "comfort_10": "0.000",
                        "compliance_10": "0.000",
                        "total_40": "0.000",
                        "status": "failed",
                        "error": str(e),
                    }
                )
    else:
        # 外层并行：每个 CSV 一个子进程
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            future_map = {
                pool.submit(
                    _score_one_csv_worker,
                    str(csv_file),
                    str(scene_root),
                    str(eval_root),
                    str(run_output_dir),
                    args.keep_workdir,
                ): idx
                for idx, csv_file in enumerate(csv_files)
            }
            ordered_rows: Dict[int, Dict[str, str]] = {}
            for fut in as_completed(future_map):
                idx = future_map[fut]
                try:
                    ordered_rows[idx] = fut.result()
                except Exception as e:
                    csv_file = csv_files[idx]
                    ordered_rows[idx] = {
                        "csv": csv_file.name,
                        "scene": extract_scene_name(csv_file.name),
                        "safety_10": "0.000",
                        "efficiency_10": "0.000",
                        "comfort_10": "0.000",
                        "compliance_10": "0.000",
                        "total_40": "0.000",
                        "status": "failed",
                        "error": str(e),
                    }
            rows = [ordered_rows[i] for i in sorted(ordered_rows.keys())]

    per_scene_path, avg_only_path = write_two_output_files(run_output_dir, rows)
    print(f"[OK] Wrote: {per_scene_path}")
    print(f"[OK] Wrote: {avg_only_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
