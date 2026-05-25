# onsite_evaluation 一体化离线打分（BV+AV）

本项目只保留一种流程：**离线一体化打分**。  
执行顺序固定为：**BV 先打分 -> AV 再打分 -> 按场景合并成 100 分制**。

## 1. 固定数据口径

- BV 输入只来自：`bv_generated_scenario_scoring/scenario` 下的 `*.xosc`
- AV 输入只来自：你传入的轨迹 CSV 目录（如 `av_ego_scoring/traj`）
- GT/xodr 只来自：`groundtruth31_exam_gt_release_final_A200T50_20260520`（或你显式指定的 GT 根目录）
- 合并按并集场景：
- 只有 BV 的场景：AV 记 0 分
- 只有 AV 的场景：BV 记 0 分

## 2. 一条命令运行
/home/zd/Program/onsite_evaluation/av_ego_scoring/traj 这里对应推理仿真保存的csv
```bash
cd /home/zd/Program/onsite_evaluation
/home/zd/anaconda3/envs/onsite/bin/python main.py \
  /home/zd/Program/onsite_evaluation/av_ego_scoring/traj \
  --bv-scene-root /home/zd/Program/onsite_evaluation/bv_generated_scenario_scoring/scenario \
  --gt-scene-root /home/zd/Program/onsite_evaluation/groundtruth31_exam_gt_release_final_A200T50_20260520 \
  --output-root /home/zd/Program/onsite_evaluation/outputs \
  --num-workers-bv 4 \
  --num-workers-av 1
```

## 3. 输出结果

每次运行输出到：`outputs/<timestamp>_<split>/`

- `per_scene_detailed_100.csv`：场景级明细（BV60 + AV40）
- `summary_averages_100.csv`：整体均值汇总

中间临时文件会自动清理，只保留最终两张 CSV。

## 4. 脚本分工

- `main.py`：统一入口（推荐）
- `integrated_bv_av_scoring.py`：执行 BV/AV 和最终合并
- `bv_generated_scenario_scoring/replay_run_2026.py`：BV 打分
- `av_ego_scoring/score_ego_effectiveness.py`：AV 打分

## 5. 常见问题

- `No scene id parsed from trajectory csv names`
  - 轨迹文件名里没有 `scenario_xxxxxxxx`，先规范命名。
- `Missing ... scene dirs/files`
  - BV/AV 场景 ID 在 GT 根目录中找不到对应场景目录或缺少 `xodr/gt.xosc`。
