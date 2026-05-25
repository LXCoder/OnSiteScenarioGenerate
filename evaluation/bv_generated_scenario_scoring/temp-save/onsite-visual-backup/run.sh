#! /bin/bash

#SBATCH --job-name=onsite-vis
#SBATCH --partition=A800
#SBATCH --nodes=1
#SBATCH --gres=gpu:a800:1           # 申请4块A800 GPU
#SBATCH --cpus-per-task=7          # 总CPU核心数=4 GPU ×7=28
#SBATCH --ntasks=1                  # 明确任务数为1（避免并行任务导致核心翻倍）
#SBATCH --mail-type=END,FAIL
#SBATCH --mail-user=2021903132@chd.edu.cn
#SBATCH --output=log/train_a800_4_%j.out
#SBATCH --error=log/train_a800_4_%j.err

# 确保使用 bash 解释器
source /share/home/u24005/miniconda3/etc/profile.d/conda.sh  # 加载conda初始化脚本
conda activate onsite                                           # 激活指定conda环境    

# dos2unix run.sh
# 进入工作目录
cd ~/codes/onsite-visual/

# 执行训练脚本
python visualization.py
