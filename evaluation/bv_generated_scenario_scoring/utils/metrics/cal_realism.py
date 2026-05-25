import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def calculate_js_divergence(p, q):
    """计算两个分布之间的JS散度，处理零值情况"""
    # 添加小量以避免除零和对零取对数
    epsilon = 1e-10
    
    # 确保p和q是概率分布（和为1）
    p = p + epsilon
    q = q + epsilon
    p = p / np.sum(p)
    q = q / np.sum(q)
    
    # 计算平均分布
    m = 0.5 * (p + q)
    
    # 计算KL散度
    kl_p_m = np.sum(p * np.log2(p / m))
    kl_q_m = np.sum(q * np.log2(q / m))
    
    # 计算JS散度
    js = 0.5 * (kl_p_m + kl_q_m)
    
    # 处理数值不稳定的情况
    if np.isnan(js) or np.isinf(js):
        return 1.0  # 返回最大散度值
        
    return js


def get_optimal_bins(data):
    """使用固定的bin数量"""
    # 不再使用Sturges规则，而是直接返回固定值
    return 20


def get_distribution(data, bins=None, range=None, density=False):
    """计算数据的分布，确保返回有效的概率分布"""
    if bins is None:
        bins = get_optimal_bins(data)
    
    hist, bin_edges = np.histogram(data, bins=bins, range=range, density=density)
    # 确保直方图非空且有效
    if np.sum(hist) == 0:
        if density:
            return np.ones(bins) / bins, bin_edges  # 如果没有数据，返回均匀分布
        else:
            return np.zeros(bins), bin_edges  # 如果没有数据，返回零频率
    return hist, bin_edges


def get_realism_metrics(sm, config, gt_dir, logger=None):
    """计算所有场景的真实性指标并保存到CSV文件"""
    # 用于存储所有场景的真实性结果
    realism_results = {}
    
    # 用于收集所有场景的数据，以便绘制分布图
    all_gen_velocities = []
    all_gt_velocities = []
    all_gen_lon_acc = []
    all_gt_lon_acc = []
    all_gen_yaw_rate = []
    all_gt_yaw_rate = []
    
    # 计算所有场景的真实性指标
    for scene_idx, scene in enumerate(sm.tasks):
        # 获取生成轨迹数据
        gen_data = scene.additional_info

        # 获取GT轨迹数据
        # 1) 优先使用内存中提供的 gt_info（raw xosc/xodr 直读模式）
        # 2) 否则回退读取磁盘 _gt.pkl（原有 pkl 工作流）
        gt_info = gen_data.get('gt_info', None)
        if gt_info is None:
            scene_name = scene.name
            gt_scene_dir = os.path.join(gt_dir, scene_name)
            gt_path = os.path.join(gt_scene_dir, f"{scene_name}_gt.pkl")
            with open(gt_path, 'rb') as handle:
                gt_info = pickle.load(handle)
        
        # 计算真实性指标
        velocity_js, lon_acc_js, yaw_rate_js, scene_data = calculate_realism_metrics(
            gen_data, 
            gt_info, 
            config['warmup'],
            sm
        )

        # 收集场景数据用于绘图
        if scene_data:
            all_gen_velocities.extend(scene_data['gen_velocities'])
            all_gt_velocities.extend(scene_data['gt_velocities'])
            all_gen_lon_acc.extend(scene_data['gen_lon_acc'])
            all_gt_lon_acc.extend(scene_data['gt_lon_acc'])
            all_gen_yaw_rate.extend(scene_data['gen_yaw_rate'])
            all_gt_yaw_rate.extend(scene_data['gt_yaw_rate'])
        
        # 计算总体真实性得分（三个指标的平均值）
        realism_score = (velocity_js + lon_acc_js + yaw_rate_js) / 3

        # 存储当前场景的真实性结果
        scene_key = scene.name
        realism_results[scene_key] = {
            'realism_score': realism_score,
            'realism_details': {
                'velocity_js': velocity_js,
                'lon_acc_js': lon_acc_js,
                'yaw_rate_js': yaw_rate_js
            }
        }
    
    # # 绘制真实性指标分布图
    # plot_realism_distributions(
    #     all_gen_velocities, all_gt_velocities,
    #     all_gen_lon_acc, all_gt_lon_acc,
    #     all_gen_yaw_rate, all_gt_yaw_rate
    # )
    
    return realism_results


def plot_realism_distributions(gen_velocities, gt_velocities, gen_lon_acc, gt_lon_acc, gen_yaw_rate, gt_yaw_rate):
    """绘制生成轨迹与GT轨迹的真实性指标分布图"""
    # 创建输出目录
    plots_dir = os.path.join("outputs", "plots")
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)
    
    # 创建一个2x2的图表布局
    fig = plt.figure(figsize=(15, 12))
    gs = GridSpec(2, 2, figure=fig)
    
    # 使用固定的bin数量
    fixed_bins = 20
    
    # 1. 速度分布图
    ax1 = fig.add_subplot(gs[0, 0])
    
    # 使用频率而不是密度
    gen_velocity_hist, gen_velocity_bins = get_distribution(gen_velocities, bins=fixed_bins, density=False)
    gt_velocity_hist, _ = get_distribution(gt_velocities, bins=fixed_bins, density=False)
    
    # 归一化频率为百分比
    gen_velocity_hist = gen_velocity_hist / len(gen_velocities) * 100 if len(gen_velocities) > 0 else gen_velocity_hist
    gt_velocity_hist = gt_velocity_hist / len(gt_velocities) * 100 if len(gt_velocities) > 0 else gt_velocity_hist
    
    bin_centers = (gen_velocity_bins[:-1] + gen_velocity_bins[1:]) / 2
    ax1.bar(bin_centers, gen_velocity_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Generated')
    ax1.bar(bin_centers, gt_velocity_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Ground Truth')
    ax1.set_xlabel('Velocity (m/s)')
    ax1.set_ylabel('Frequency (%)')
    ax1.set_title('Velocity Distribution')
    ax1.set_ylim(0, 100)  # 锁定纵坐标最大值为100
    ax1.legend()
    ax1.grid(True, linestyle='--', alpha=0.7)
    
    # 2. 纵向加速度分布图
    ax2 = fig.add_subplot(gs[0, 1])
    
    # 使用频率而不是密度
    gen_acc_hist, gen_acc_bins = get_distribution(gen_lon_acc, bins=fixed_bins, density=False)
    gt_acc_hist, _ = get_distribution(gt_lon_acc, bins=fixed_bins, density=False)
    
    # 归一化频率为百分比
    gen_acc_hist = gen_acc_hist / len(gen_lon_acc) * 100 if len(gen_lon_acc) > 0 else gen_acc_hist
    gt_acc_hist = gt_acc_hist / len(gt_lon_acc) * 100 if len(gt_lon_acc) > 0 else gt_acc_hist
    
    bin_centers = (gen_acc_bins[:-1] + gen_acc_bins[1:]) / 2
    ax2.bar(bin_centers, gen_acc_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Generated')
    ax2.bar(bin_centers, gt_acc_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Ground Truth')
    ax2.set_xlabel('Longitudinal Acceleration (m/s²)')
    ax2.set_ylabel('Frequency (%)')
    ax2.set_title('Longitudinal Acceleration Distribution')
    ax2.set_ylim(0, 100)  # 锁定纵坐标最大值为100
    ax2.legend()
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    # 3. 角速度分布图
    ax3 = fig.add_subplot(gs[1, 0])
    
    # 使用频率而不是密度
    gen_yaw_hist, gen_yaw_bins = get_distribution(gen_yaw_rate, bins=fixed_bins, density=False)
    gt_yaw_hist, _ = get_distribution(gt_yaw_rate, bins=fixed_bins, density=False)
    
    # 归一化频率为百分比
    gen_yaw_hist = gen_yaw_hist / len(gen_yaw_rate) * 100 if len(gen_yaw_rate) > 0 else gen_yaw_hist
    gt_yaw_hist = gt_yaw_hist / len(gt_yaw_rate) * 100 if len(gt_yaw_rate) > 0 else gt_yaw_hist
    
    bin_centers = (gen_yaw_bins[:-1] + gen_yaw_bins[1:]) / 2
    ax3.bar(bin_centers, gen_yaw_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Generated')
    ax3.bar(bin_centers, gt_yaw_hist, width=bin_centers[1]-bin_centers[0], alpha=0.6, label='Ground Truth')
    ax3.set_xlabel('Yaw Rate (rad/s)')
    ax3.set_ylabel('Frequency (%)')
    ax3.set_title('Yaw Rate Distribution')
    ax3.set_ylim(0, 100)  # 锁定纵坐标最大值为100
    ax3.legend()
    ax3.grid(True, linestyle='--', alpha=0.7)
    
    # 4. JS散度比较 - 这部分仍然需要使用密度计算
    ax4 = fig.add_subplot(gs[1, 1])
    
    # 为JS散度计算获取密度分布
    gen_velocity_dist, _ = get_distribution(gen_velocities, bins=fixed_bins, density=True)
    gt_velocity_dist, _ = get_distribution(gt_velocities, bins=fixed_bins, density=True)
    
    gen_acc_dist, _ = get_distribution(gen_lon_acc, bins=fixed_bins, density=True)
    gt_acc_dist, _ = get_distribution(gt_lon_acc, bins=fixed_bins, density=True)
    
    gen_yaw_dist, _ = get_distribution(gen_yaw_rate, bins=fixed_bins, density=True)
    gt_yaw_dist, _ = get_distribution(gt_yaw_rate, bins=fixed_bins, density=True)
    
    velocity_js = calculate_js_divergence(gen_velocity_dist, gt_velocity_dist)
    lon_acc_js = calculate_js_divergence(gen_acc_dist, gt_acc_dist)
    yaw_rate_js = calculate_js_divergence(gen_yaw_dist, gt_yaw_dist)
    avg_js = (velocity_js + lon_acc_js + yaw_rate_js) / 3
    
    metrics = ['Velocity', 'Lon. Acc.', 'Yaw Rate', 'Average']
    js_values = [velocity_js, lon_acc_js, yaw_rate_js, avg_js]

    ax4.bar(metrics, js_values, color='skyblue')
    ax4.set_ylabel('JS Divergence')
    ax4.set_title('JS Divergence Between Generated and GT Distributions')
    ax4.set_ylim(0, 1)  # 锁定纵坐标最大值为0.5
    ax4.grid(True, linestyle='--', alpha=0.7)
    
    # 在柱状图上添加具体数值
    for i, v in enumerate(js_values):
        ax4.text(i, v + 0.02, f'{v:.4f}', ha='center')
    
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, 'realism_distributions.png'))
    plt.close()
    
    print(f"Realism distribution plots saved to {plots_dir}/realism_distributions.png")


def calculate_realism_metrics(gen_data, gt_info, warmup, sm):
    """计算生成轨迹与GT轨迹之间的真实性指标"""
    predict_mask = gen_data['predict_mask']
    gen_valid_mask = gen_data['valid_mask']
    gen_velocities = gen_data['velocities']
    gen_headings = gen_data['headings']
    gen_accelerations = gen_data['accelerations']
    
    # 获取GT数据
    gt_valid_mask = gt_info['valid_mask']
    # 使用sm的函数计算GT轨迹的速度和加速度
    gt_velocities, gt_accelerations, gt_vector_velocities = sm._calculate_velocity_acceleration(gt_valid_mask, gt_info['positions'])
    gt_headings = gt_info['headings']
    
    # 初始化JS散度总和
    total_velocity_js = 0.0
    total_lon_acc_js = 0.0
    total_yaw_rate_js = 0.0
    vehicle_count = 0
    
    # 收集所有车辆的数据用于绘图
    all_gen_velocities = []
    all_gt_velocities = []
    all_gen_lon_acc = []
    all_gt_lon_acc = []
    all_gen_yaw_rate = []
    all_gt_yaw_rate = []
    
    # 使用固定的bin数量
    fixed_bins = 20
    
    for i in range(len(predict_mask)):
        if predict_mask[i]:  # 只计算需要预测的车辆
            # 提取该车辆在warmup之后的有效轨迹
            gen_valid_indices = np.where(gen_valid_mask[i, warmup:])[0] + warmup
            gt_valid_indices = np.where(gt_valid_mask[i, warmup:])[0] + warmup
            
            if len(gen_valid_indices) < 3 or len(gt_valid_indices) < 3:
                continue  # 轨迹太短，无法计算指标
            
            # 提取速度数据
            gen_vehicle_velocities = gen_velocities[i, gen_valid_indices]
            gt_vehicle_velocities = gt_velocities[i, gt_valid_indices]
            
            # 收集速度数据
            all_gen_velocities.extend(gen_vehicle_velocities)
            all_gt_velocities.extend(gt_vehicle_velocities)
            
            # 使用计算好的加速度
            gen_lon_acc = gen_accelerations[i, gen_valid_indices[1:]]
            gt_lon_acc = gt_accelerations[i, gt_valid_indices[1:]]  # 从第二个时间步开始，与生成轨迹保持一致
            
            # 收集加速度数据
            all_gen_lon_acc.extend(gen_lon_acc)
            all_gt_lon_acc.extend(gt_lon_acc)
            
            # 计算角速度
            gen_yaw_rate = []
            gt_yaw_rate = []
            
            # 生成轨迹的角速度
            for j in range(1, len(gen_valid_indices)):
                heading_diff = gen_headings[i, gen_valid_indices[j]] - gen_headings[i, gen_valid_indices[j-1]]
                # 处理角度跨越±π的情况
                if heading_diff > np.pi:
                    heading_diff -= 2 * np.pi
                elif heading_diff < -np.pi:
                    heading_diff += 2 * np.pi
                dt = gen_valid_indices[j] - gen_valid_indices[j-1]
                gen_yaw_rate.append(heading_diff / dt)
            
            # GT轨迹的角速度
            for j in range(1, len(gt_valid_indices)):
                heading_diff = gt_headings[i, gt_valid_indices[j]] - gt_headings[i, gt_valid_indices[j-1]]
                # 处理角度跨越±π的情况
                if heading_diff > np.pi:
                    heading_diff -= 2 * np.pi
                elif heading_diff < -np.pi:
                    heading_diff += 2 * np.pi
                dt = gt_valid_indices[j] - gt_valid_indices[j-1]
                gt_yaw_rate.append(heading_diff / dt)
            
            # 收集角速度数据
            all_gen_yaw_rate.extend(gen_yaw_rate)
            all_gt_yaw_rate.extend(gt_yaw_rate)
            
            # 计算分布时使用固定的bins
            gen_velocity_dist, _ = get_distribution(gen_vehicle_velocities, bins=fixed_bins, density=True)
            gt_velocity_dist, _ = get_distribution(gt_vehicle_velocities, bins=fixed_bins, density=True)
            
            gen_lon_acc_dist, _ = get_distribution(gen_lon_acc, bins=fixed_bins, density=True)
            gt_lon_acc_dist, _ = get_distribution(gt_lon_acc, bins=fixed_bins, density=True)
            
            gen_yaw_rate_dist, _ = get_distribution(gen_yaw_rate, bins=fixed_bins, density=True)
            gt_yaw_rate_dist, _ = get_distribution(gt_yaw_rate, bins=fixed_bins, density=True)
            
            # 计算JS散度
            velocity_js = calculate_js_divergence(gen_velocity_dist, gt_velocity_dist)
            lon_acc_js = calculate_js_divergence(gen_lon_acc_dist, gt_lon_acc_dist)
            yaw_rate_js = calculate_js_divergence(gen_yaw_rate_dist, gt_yaw_rate_dist)
            
            # 累加JS散度
            total_velocity_js += velocity_js
            total_lon_acc_js += lon_acc_js
            total_yaw_rate_js += yaw_rate_js
            vehicle_count += 1
    
    # 计算平均JS散度
    if vehicle_count > 0:
        avg_velocity_js = total_velocity_js / vehicle_count
        avg_lon_acc_js = total_lon_acc_js / vehicle_count
        avg_yaw_rate_js = total_yaw_rate_js / vehicle_count
        
        # 返回收集的数据用于绘图
        collected_data = {
            'gen_velocities': all_gen_velocities,
            'gt_velocities': all_gt_velocities,
            'gen_lon_acc': all_gen_lon_acc,
            'gt_lon_acc': all_gt_lon_acc,
            'gen_yaw_rate': all_gen_yaw_rate,
            'gt_yaw_rate': all_gt_yaw_rate
        }
        
        return avg_velocity_js, avg_lon_acc_js, avg_yaw_rate_js, collected_data
    else:
        return 0.0, 0.0, 0.0, None