import os
import numpy as np


def get_comfort_metrics(sm, config):
    """计算所有场景的舒适度指标并保存到CSV文件"""
    # 用于存储所有场景的舒适度结果
    comfort_results = {}
    
    # 计算所有场景的舒适度指标
    for scene_idx, scene in enumerate(sm.tasks):
        # 计算场景中所有需要预测的车辆的舒适度
        comfort_score, comfort_details = calculate_all_vehicles_comfort(
            scene.additional_info, 
            config['warmup'], 
            scene.task_info['dt']
        )
        
        # 存储当前场景的舒适度结果
        # scene_key = f"{scene.type}_{scene.num}_{scene.name}"
        scene_key = scene.name
        comfort_results[scene_key] = {
            # 'scene_num': scene.num,
            'comfort_score': comfort_score,
            'comfort_details': comfort_details
        }
    
    return comfort_results
    
    
def calculate_all_vehicles_comfort(additional_info, warmup, dt):
    """计算场景中所有需要预测的车辆的舒适度"""
    predict_mask = additional_info['predict_mask']
    valid_mask = additional_info['valid_mask']
    positions = additional_info['positions']
    headings = additional_info['headings']
    velocities = additional_info['velocities']
    
    # 初始化各项指标的总和
    total_lon_acc_ratio = 0.0
    total_lon_jerk_ratio = 0.0
    total_lat_acc_ratio = 0.0
    total_lat_jerk_ratio = 0.0
    total_yaw_rate_ratio = 0.0
    vehicle_count = 0
    
    for i in range(len(predict_mask)):
        if predict_mask[i]:  # 只计算需要预测的车辆
            # 提取该车辆在warmup之后的有效轨迹
            valid_indices = np.where(valid_mask[i, warmup:])[0] + warmup
            
            if len(valid_indices) < 3:
                continue  # 轨迹太短，无法计算舒适度
            
            vehicle_positions = positions[i, valid_indices]
            vehicle_headings = headings[i, valid_indices]
            vehicle_velocities = velocities[i, valid_indices]
            
            # 计算纵向和横向加速度
            lon_acc = []
            lat_acc = []
            yaw_rate = []
            
            for j in range(1, len(valid_indices)):
                # 计算航向变化率
                heading_diff = vehicle_headings[j] - vehicle_headings[j-1]
                # 处理角度跨越±π的情况
                if heading_diff > np.pi:
                    heading_diff -= 2 * np.pi
                elif heading_diff < -np.pi:
                    heading_diff += 2 * np.pi
                current_yaw_rate = heading_diff / dt
                yaw_rate.append(current_yaw_rate)
                
                # 计算速度向量
                v_x = vehicle_velocities[j] * np.cos(vehicle_headings[j])
                v_y = vehicle_velocities[j] * np.sin(vehicle_headings[j])
                
                if j > 0:
                    prev_v_x = vehicle_velocities[j-1] * np.cos(vehicle_headings[j-1])
                    prev_v_y = vehicle_velocities[j-1] * np.sin(vehicle_headings[j-1])
                    a_x = (v_x - prev_v_x) / dt
                    a_y = (v_y - prev_v_y) / dt
                    
                    # 转换为车身坐标系下的纵向和横向加速度
                    lon_acc.append(a_x * np.cos(vehicle_headings[j]) + a_y * np.sin(vehicle_headings[j]))
                    lat_acc.append(-a_x * np.sin(vehicle_headings[j]) + a_y * np.cos(vehicle_headings[j]))
            
            # 计算加加速度
            lon_jerk = []
            lat_jerk = []
            
            for j in range(1, len(lon_acc)):
                lon_jerk.append((lon_acc[j] - lon_acc[j-1]) / dt)
                lat_jerk.append((lat_acc[j] - lat_acc[j-1]) / dt)
            
            # 设定舒适度阈值
            lon_acc_threshold = 3.0  # m/s²
            lon_jerk_threshold = 6.0  # m/s³
            lat_acc_threshold = 0.5  # m/s²
            lat_jerk_threshold = 1.0  # m/s³
            yaw_rate_threshold = 0.5  # rad/s
            
            # 计算超过阈值的比例
            lon_acc_ratio = np.mean(np.abs(np.array(lon_acc)) > lon_acc_threshold)
            lon_jerk_ratio = np.mean(np.abs(np.array(lon_jerk)) > lon_jerk_threshold)
            lat_acc_ratio = np.mean(np.abs(np.array(lat_acc)) > lat_acc_threshold)
            lat_jerk_ratio = np.mean(np.abs(np.array(lat_jerk)) > lat_jerk_threshold)
            yaw_rate_ratio = np.mean(np.abs(np.array(yaw_rate)) > yaw_rate_threshold)
            
            # 累加各项指标
            total_lon_acc_ratio += lon_acc_ratio
            total_lon_jerk_ratio += lon_jerk_ratio
            total_lat_acc_ratio += lat_acc_ratio
            total_lat_jerk_ratio += lat_jerk_ratio
            total_yaw_rate_ratio += yaw_rate_ratio
            vehicle_count += 1
    
    # 计算所有车辆的平均舒适度得分
    if vehicle_count > 0:
        avg_lon_acc_ratio = total_lon_acc_ratio / vehicle_count
        avg_lon_jerk_ratio = total_lon_jerk_ratio / vehicle_count
        avg_lat_acc_ratio = total_lat_acc_ratio / vehicle_count
        avg_lat_jerk_ratio = total_lat_jerk_ratio / vehicle_count
        avg_yaw_rate_ratio = total_yaw_rate_ratio / vehicle_count
        
        comfort_score = avg_lon_acc_ratio + avg_lon_jerk_ratio + avg_lat_acc_ratio + avg_lat_jerk_ratio + avg_yaw_rate_ratio
        
        comfort_details = {
            'lon_acc_ratio': avg_lon_acc_ratio,
            'lon_jerk_ratio': avg_lon_jerk_ratio,
            'lat_acc_ratio': avg_lat_acc_ratio,
            'lat_jerk_ratio': avg_lat_jerk_ratio,
            'yaw_rate_ratio': avg_yaw_rate_ratio
        }
        
        return comfort_score, comfort_details
    else:
        return 0.0, {
            'lon_acc_ratio': 0.0,
            'lon_jerk_ratio': 0.0,
            'lat_acc_ratio': 0.0,
            'lat_jerk_ratio': 0.0,
            'yaw_rate_ratio': 0.0
        }