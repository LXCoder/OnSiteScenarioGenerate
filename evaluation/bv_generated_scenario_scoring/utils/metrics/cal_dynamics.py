import numpy as np

def get_dynamic_constraint(sm, config):
    # 用于存储所有场景的碰撞检测结果
    dynamic_constraint_results = {}
    # 用于存储符合动力学约束的场景
    valid_scenes = []
    
    # 统计违反约束的场景数量
    acc_violation_count = 0
    heading_violation_count = 0
    is_invalid_mask_count = 0
    continuity_violation_count = 0
    reverse_violation_count = 0  # 新增：倒车违规统计
    
    # 定义运动学约束阈值
    acc_threshold_min = -9.8  # 最小加速度阈值 (m/s^2)
    acc_threshold_max = 9.8   # 最大加速度阈值 (m/s^2)
    heading_threshold_min = -0.7  # 最小航向角变化阈值 (rad)
    heading_threshold_max = 0.7   # 最大航向角变化阈值 (rad)
    # 浮点容差：避免 -0.7000000477 这类边界数值抖动造成误判
    threshold_eps = float(config.get("dynamic_threshold_eps", 1e-4))
    # 倒车检测阈值
    reverse_speed_threshold = 0.5  # 最小速度阈值，低于此值不检测倒车 (m/s)
    reverse_threshold_frames = 10  # 连续倒车帧数阈值

    # 可配置参数（保持向后兼容）
    start_idx = int(config.get("dynamic_start_index", config.get("warmup", 31)))
    require_valid_at_start = bool(config.get("dynamic_require_valid_at_start", True))
    min_valid_points = int(config.get("dynamic_min_valid_points", 1))
    check_continuity = bool(config.get("dynamic_check_continuity", True))
    heading_adjacent_only = bool(config.get("dynamic_heading_adjacent_only", False))

    skipped_insufficient_agents = 0
    
    for scene_idx, scene in enumerate(sm.tasks):
        positions = scene.additional_info['positions']  # [Na,Nt,2]
        headings = scene.additional_info['headings']  # [Na,Nt]
        valid_mask = scene.additional_info['valid_mask']  # [Na,Nt]
        predict_mask = scene.additional_info['predict_mask']  # [Na]
        ids = scene.additional_info.get('ids', [])
        dt = float(scene.task_info.get("dt", 0.1))
        if dt <= 0:
            dt = 0.1

        # 计算场景是否超过运动学约束
        is_acc_violation = False
        is_heading_violation = False
        is_invalid_mask = False
        is_continuity_violation = False
        is_reverse_violation = False  # 新增：倒车违规标志
        first_reason_type = ""
        first_reason_agent = ""
        first_reason_min = None
        first_reason_max = None

        # 只遍历需要预测的代理predict_mask=True
        pred_agent_indices = np.where(predict_mask)[0]
        for agent_idx in pred_agent_indices:
            agent_valid_mask = valid_mask[agent_idx].astype(bool)
            nt = len(agent_valid_mask)

            # 旧版默认要求“起始评估帧必须有效”；2026可通过配置关闭。
            if require_valid_at_start:
                if start_idx >= nt or not agent_valid_mask[start_idx]:
                    is_invalid_mask = True
                    if not first_reason_type:
                        first_reason_type = "invalid_start_frame"
                        first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)

            eval_indices = np.where(agent_valid_mask & (np.arange(nt) >= start_idx))[0]
            if len(eval_indices) < min_valid_points:
                skipped_insufficient_agents += 1
                if is_invalid_mask:
                    break
                continue

            if check_continuity:
                first_valid_idx = int(eval_indices[0])
                last_valid_idx = int(eval_indices[-1])
                if np.any(~agent_valid_mask[first_valid_idx:last_valid_idx + 1]):
                    is_continuity_violation = True
                    if not first_reason_type:
                        first_reason_type = "continuity_gap"
                        first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)

            # 使用连续帧差分计算加速度，避免跨缺失帧导致的虚假极值
            if nt >= 3:
                triplet_mask = (
                    agent_valid_mask[2:]
                    & agent_valid_mask[1:-1]
                    & agent_valid_mask[:-2]
                )
                t_triplets = np.where(triplet_mask)[0] + 2
                t_triplets = t_triplets[t_triplets >= max(start_idx, 2)]
                if t_triplets.size > 0:
                    p = positions[agent_idx]
                    speed_now = np.linalg.norm(p[t_triplets] - p[t_triplets - 1], axis=1) / dt
                    speed_prev = np.linalg.norm(p[t_triplets - 1] - p[t_triplets - 2], axis=1) / dt
                    accel_vals = (speed_now - speed_prev) / dt
                    if np.any(accel_vals < (acc_threshold_min - threshold_eps)) or np.any(accel_vals > (acc_threshold_max + threshold_eps)):
                        is_acc_violation = True
                        if not first_reason_type:
                            first_reason_type = "acc_out_of_range"
                            first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)
                            first_reason_min = float(np.min(accel_vals))
                            first_reason_max = float(np.max(accel_vals))

            # 航向角变化约束（2026建议只在连续有效帧上检测）
            if heading_adjacent_only:
                pair_mask = agent_valid_mask[1:] & agent_valid_mask[:-1]
                t_pairs = np.where(pair_mask)[0] + 1
                t_pairs = t_pairs[t_pairs >= max(start_idx, 1)]
                if t_pairs.size > 0:
                    heading_changes = headings[agent_idx, t_pairs] - headings[agent_idx, t_pairs - 1]
                    heading_changes = np.arctan2(np.sin(heading_changes), np.cos(heading_changes))
                    if np.any(heading_changes < (heading_threshold_min - threshold_eps)) or np.any(heading_changes > (heading_threshold_max + threshold_eps)):
                        is_heading_violation = True
                        if not first_reason_type:
                            first_reason_type = "heading_change_out_of_range"
                            first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)
                            first_reason_min = float(np.min(heading_changes))
                            first_reason_max = float(np.max(heading_changes))
            else:
                valid_headings = headings[agent_idx][agent_valid_mask]
                heading_changes = np.diff(valid_headings)
                heading_changes = np.arctan2(np.sin(heading_changes), np.cos(heading_changes))
                start_heading_idx = max(0, start_idx - 1)
                heading_changes = heading_changes[start_heading_idx:]
                if heading_changes.size > 0:
                    if np.any(heading_changes < (heading_threshold_min - threshold_eps)) or np.any(heading_changes > (heading_threshold_max + threshold_eps)):
                        is_heading_violation = True
                        if not first_reason_type:
                            first_reason_type = "heading_change_out_of_range"
                            first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)
                            first_reason_min = float(np.min(heading_changes))
                            first_reason_max = float(np.max(heading_changes))

            # 倒车检测：仅连续有效帧上计算前向速度
            consecutive_reverse_count = 0
            for t in range(max(start_idx, 1), nt):
                if not (agent_valid_mask[t] and agent_valid_mask[t - 1]):
                    consecutive_reverse_count = 0
                    continue

                vx, vy = (positions[agent_idx, t] - positions[agent_idx, t - 1]) / dt
                speed = np.sqrt(vx ** 2 + vy ** 2)
                if speed < reverse_speed_threshold:
                    consecutive_reverse_count = 0
                    continue

                heading_angle = headings[agent_idx, t]
                forward_speed = vx * np.cos(heading_angle) + vy * np.sin(heading_angle)
                if forward_speed < 0:
                    consecutive_reverse_count += 1
                    if consecutive_reverse_count >= reverse_threshold_frames:
                        is_reverse_violation = True
                        if not first_reason_type:
                            first_reason_type = "continuous_reverse"
                            first_reason_agent = ids[agent_idx] if agent_idx < len(ids) else str(agent_idx)
                        break
                else:
                    consecutive_reverse_count = 0
            
            # 如果五个约束中的任何一个被违反，就不需要继续检查其他智能体
            if is_acc_violation or is_heading_violation or is_invalid_mask or is_continuity_violation or is_reverse_violation:
                break

        # 更新违反约束的统计
        if is_acc_violation:
            acc_violation_count += 1
        if is_heading_violation:
            heading_violation_count += 1
        if is_invalid_mask:
            is_invalid_mask_count += 1
        if is_continuity_violation:
            continuity_violation_count += 1
        if is_reverse_violation:
            reverse_violation_count += 1

        scene_name = scene.name
        dynamic_constraint_results[scene_name] = {
            'is_out_dynamic': 1 if (is_acc_violation or is_heading_violation or is_invalid_mask or is_continuity_violation or is_reverse_violation) else 0,
            'dynamic_reason_type': first_reason_type,
            'dynamic_reason_agent': first_reason_agent,
            'dynamic_reason_min': first_reason_min,
            'dynamic_reason_max': first_reason_max,
            'dynamic_acc_violation': 1 if is_acc_violation else 0,
            'dynamic_heading_violation': 1 if is_heading_violation else 0,
            'dynamic_invalid_mask_violation': 1 if is_invalid_mask else 0,
            'dynamic_continuity_violation': 1 if is_continuity_violation else 0,
            'dynamic_reverse_violation': 1 if is_reverse_violation else 0,
        }
        
        # 如果场景符合动力学约束，则添加到有效场景列表
        if not (is_acc_violation or is_heading_violation or is_invalid_mask or is_continuity_violation or is_reverse_violation):
            valid_scenes.append(scene)
    
    # 更新sm.tasks，只保留符合动力学约束的场景
    sm.tasks = valid_scenes
    print('---------acc_violation_count---------', acc_violation_count)
    print('---------heading_violation_count---------', heading_violation_count)
    print('---------is_invalid_mask_count---------', is_invalid_mask_count)
    print('---------continuity_violation_count---------', continuity_violation_count)
    print('---------reverse_violation_count---------', reverse_violation_count)
    print('---------dynamic_start_index---------', start_idx)
    print('---------dynamic_require_valid_at_start---------', require_valid_at_start)
    print('---------dynamic_min_valid_points---------', min_valid_points)
    print('---------dynamic_heading_adjacent_only---------', heading_adjacent_only)
    print('---------skipped_insufficient_agents---------', skipped_insufficient_agents)
    # print('---------other_violation_count---------', heading_violation_count + acc_violation_count + is_invalid_mask_count + continuity_violation_count)
    return dynamic_constraint_results
