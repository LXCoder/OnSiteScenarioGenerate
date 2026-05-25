import os
import cv2
import numpy as np

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from replay_test_for_pkl import ScenarioManagerForISG


ROOT_DIR = os.path.abspath(".")  # 开发环境

GT_DIR = os.path.join(ROOT_DIR, "ground_truth")     # 包含gt文件的试题文件夹
SCENE_DIR = os.path.join(ROOT_DIR, "scenario")      # 选手生成的场景文件夹

def quick_collision_detection(positions, headings, valid_mask, shapes, predict_mask=None):
    Na, Nt = valid_mask.shape
    
    # 遍历每个时刻
    for t in range(31, Nt):
        # 获取当前时刻有效的代理
        valid_agents = np.where(valid_mask[:,t])[0]
        # 保留predict_mask为True的代理
        valid_agents = np.array([agent for agent in valid_agents if predict_mask[agent]])
        
        if len(valid_agents) < 2:
            continue

        # 获取所有有效代理的位置和尺寸
        valid_positions = positions[valid_agents, t]  # [N,2]
        valid_shapes = shapes[valid_agents]  # [N,2]
        
        # 计算每个代理的AABB包围盒,增加一定的安全边界
        margin = 1.5  # 100cm的安全边界
        half_lengths = valid_shapes / 2 + margin
        aabb_mins = valid_positions - half_lengths  # [N,2]
        aabb_maxs = valid_positions + half_lengths  # [N,2]
        
        # 矩阵化AABB相交测试
        # [N,1,2] vs [1,N,2] -> [N,N,2]
        overlaps_min = np.maximum(aabb_mins[:,np.newaxis], aabb_mins)
        overlaps_max = np.minimum(aabb_maxs[:,np.newaxis], aabb_maxs)
        
        # 判断是否相交 [N,N]
        is_overlapping = np.all(overlaps_min < overlaps_max, axis=2)
        
        # 移除自身相交
        np.fill_diagonal(is_overlapping, False)
        
        if np.any(is_overlapping):
            # 找到相交的代理对进行精确碰撞检测
            colliding_pairs = np.where(is_overlapping)
            for i, j in zip(*colliding_pairs):
                agent1, agent2 = valid_agents[i], valid_agents[j]
                
                # 直接根据当前时刻的位置和航向判断是否碰撞
                rect1 = ((positions[agent1,t,0], positions[agent1,t,1]),
                        (shapes[agent1,0], shapes[agent1,1]),
                        np.rad2deg(headings[agent1,t]))
                rect2 = ((positions[agent2,t,0], positions[agent2,t,1]),
                        (shapes[agent2,0], shapes[agent2,1]),
                        np.rad2deg(headings[agent2,t]))
                if cv2.rotatedRectangleIntersection(rect1, rect2)[0]:
                    return True
                    
    return False

def precise_collision_detection(positions, headings, valid_mask, shapes):
    Na, Nt = valid_mask.shape
    
    # 遍历每个时刻
    for t in range(31, Nt):
        # 获取当前时刻有效的代理
        valid_agents = np.where(valid_mask[:,t])[0]
        if len(valid_agents) < 2:
            continue
            
        # 检查所有可能的代理对
        for i, agent1 in enumerate(valid_agents[:-1]):
            for agent2 in valid_agents[i+1:]:
                # 直接构造旋转矩形并检查碰撞
                rect1 = ((positions[agent1,t,0], positions[agent1,t,1]), 
                        (shapes[agent1,0], shapes[agent1,1]),
                        np.rad2deg(headings[agent1,t]))
                rect2 = ((positions[agent2,t,0], positions[agent2,t,1]),
                        (shapes[agent2,0], shapes[agent2,1]), 
                        np.rad2deg(headings[agent2,t]))
                
                if cv2.rotatedRectangleIntersection(rect1, rect2)[0]:
                    # print(f"Precise: Collision detected between agent {agent1} and agent {agent2} at time {t}")
                    return True
    return False

            
def plot_dynamic_scenario(positions, headings, valid_mask, shapes):
    """动态绘制场景
    
    Args:
        positions: [Na,Nt,2] 所有代理在所有时刻的位置
        headings: [Na,Nt] 所有代理在所有时刻的朝向角
        valid_mask: [Na,Nt] 代理在各时刻是否有效的布尔数组
        shapes: [Na,2] 所有代理的尺寸(长度和宽度)
    """
    import matplotlib
    matplotlib.use('TkAgg')  # 在导入pyplot之前设置后端
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import matplotlib.patches as patches
    
    # 创建图形和坐标轴
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_aspect('equal')
    
    # 计算场景边界
    valid_positions = positions[valid_mask]
    if len(valid_positions) > 0:
        x_min, y_min = np.min(valid_positions, axis=0) - 10
        x_max, y_max = np.max(valid_positions, axis=0) + 10
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
    
    # 用于存储所有矩形patch的列表
    rect_patches = []
    
    def init():
        """初始化函数"""
        ax.set_title('Scenario Animation')
        ax.grid(True)
        return []
    
    def update(frame):
        """更新函数"""
        # 清除之前的所有patch
        for patch in rect_patches:
            patch.remove()
        rect_patches.clear()
        
        # 获取当前帧的有效代理
        valid_agents = np.where(valid_mask[:,frame])[0]
        
        # 为每个有效代理创建矩形
        for agent in valid_agents:
            # 创建旋转矩形
            rect = patches.Rectangle(
                (positions[agent,frame,0] - shapes[agent,0]/2,
                 positions[agent,frame,1] - shapes[agent,1]/2),
                shapes[agent,0], shapes[agent,1],
                angle=np.rad2deg(headings[agent,frame]),
                color='blue' if agent != 0 else 'red',  # ego车设为红色
                alpha=0.5,
                label=f'Agent {agent}' if agent == 0 else None
            )
            ax.add_patch(rect)
            rect_patches.append(rect)
            
            # 添加代理ID标签
            ax.text(positions[agent,frame,0], 
                   positions[agent,frame,1], 
                   f'{agent}',
                   horizontalalignment='center',
                   verticalalignment='center',
                   color='white' if agent == 0 else 'black')
        
        # 添加时间戳
        timestamp = ax.text(0.02, 0.98, f'Frame: {frame}',
                          transform=ax.transAxes,
                          verticalalignment='top')
        rect_patches.append(timestamp)
        
        if frame == 31:  # 第一帧添加图例
            ax.legend()
            
        return rect_patches
    
    # 创建动画
    Na, Nt = valid_mask.shape
    anim = FuncAnimation(
        fig, update, init_func=init,
        frames=range(31, Nt),  # 从第31帧开始
        interval=100,  # 100ms per frame
        blit=True,
        repeat=False
    )
    
    plt.show()

def validation(sm):
    """执行单个planner的任务"""
    import time

    quick_collision_count = 0
    precise_collision_count = 0
    
    quick_times = []
    precise_times = []
    quick_results = []
    precise_results = []
    
    for scene_idx, scene in enumerate(sm.tasks):
        # 检测当前场景是否存在发生碰撞的情况
        positions = scene.additional_info['positions']  # [Na,Nt,2]
        valid_mask = scene.additional_info['valid_mask']  # [Na,Nt] 
        shapes = scene.additional_info['shapes']  # [Na,2]
        headings = scene.additional_info['headings']  # [Na,Nt]
        
        # 测试快速碰撞检测
        start = time.time()
        quick_collision = quick_collision_detection(positions, headings, valid_mask, shapes)
        quick_time = time.time() - start
        quick_times.append(quick_time)
        quick_results.append(quick_collision)
        
        # 测试精确碰撞检测
        start = time.time()
        precise_collision = precise_collision_detection(positions, headings, valid_mask, shapes)
        precise_time = time.time() - start
        precise_times.append(precise_time)
        precise_results.append(precise_collision)
        
        if quick_collision:
            quick_collision_count += 1
        if precise_collision:
            precise_collision_count += 1

        if quick_collision != precise_collision:
            plot_dynamic_scenario(positions, headings, valid_mask, shapes)
            
    # 计算平均时间
    avg_quick_time = sum(quick_times) / len(quick_times)
    avg_precise_time = sum(precise_times) / len(precise_times)
    
    # 计算结果一致性
    quick_accuracy = sum([1 for q,p in zip(quick_results, precise_results) if q == p]) / len(quick_results)
    
    print("\n性能评估结果:")
    print(f"快速检测平均时间: {avg_quick_time:.4f}秒")
    print(f"精确检测平均时间: {avg_precise_time:.4f}秒") 
    print(f"快速检测准确率: {quick_accuracy*100:.1f}%")
    print(f"快速检测碰撞次数: {quick_collision_count}")
    print(f"精确检测碰撞次数: {precise_collision_count}")
    print(f"\n相对精确检测的性能提升:")
    print(f"快速检测: {avg_precise_time/avg_quick_time:.1f}倍")


def get_collision_rate(sm, config, logger=None):
    # 用于存储所有场景的碰撞检测结果
    collision_results = {}
    
    for scene_idx, scene in enumerate(sm.tasks):
        try:
            # 检测当前场景是否存在发生碰撞的情况
            positions = scene.additional_info['positions']  # [Na,Nt,2]
            valid_mask = scene.additional_info['valid_mask']  # [Na,Nt] 
            shapes = scene.additional_info['shapes']  # [Na,2]
            headings = scene.additional_info['headings']  # [Na,Nt]
            predict_mask = scene.additional_info['predict_mask']  # [Na]

            # 测试快速碰撞检测
            is_collision = quick_collision_detection(positions, headings, valid_mask, shapes, predict_mask)
            
            # 存储当前场景的碰撞结果
            # scene_name = f"{scene.type}_{scene.num}_{scene.name}"
            scene_name = scene.name
            collision_results[scene_name] = {
                'is_collision': 1 if is_collision else 0
            }
        except Exception as e:
            if logger:
                logger.error(f"Error detecting collision in {scene.num}-{scene.name}: {repr(e)}")
    
    return collision_results


if __name__ == '__main__':
    import sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    from ScenarioManager.ScenarioManagerForISG import ScenarioManagerForISG

    config = {
        'tasks': [],
        'warmup': 31,
        'skipExist': False,
        'visualize': False,
    }
    sm = ScenarioManagerForISG(SCENE_DIR, GT_DIR, config)

    # 计算碰撞率
    get_collision_rate(sm, config)