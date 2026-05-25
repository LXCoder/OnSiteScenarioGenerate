import pickle
import matplotlib.pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET
import math
import imageio
import os
import glob

# 解析车道宽度函数
def get_lane_width(lane, s):
    """根据 s 坐标计算车道宽度，支持多项式插值"""
    widths = lane.findall('width')
    if not widths:
        return 0
    suitable_widths = [w for w in widths if float(w.get('sOffset')) <= s]
    width_elem = max(suitable_widths, key=lambda w: float(w.get('sOffset'))) if suitable_widths else widths[0]
    a = float(width_elem.get('a'))
    b = float(width_elem.get('b'))
    c = float(width_elem.get('c'))
    d = float(width_elem.get('d'))
    ds = s - float(width_elem.get('sOffset'))
    return a + b * ds + c * ds**2 + d * ds**3

# 处理几何形状，返回点列表 (x, y, s, hdg)
def process_geometry(geom, num_points=20):
    """将几何形状（直线或弧线）离散化为点"""
    s_start = float(geom.get('s'))
    x = float(geom.get('x'))
    y = float(geom.get('y'))
    hdg = float(geom.get('hdg'))
    length = float(geom.get('length'))
    points = []
    
    if geom.find('line') is not None:
        for i in range(num_points + 1):
            u = i * length / num_points
            s = s_start + u
            px = x + u * math.cos(hdg)
            py = y + u * math.sin(hdg)
            points.append((px, py, s, hdg))
    elif geom.find('arc') is not None:
        k = float(geom.find('arc').get('curvature'))
        for i in range(num_points + 1):
            u = i * length / num_points
            s = s_start + u
            if k != 0:
                px = x + (1/k) * (math.sin(hdg + k*u) - math.sin(hdg))
                py = y - (1/k) * (math.cos(hdg + k*u) - math.cos(hdg))
                heading = hdg + k * u
            else:
                px = x + u * math.cos(hdg)
                py = y + u * math.sin(hdg)
                heading = hdg
            points.append((px, py, s, heading))
    return points

# 获取道路参考线
def get_reference_line(road, num_points_per_geom=20):
    """从道路的 planView 中提取参考线点"""
    geometries = road.find('planView').findall('geometry')
    ref_line = []
    for geom in geometries:
        points = process_geometry(geom, num_points_per_geom)
        if ref_line:
            ref_line.extend(points[1:])  # 跳过重复点
        else:
            ref_line.extend(points)
    return ref_line

# 解析 .xodr 文件，提取道路边界
def parse_xodr(xodr_path):
    """解析 .xodr 文件，返回道路的左右边界点列表"""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    boundaries = []
    
    for road in root.findall('road'):
        ref_line = get_reference_line(road)
        lane_sections = sorted(road.find('lanes').findall('laneSection'), 
                              key=lambda ls: float(ls.get('s')))
        if not lane_sections:
            continue
        
        left_boundary = []
        right_boundary = []
        current_ls_index = 0
        
        for p in ref_line:
            px, py, s, hdg = p
            while (current_ls_index < len(lane_sections) - 1 and 
                   s >= float(lane_sections[current_ls_index + 1].get('s'))):
                current_ls_index += 1
            lane_section = lane_sections[current_ls_index]
            
            left_lanes = lane_section.find('left').findall('lane') if lane_section.find('left') else []
            right_lanes = lane_section.find('right').findall('lane') if lane_section.find('right') else []
            
            ls_s = float(lane_section.get('s'))
            total_left_width = sum(get_lane_width(lane, s - ls_s) for lane in left_lanes)
            total_right_width = sum(get_lane_width(lane, s - ls_s) for lane in right_lanes)
            
            left_offset_x = total_left_width * math.sin(hdg)
            left_offset_y = -total_left_width * math.cos(hdg)
            left_bx = px + left_offset_x
            left_by = py + left_offset_y
            left_boundary.append((left_bx, left_by))
            
            right_offset_x = -total_right_width * math.sin(hdg)
            right_offset_y = total_right_width * math.cos(hdg)
            right_bx = px + right_offset_x
            right_by = py + right_offset_y
            right_boundary.append((right_bx, right_by))
        
        boundaries.append(left_boundary)
        boundaries.append(right_boundary)
    
    return boundaries

# 生成轨迹 GIF
def generate_trajectory_gif(xodr_path, trajectory_pkl_path, output_gif_path, fps=5):
    """生成轨迹 GIF 动画，主车橙色，其他车辆蓝色，标注车辆 ID，遵循 valid_mask"""
    # 解析地图数据
    boundaries = parse_xodr(xodr_path)
    
    # 加载轨迹数据
    with open(trajectory_pkl_path, 'rb') as f:
        data = pickle.load(f)
    positions = data['positions']  # (num_vehicles, num_timesteps, 2)
    valid_mask = data['valid_mask']  # (num_vehicles, num_timesteps)
    headings = data['headings']  # (num_vehicles, num_timesteps)
    shapes = data['shapes']  # (num_vehicles, 2)
    ids = data['ids']  # 车辆 ID 列表，包含 'Ego'
    
    # 计算全局坐标范围
    valid_positions = positions[valid_mask]  # 只取有效位置
    if len(valid_positions) > 0:
        x_min, y_min = np.min(valid_positions, axis=0)
        x_max, y_max = np.max(valid_positions, axis=0)
    else:
        x_min, y_min, x_max, y_max = 0, 0, 0, 0
    padding = 20  # 根据车辆尺寸调整的边界填充
    x_min -= padding
    x_max += padding
    y_min -= padding
    y_max += padding
    
    # 生成每一帧
    num_vehicles, num_timesteps, _ = positions.shape
    temp_images = []
    os.makedirs(os.path.dirname(output_gif_path) or '.', exist_ok=True)
    
    for t in range(num_timesteps):
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # 绘制道路网络
        for boundary in boundaries:
            xs = [p[0] for p in boundary]
            ys = [p[1] for p in boundary]
            ax.plot(xs, ys, color='gray', linewidth=1, alpha=0.5, zorder=1)
        
        # 绘制轨迹和车辆
        for i in range(num_vehicles):
            # 确定颜色：主车橙色，其他车辆蓝色
            color = 'orange' if ids[i] == 'Ego' else 'blue'
            
            # 绘制历史轨迹（仅包含有效点）
            valid_up_to_t = valid_mask[i, :t+1]
            x = positions[i, :t+1, 0][valid_up_to_t]
            y = positions[i, :t+1, 1][valid_up_to_t]
            if len(x) > 0:
                ax.plot(x, y, color=color, linestyle='-', linewidth=2, alpha=0.7, zorder=2)
            
            # 绘制车辆矩形和 ID 标签（仅当当前时刻有效）
            if valid_mask[i, t]:
                px, py = positions[i, t]
                theta = headings[i, t]
                length, width = shapes[i]
                half_l, half_w = length / 2, width / 2
                
                corners = np.array([[-half_l, -half_w],
                                   [half_l, -half_w],
                                   [half_l, half_w],
                                   [-half_l, half_w]])
                
                rotation = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
                
                rotated_corners = np.dot(corners, rotation.T) + np.array([px, py])
                
                ax.fill(rotated_corners[:, 0], rotated_corners[:, 1],
                        color=color, alpha=0.9, zorder=3, edgecolor='black')
                
                # 标注车辆 ID（在车辆顶部中心，稍向上偏移）
                label_x = px
                label_y = py + half_w + 1  # 偏移到车辆顶部上方
                ax.text(label_x, label_y, ids[i], fontsize=8, ha='center', va='bottom',
                        color='black', zorder=4, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'))
        
        # 设置固定坐标轴
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')
        plt.title(f'Time step: {t}')
        plt.tight_layout()
        
        # 保存临时图像
        temp_path = f'temp2_frame_{t}.png'
        plt.savefig(temp_path, dpi=250, bbox_inches='tight')
        temp_images.append(temp_path)
        plt.close()
    
    # 生成 GIF
    images = [imageio.imread(img) for img in temp_images]
    imageio.mimsave(output_gif_path, images, fps=fps)
    
    # 清理临时文件
    for img in temp_images:
        os.remove(img)


def batch_visualize_outputs(map_root, output_dir, gif_output_dir, fps=5):
    """
    Batch visualize all .pkl files in output_dir, matching with .xodr files in map_root subdirectories.
    Skips generation if GIF already exists.

    Parameters:
    - map_root: Path to the root directory containing subfolders with .xodr files (第五赛道_B卷)
    - output_dir: Path to the directory containing .pkl output files (output-v5)
    - gif_output_dir: Directory to save generated GIFs
    - fps: Frames per second for GIFs
    """
    # Ensure output directory for GIFs exists
    os.makedirs(gif_output_dir, exist_ok=True)

    # Get all .pkl files in output_dir
    pkl_files = glob.glob(os.path.join(output_dir, "*.pkl"))
    if not pkl_files:
        print(f"No .pkl files found in {output_dir}")
        return

    # Process each .pkl file
    for pkl_path in pkl_files:
        # Extract base filename (e.g., '0852follow789' from '0852follow789_output.pkl')
        pkl_filename = os.path.basename(pkl_path)
        base_name = pkl_filename.replace("_output.pkl", "")

        # Define output GIF path
        gif_filename = f"{base_name}.gif"
        gif_path = os.path.join(gif_output_dir, gif_filename)

        # Skip if GIF already exists
        if os.path.exists(gif_path):
            print(f"Skipping {pkl_filename} - GIF already exists")
            continue

        # Search for the corresponding .xodr file in map_root subdirectories
        xodr_path = None
        for root, dirs, files in os.walk(map_root):
            for file in files:
                if file == f"{base_name}.xodr":
                    xodr_path = os.path.join(root, file)
                    break
            if xodr_path:
                break

        if not xodr_path:
            print(f"No matching .xodr file found for {pkl_filename}")
            continue

        print(f"Processing {pkl_filename} with map {xodr_path} -> {gif_filename}")

        try:
            # Generate GIF using the provided function
            generate_trajectory_gif(xodr_path, pkl_path, gif_path, fps=fps)
            print(f"Successfully generated {gif_filename}")
        except Exception as e:
            print(f"Error processing {pkl_filename}: {str(e)}")


# Example usage
map_root = r"Onsite/第五赛道_B卷"
output_dir = r"selected_outputs_30-pre_score/score/2"
gif_output_dir = r"Onsite/visual-30/2"

batch_visualize_outputs(map_root, output_dir, gif_output_dir, fps=5)

# # 使用示例
# generate_trajectory_gif(
#     r"Onsite/第五赛道_B卷/0852follow789/0852follow789.xodr",
#     r"Onsite\output-v5\0852follow789_output.pkl",
#     "0852follow789.gif",
#     fps=5
# )