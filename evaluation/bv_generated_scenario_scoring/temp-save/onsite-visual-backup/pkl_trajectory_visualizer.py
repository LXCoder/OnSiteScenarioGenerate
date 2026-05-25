import os
import pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免tkinter问题
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import seaborn as sns
import glob

class PKLTrajectoryVisualizer:
    """专门用于可视化pkl轨迹文件并保存为gif的类"""
    
    def __init__(self):
        self.data = None
        self.fig = None
        self.ax = None
        # 使用seaborn的crest色盘
        self.crest_colors = sns.color_palette("crest", 15)  # 获取15种crest颜色
        # 设置matplotlib参数
        plt.rcParams['figure.max_open_warning'] = 0  # 关闭图形数量警告
        
    def load_pkl_file(self, pkl_path):
        """加载pkl轨迹文件"""
        try:
            with open(pkl_path, 'rb') as f:
                self.data = pickle.load(f)
            print(f"Loading file: {pkl_path}")
            print(f"Scene name: {self.data['scene_name']}")
            print(f"Simulation frequency: {self.data['sim_freq']} Hz")
            print(f"Simulation duration: {self.data['sim_duration']} frames")
            print(f"Number of vehicles: {len(self.data['ids'])}")
            return True
        except Exception as e:
            print(f"Failed to load pkl file: {e}")
            return False
    
    def find_valid_frame_range(self, start_frame=1):
        """找到有效的帧范围，从start_frame开始到最后一个有车辆的帧"""
        valid_mask = self.data['valid_mask']
        
        # 找到最后一个有任何车辆有效的帧
        last_valid_frame = start_frame
        for frame_idx in range(start_frame, valid_mask.shape[1]):
            # 检查这一帧是否有任何车辆是有效的
            if np.any(valid_mask[:, frame_idx]):
                last_valid_frame = frame_idx
            
        print(f"Valid frame range: {start_frame} - {last_valid_frame}")
        return start_frame, last_valid_frame + 1  # +1 因为range不包含末尾
    
    def calculate_tight_bounds(self, start_frame, end_frame):
        """计算紧凑的绘图边界，只考虑有效帧范围内的位置"""
        positions = self.data['positions']
        valid_mask = self.data['valid_mask']
        shapes = self.data['shapes']
        
        # 获取指定帧范围内的所有有效位置点
        valid_positions = []
        max_vehicle_size = 0
        
        for agent_idx in range(positions.shape[0]):
            for frame_idx in range(start_frame, min(end_frame, positions.shape[1])):
                if valid_mask[agent_idx, frame_idx]:
                    valid_positions.append(positions[agent_idx, frame_idx])
                    # 记录最大车辆尺寸用于计算边距
                    vehicle_size = max(shapes[agent_idx])
                    max_vehicle_size = max(max_vehicle_size, vehicle_size)
        
        if valid_positions:
            valid_positions = np.array(valid_positions)
            x_min, y_min = np.min(valid_positions, axis=0)
            x_max, y_max = np.max(valid_positions, axis=0)
            
            # 使用车辆尺寸的1.5倍作为边距，确保完整显示
            margin = max(max_vehicle_size * 1.5, 5)  # 至少5米边距
            
            return {
                'x_min': x_min - margin,
                'x_max': x_max + margin,
                'y_min': y_min - margin,
                'y_max': y_max + margin
            }
        return None
    
    def setup_plot(self, bounds=None):
        """设置绘图环境"""
        if self.fig is None:
            self.fig, self.ax = plt.subplots(figsize=(12, 8))
            
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(f"Trajectory Visualization: {self.data['scene_name']}", fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        
        # 设置坐标轴范围
        if bounds:
            self.ax.set_xlim(bounds['x_min'], bounds['x_max'])
            self.ax.set_ylim(bounds['y_min'], bounds['y_max'])
        
        # 调整布局以减少空白
        plt.tight_layout()
    
    def draw_vehicle(self, x, y, heading, length, width, color, alpha=0.8, agent_id=""):
        """绘制车辆矩形"""
        # 计算矩形的左下角位置（考虑旋转）
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        
        # 车辆矩形的四个角点（相对于中心）
        half_length = length / 2
        half_width = width / 2
        
        # 计算旋转后的左下角位置
        rect_x = x - half_length * cos_h + half_width * sin_h
        rect_y = y - half_length * sin_h - half_width * cos_h
        
        # 创建车辆矩形
        rect = patches.Rectangle(
            (rect_x, rect_y),
            length, width,
            angle=np.degrees(heading),
            facecolor=color,
            edgecolor='black',
            alpha=alpha,
            linewidth=1.5
        )
        self.ax.add_patch(rect)
        
        # 将车辆ID显示在矩形框外面（上方偏移），不添加边框
        # 计算ID显示位置（在车辆上方）
        offset_distance = max(length, width) * 0.7  # 偏移距离
        id_x = x + offset_distance * sin_h  # 垂直于车辆朝向的偏移
        id_y = y + offset_distance * cos_h
        
        self.ax.text(id_x, id_y, str(agent_id), 
                    ha='center', va='center', 
                    fontsize=10, fontweight='bold',
                    color='black')
        
        # 绘制朝向箭头 - 从车辆中心开始
        arrow_length = length * 0.4
        arrow_end_x = x + arrow_length * cos_h
        arrow_end_y = y + arrow_length * sin_h
        
        self.ax.arrow(x, y, 
                     arrow_end_x - x, arrow_end_y - y,
                     head_width=width*0.25, head_length=length*0.15,
                     fc=color, ec='black', alpha=0.7, linewidth=1)
    
    def animate_frame(self, frame):
        """动画帧更新函数"""
        self.setup_plot(self.plot_bounds)
        
        positions = self.data['positions']
        headings = self.data['headings']
        valid_mask = self.data['valid_mask']
        shapes = self.data['shapes']
        ids = self.data['ids']
        
        # 绘制每个有效的代理
        vehicle_count = 0
        other_vehicle_idx = 0  # 用于其他车辆的颜色索引
        
        for agent_idx in range(positions.shape[0]):
            if valid_mask[agent_idx, frame]:
                x, y = positions[agent_idx, frame]
                heading = headings[agent_idx, frame]
                length, width = shapes[agent_idx]
                agent_id = ids[agent_idx]
                
                # Ego车用红色，其他车辆用crest色盘
                if agent_id == 'Ego':
                    color = 'red'
                else:
                    color = self.crest_colors[other_vehicle_idx % len(self.crest_colors)]
                    other_vehicle_idx += 1
                
                self.draw_vehicle(x, y, heading, length, width, color, agent_id=agent_id)
                vehicle_count += 1
        
        # 添加帧信息
        time_stamp = frame / self.data['sim_freq']
        info_text = f'Time: {time_stamp:.2f}s | Frame: {frame} | Vehicles: {vehicle_count}'
        self.ax.text(0.02, 0.98, info_text,
                    transform=self.ax.transAxes, fontsize=11,
                    bbox=dict(boxstyle="round,pad=0.4", facecolor="lightblue", alpha=0.8),
                    verticalalignment='top')
        
        # 添加简化的图例
        ego_patch = patches.Patch(color='red', label='Ego')
        other_patch = patches.Patch(color=self.crest_colors[0], label='Other Vehicles')
        self.ax.legend(handles=[ego_patch, other_patch], loc='upper right', 
                      bbox_to_anchor=(0.98, 0.98), fontsize=10)
    
    def create_gif(self, pkl_path, output_path, fps=10, start_frame=1):
        """创建gif动画"""
        if not self.load_pkl_file(pkl_path):
            return False
        
        try:
            # 找到有效的帧范围
            start_frame, end_frame = self.find_valid_frame_range(start_frame)
            
            if start_frame >= end_frame:
                print(f"No valid frames found after frame {start_frame}")
                return False
            
            # 计算紧凑的绘图边界
            self.plot_bounds = self.calculate_tight_bounds(start_frame, end_frame)
            if not self.plot_bounds:
                print("No valid positions found for plotting")
                return False
            
            print(f"Creating animation... Frame range: {start_frame} - {end_frame-1}")
            print(f"Plot bounds: X[{self.plot_bounds['x_min']:.1f}, {self.plot_bounds['x_max']:.1f}], Y[{self.plot_bounds['y_min']:.1f}, {self.plot_bounds['y_max']:.1f}]")
            
            # 创建图形，调整subplot参数以减少空白
            self.fig, self.ax = plt.subplots(figsize=(12, 8))
            plt.subplots_adjust(left=0.08, right=0.95, top=0.92, bottom=0.08)
            
            # 创建动画
            anim = FuncAnimation(
                self.fig, 
                self.animate_frame,
                frames=range(start_frame, end_frame),
                interval=1000//fps,  # 转换为毫秒
                repeat=False,
                blit=False
            )
            
            # 保存为gif
            print(f"Saving gif to: {output_path}")
            anim.save(output_path, writer='pillow', fps=fps, dpi=100)
            print(f"GIF saved successfully!")
            return True
            
        except Exception as e:
            print(f"Failed to save gif: {e}")
            return False
        finally:
            # 确保清理资源
            if self.fig:
                plt.close(self.fig)
                self.fig = None
                self.ax = None
            plt.clf()
            plt.cla()

def visualize_all_pkl_files(input_dir, output_dir, fps=10):
    """批量可视化指定目录下的所有pkl文件"""
    # 确保输出目录存在
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")
    
    # 查找所有pkl文件
    pkl_files = glob.glob(os.path.join(input_dir, "*.pkl"))
    
    if not pkl_files:
        print(f"No pkl files found in directory: {input_dir}")
        return
    
    print(f"Found {len(pkl_files)} pkl files")
    
    for i, pkl_file in enumerate(pkl_files, 1):
        print(f"\n[{i}/{len(pkl_files)}] Processing file: {os.path.basename(pkl_file)}")
        
        # 生成输出文件名
        base_name = os.path.splitext(os.path.basename(pkl_file))[0]
        output_path = os.path.join(output_dir, f"{base_name}.gif")
        
        # 检查是否已经存在
        if os.path.exists(output_path):
            print(f"File {output_path} already exists, skipping...")
            continue
        
        # 创建可视化器实例（每个文件单独创建，避免内存累积）
        visualizer = PKLTrajectoryVisualizer()
        success = visualizer.create_gif(pkl_file, output_path, fps=fps)
        
        if success:
            print(f"✓ Successfully generated: {os.path.basename(output_path)}")
        else:
            print(f"✗ Failed: {os.path.basename(pkl_file)}")
        
        # 强制垃圾回收
        del visualizer

def main():
    """主函数"""
    # 配置路径
    input_dir = "scenario/B"
    output_dir = "gifs/tgpo_mini"
    
    print("=== PKL Trajectory Visualization Tool ===")
    print(f"Input directory: {input_dir}")
    print(f"Output directory: {output_dir}")
    
    # 检查输入目录是否存在
    if not os.path.exists(input_dir):
        print(f"Error: Input directory {input_dir} does not exist")
        return
    
    # 开始批量处理
    visualize_all_pkl_files(input_dir, output_dir, fps=10)
    print("\n=== Processing completed ===")

if __name__ == "__main__":
    main() 