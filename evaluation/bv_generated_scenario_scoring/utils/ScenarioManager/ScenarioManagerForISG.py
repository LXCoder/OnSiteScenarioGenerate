import os
import pickle
import numpy as np

from .ScenarioInfo import ScenarioInfo
from .ScenarioManagerBase import ScenarioManagerBase

class ScenarioManagerForISG(ScenarioManagerBase):
    def __init__(self, scene_dir: str, gt_dir: str, config: dict):
        super().__init__({'skipExist': False})

        self.scene_dir = scene_dir
        self.gt_dir = gt_dir
        tasks = config.get('tasks', [])
        self.warmup = config.get('warmup', 31)
        self.scenario_type = "REPLAY"

        scenes = []
        if tasks:
            for task in tasks:
                scene_file = f"{task}_output.pkl"
                scene_path = os.path.join(scene_dir, scene_file)
                if os.path.exists(scene_path):
                    scenes.append(scene_file)
                else:
                    print(f"[LOAD SENARIO ERROR]: Cannot find scene file {scene_file}!")
        else:
            for scene in sorted(os.listdir(scene_dir)):
                if not scene.startswith('.') and scene.endswith('_output.pkl'):
                    scenes.append(scene)

        for scene_file in scenes:
            self.cur_scene_num += 1
            scene_path = os.path.join(scene_dir, scene_file)
            try:
                cur_scene = self.get_scenario_info(scene_path)

                if cur_scene:
                    self.tasks.append(cur_scene)
                else:
                    print(f"[LOAD SENARIO ERROR]: Cannot find ground truth for task {scene_file}!")
                    # # 删除无效的场景文件
                    # print(f"[LOAD SENARIO ERROR]: {scene_path}")
                    # os.remove(scene_path)
            except Exception as e:
                print(f"[LOAD SENARIO ERROR]: Load scene {scene_file} with error {repr(e)}")

        self.tot_scene_num = len(self.tasks)

    @staticmethod
    def _calculate_velocity_acceleration(valid_mask, positions, dt=0.1):
        na, nt, _ = positions.shape
        
        # 初始化标量速度和加速度
        scalar_velocities = np.zeros((na, nt))  # [na, nt]
        scalar_accelerations = np.zeros((na, nt))  # [na, nt]
        # 初始化矢量速度
        vector_velocities = np.zeros((na, nt, 2))  # [na, nt, 2]
        
        for i in range(na):
            # 获取有效状态的索引
            valid_indices = np.where(valid_mask[i])[0]
            if len(valid_indices) < 3:
                continue  # 如果有效状态少于3，无法计算速度或加速度
            
            # 计算速度向量
            valid_positions = positions[i, valid_indices]  # [num_valid, 2]
            velocity_vectors = (valid_positions[1:] - valid_positions[:-1]) / dt  # [num_valid-1, 2]
            
            # 计算标量速度（速度大小）
            valid_scalar_velocities = np.linalg.norm(velocity_vectors, axis=1)  # [num_valid-1]
            
            # 填充标量速度：第一个速度用第二个速度代替
            valid_scalar_velocities = np.concatenate(([valid_scalar_velocities[0]], valid_scalar_velocities), axis=0)
            # 确保长度匹配
            if len(valid_scalar_velocities) > len(valid_indices):
                valid_scalar_velocities = valid_scalar_velocities[:len(valid_indices)]
            # 更新标量速度到全局数组
            scalar_velocities[i, valid_indices] = valid_scalar_velocities
            
            # 填充矢量速度：第一个速度矢量用第二个速度矢量代替
            valid_vector_velocities = np.concatenate(([velocity_vectors[0]], velocity_vectors), axis=0)  # [num_valid, 2]
            # 确保长度匹配
            if len(valid_vector_velocities) > len(valid_indices):
                valid_vector_velocities = valid_vector_velocities[:len(valid_indices)]
            # 更新矢量速度到全局数组
            vector_velocities[i, valid_indices] = valid_vector_velocities
            
            # 计算沿着运动方向的加速度（可以有正负值）
            if len(valid_scalar_velocities) >= 3:  # 需要至少3个速度值来计算加速度
                # 使用速度的标量值计算加速度，而不是速度向量
                # 这样可以得到沿着运动方向的加速度（加速为正，减速为负）
                valid_scalar_accelerations = (valid_scalar_velocities[2:] - valid_scalar_velocities[1:-1]) / dt
                
                # 填充加速度：前两个时间步的加速度设为0
                valid_scalar_accelerations = np.concatenate(([0, 0], valid_scalar_accelerations), axis=0)
                # 确保长度匹配
                if len(valid_scalar_accelerations) > len(valid_indices):
                    valid_scalar_accelerations = valid_scalar_accelerations[:len(valid_indices)]
                # 更新标量加速度到全局数组
                scalar_accelerations[i, valid_indices] = valid_scalar_accelerations
        
        return scalar_velocities, scalar_accelerations, vector_velocities

    def get_scenario_info(self, scene_path: dict) -> ScenarioInfo:
        with open(scene_path, 'rb') as handle:
            scene_info = pickle.load(handle)
        
        scene_name = scene_info['scene_name']
        gt_scene_dir = os.path.join(self.gt_dir, scene_name)
        if not os.path.exists(gt_scene_dir):
            return None

        xodr_path = self._find_file_with_suffix(gt_scene_dir, '.xodr')
        gt_path = self._find_file_with_suffix(gt_scene_dir, '_gt.pkl')
        with open(gt_path, 'rb') as handle:
            gt_info = pickle.load(handle)

        predict_mask = gt_info['predict_mask']
        valid_mask = gt_info['valid_mask']
        valid_mask[predict_mask, self.warmup:] = scene_info['valid_mask'][predict_mask, self.warmup:]
        positions = gt_info['positions']
        positions[predict_mask, self.warmup:] = scene_info['positions'][predict_mask, self.warmup:]
        headings = gt_info['headings']
        headings[predict_mask, self.warmup:] = scene_info['headings'][predict_mask, self.warmup:]
        velocities, accelerations, vector_velocities = self._calculate_velocity_acceleration(valid_mask, positions)

        return ScenarioInfo(
            num = self.cur_scene_num,
            name = scene_name,
            type = self.scenario_type,
            source_file = {
                "xodr": xodr_path, 
                "pkl": scene_path,
                "gt": gt_path,
                "json": ""
            },
            task_info = {
                "startPos": gt_info["positions"][gt_info['ids'].index('Ego'), self.warmup-1].tolist(), 
                "targetPos": [[gt_info["goal"][0], gt_info["goal"][2]], [gt_info["goal"][1], gt_info["goal"][3]]], 
                "waypoints": [], 
                "dt": round(1.0 / gt_info["sim_freq"], 3),
            },
            additional_info = {
                'ids': gt_info['ids'],
                'types': gt_info['types'],
                'shapes': gt_info['shapes'],
                'predict_mask': gt_info['predict_mask'],
                'valid_mask': valid_mask,
                'positions': positions,
                'headings': headings,
                'velocities': velocities,
                'accelerations': accelerations,
                'vector_velocities': vector_velocities,
            }
        )
