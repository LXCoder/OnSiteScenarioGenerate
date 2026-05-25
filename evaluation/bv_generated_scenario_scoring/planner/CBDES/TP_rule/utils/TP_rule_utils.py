import pandas as pd
import re
import numpy as np
from scipy.interpolate import interp1d

class History_rule:
    def __init__(self):
        self.df = pd.DataFrame(columns=['timestamp', 'x', 'y', 'trackID', 'yaw', 'v'])
        self.dt = 0.1
        self.target_dt = 0.1
        self.current_time = 0.0
        self.history_time = 2
        self.obs_len = 20
        self.feature = {}



    def update(self, observation):
        self.dt = observation.test_info["dt"]
        self.current_time = observation.test_info["t"]
        timestamp = self.current_time
        ego_x = observation.ego_info.x
        ego_y = observation.ego_info.y
        ego_yaw = observation.ego_info.yaw
        ego_v = observation.ego_info.v
        ego_trackID = "ego"
        new_row = {'timestamp': timestamp, 'x': ego_x, 'y': ego_y, 'trackID': ego_trackID, 'yaw': ego_yaw, 'v': ego_v}
        self.df = self.df.append(new_row, ignore_index=True)
        for type in observation.object_info:
            for trackID in observation.object_info[type]:
                x = observation.object_info[type][trackID].x
                y = observation.object_info[type][trackID].y
                yaw = observation.object_info[type][trackID].yaw
                v = observation.object_info[type][trackID].v
                if type == "vehicle" and (not trackID.startswith("car")):
                    new_id = "car" + trackID
                elif type == "bicycle" and (not trackID.startswith("bicycle")):
                    new_id = "bicycle" + trackID
                elif type == "pedestrian" and (not trackID.startswith("pedestrian")):
                    new_id = "pedestrian" + trackID
                else:
                    new_id = trackID
                new_row = {'timestamp': timestamp, 'x': x, 'y': y, 'trackID': new_id, 'yaw': yaw, 'v': v}
                self.df = self.df.append(new_row, ignore_index=True)
        self.df = self.df[self.df['timestamp'] >= timestamp-2]


    # EDIT: update pad_track function
    def pad_track(self, agent_df):
        xys = agent_df[["x", "y", "v", "yaw"]].values

        # 计算后续填充步数
        padding_behind = round((self.current_time - agent_df['timestamp'].values[-1]) / self.dt)
        
        # 计算前向填充步数
        padding_front = int(self.history_time / self.dt) + 1 - padding_behind - xys.shape[0]

        # 如果前向填充值小于0，使用切片裁剪数组
        if padding_front < 0:
            xys = xys[-padding_front:]  # 裁剪前面的数据，只保留最后 self.history_time 的数据
            padding_front = 0  # 将前向填充置为0

        # 使用 np.pad 填充数据，填充的方式是用边缘值
        padded_track = np.pad(xys, ((padding_front, padding_behind), (0, 0)), "edge")

        return padded_track
    # def pad_track(self,agent_df):
    #     xys = agent_df[["x", "y", "v", "yaw"]].values
    #     start_time = self.current_time - 2
        
    #     padding_behind = round((self.current_time - agent_df['timestamp'].values[-1]) / self.dt)
    #     padding_front = int(self.history_time/self.dt) + 1 - padding_behind - xys.shape[0]
        
    #     # print("padded_track:",len(xys), padding_behind, padding_front)
    #     padded_track = np.pad(xys,((padding_front, padding_behind),(0, 0)), "edge")
        
    #     return padded_track


    def reconstruct_trajectory(self, traj, dt_observed, dt_target):
        # print(dt_observed,dt_target)
        # 提取 x, y, mask 列
        x_observed = traj[:, 0]
        y_observed = traj[:, 1]
        v_observed = traj[:, 2]
        yaw_observed = traj[:, 3]
        # print(len(x_observed))
        timestamps_observed = np.arange(0, self.history_time + dt_observed, dt_observed)
        # print(len(timestamps_observed))
        timestamps_target = np.arange(0, self.history_time + dt_target, dt_target)
        # print(len(timestamps_target))
        f_x = interp1d(timestamps_observed, x_observed, kind='linear',axis=0)
        # print('fx')
        f_y = interp1d(timestamps_observed, y_observed, kind='linear',axis=0)
        # print('fy')
        f_v = interp1d(timestamps_observed, v_observed, kind='linear',axis=0)
        f_yaw = interp1d(timestamps_observed, yaw_observed, kind='linear',axis=0)
        # print('f_mask')
        x_target = f_x(timestamps_target)
        # print(len(x_target))
        y_target = f_y(timestamps_target)
        # print(len(y_target))
        v_target = f_v(timestamps_target)
        yaw_target = f_yaw(timestamps_target)
        timestamp = np.arange(0, self.obs_len * self.target_dt + self.target_dt, self.target_dt)
        reconstructed_traj = np.column_stack((x_target, y_target, v_target, yaw_target))
        return reconstructed_traj[-self.obs_len:]
            
    def get_agent_feature(self, df, target_id):
        # print("enter get_agent_feature")
        reconstructed_traj = []
        for track_id, remain_df in df.groupby('trackID'):
            if track_id == target_id:
                # print("enter get_agent_feature for")
                agent_track =self.pad_track(remain_df)
                # print("finish pad_track")
                reconstructed_traj = self.reconstruct_trajectory(agent_track, self.dt, self.target_dt)
                # print("finish reconstruct_trajectory")
            
        return reconstructed_traj

    def compute_feature(self, trackID):
        # print("enter compute_feature")
        df = self.df.copy(deep=True)
        # print("finish get_norm_center")
        traj = self.get_agent_feature(df, trackID)   
        # print("finish get_agent_feature")     
        return traj
       

    def get_all_vehicle_feature(self, observation, vehicle_interest_list):
 
        self.feature = {}
        for type in observation.object_info:
            for trackID in observation.object_info[type]:
                if type == "vehicle" and (not trackID.startswith("car")):
                    new_id = "car" + trackID
                elif type == "bicycle" and (not trackID.startswith("bicycle")):
                    new_id = "bicycle" + trackID
                elif type == "pedestrian" and (not trackID.startswith("pedestrian")):
                    new_id = "pedestrian" + trackID
                else:
                    new_id = trackID
                if new_id in vehicle_interest_list:
                    traj = self.compute_feature(new_id)
                    if len(traj) > 0:
                        self.feature[new_id] = traj
     
                
       
    
   


        
            
        

        
     