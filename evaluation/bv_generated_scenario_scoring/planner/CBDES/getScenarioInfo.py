import numpy as np
from . import utils_cbdes
import pandas as pd
import math
from scipy import interpolate


def getCords(x, y, angle, ReferencePath, imwidth, imheight):
    if abs(angle - 0) < 10e-2 or abs(angle - np.pi) < 10e-2:
        x_inter = [-100, 100]
        y_inter = [y, y]
    elif abs(angle - np.pi / 2) < 10e-2 or abs(angle - 3 * np.pi / 2) < 10e-2:
        x_inter = [x, x + 10e-5]
        y_inter = [-100, 100]
    else:
        x1_length = (x-imwidth) / math.cos(angle)
        y1_length = (y-imheight) / math.sin(angle)
        length = max(abs(x1_length), abs(y1_length))
        endx1 = x + length * math.cos(angle)
        endy1 = y + length * math.sin(angle)

        x2_length = (x-imwidth) / math.cos(angle+np.pi)
        y2_length = (y-imheight) / math.sin(angle+np.pi)
        length = max(abs(x2_length), abs(y2_length))
        endx2 = x + length * math.cos(angle+np.pi)
        endy2 = y + length * math.sin(angle+np.pi)
        if endx1 < endx2:
            x_inter = [endx1, endx2]
            y_inter = [endy1, endy2]
        else:
            x_inter = [endx2, endx1]
            y_inter = [endy2, endy1]

    func1 = interpolate.UnivariateSpline(x_inter, y_inter, k=1, s=0)
    # PointY = func1(ReferencePath[:, 0])
    # f = interpolate.interp1d(x_inter, y_inter, kind='linear')
    # PointYY = f(ReferencePath[:, 0])
    CrossingPointInTrajectory = ReferencePath[abs(ReferencePath[:, 1] - func1(ReferencePath[:, 0])) == min(abs(ReferencePath[:, 1] - func1(ReferencePath[:, 0])))][0]
    return CrossingPointInTrajectory

def getCurvature(egoTrajectory):
    x_t = np.gradient(egoTrajectory[:, 0])
    y_t = np.gradient(egoTrajectory[:, 1])
    vel = np.array([[x_t[i], y_t[i]] for i in range(x_t.size)])
    speed = np.sqrt(x_t * x_t + y_t * y_t)
    tangent = np.array([1 / speed] * 2).transpose() * vel
    ss_t = np.gradient(speed)
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)
    curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t) ** 1.5
    return curvature_val, tangent


def YawCalibration(VehicleInfo):
    for index in VehicleInfo.index:
        VehicleInfo.at[index, 'yaw'] = AngleCalibration(VehicleInfo.at[index, 'yaw'])
    return VehicleInfo


def AngleCalibration(Angle):
    if Angle < 0:
        Angle = Angle + np.pi * 2
    elif Angle > np.pi * 2:
        Angle = Angle - np.pi * 2
    if Angle > np.pi * 2 or Angle < 0:
        raise Exception('Ego Vehicle Yaw Error')
    return Angle


class ScenarioExtract:
    def __init__(self, LaneWidth=4):
        self.LaneWidth = LaneWidth

    def getEgoTrajectory(self, task_info, road_data):
        goalPointX = [task_info['targetPos'][0][0], task_info['targetPos'][1][0]]
        goalPointY = [task_info['targetPos'][0][1], task_info['targetPos'][1][1]]
        ego_vehicle_x = task_info['startPos'][0]
        ego_vehicle_y = task_info['startPos'][1]
        '''
        goalPointX = observation['test_setting']['goal']['x']
        goalPointY = observation['test_setting']['goal']['y']
        ego_vehicle_x = observation['vehicle_info']['ego']['x']
        ego_vehicle_y = observation['vehicle_info']['ego']['y']
        '''
        # Find Ego Vehicle Lane
        # Find Goal Lane
        # EgoVehicleLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        # GoalLaneIndexDict = dict.fromkeys([discrete_lane.lane_id for discrete_lane in road_data.discretelanes])
        EgoVehicleLaneIndexDict = dict()
        GoalLaneIndexDict = dict()
        EgoVehicleLaneDataList = list()
        GoalLaneDataList = list()
        for discrete_lane in road_data.discretelanes:
            center_lane = discrete_lane.center_vertices
            for point in center_lane:
                LaneWidth = 2 * self.LaneWidth
                if ((point[0] - ego_vehicle_x) ** 2 + (point[1] - ego_vehicle_y) ** 2) ** 0.5 < LaneWidth:
                    EgoVehicleLaneIndexDict[discrete_lane.lane_id] = point
                    EgoVehicleLaneDataList.append(discrete_lane)
                    break
                if goalPointX[0] - LaneWidth < point[0] < goalPointX[1] + LaneWidth and goalPointY[0] - LaneWidth < \
                        point[1] < goalPointY[1] + LaneWidth:
                    GoalLaneIndexDict[discrete_lane.lane_id] = point
                    GoalLaneDataList.append(discrete_lane)
                    break

        # Find Turning Lane ID
        TrajectoryList = pd.DataFrame(columns=['InLaneID', 'InLaneData',
                                               'MiddleLaneID', 'MiddleLaneData',
                                               'OutLaneID', 'OutLaneData', 'TurningDirection',
                                               'Trajectory', 'LaneStatus','StoppingLinePoint'])
        TurningLane = []
        TurningLaneCount = 0
        for EgoVehicleLaneDataTemp in EgoVehicleLaneDataList:
            for CandidateTurningLane in EgoVehicleLaneDataTemp.successor:
                for GoalLaneDataTemp in GoalLaneDataList:
                    if CandidateTurningLane in GoalLaneDataTemp.predecessor:
                        TurningLane.append(CandidateTurningLane)
                        EgoVehicleLaneData = EgoVehicleLaneDataTemp
                        GoalLaneData = GoalLaneDataTemp
                        TrajectoryList.at[TurningLaneCount, 'InLaneID'] = EgoVehicleLaneDataTemp.lane_id
                        TrajectoryList.at[TurningLaneCount, 'InLaneData'] = EgoVehicleLaneDataTemp.center_vertices
                        TrajectoryList.at[TurningLaneCount, 'MiddleLaneID'] = CandidateTurningLane
                        TrajectoryList.at[TurningLaneCount, 'OutLaneID'] = GoalLaneDataTemp.lane_id
                        TrajectoryList.at[TurningLaneCount, 'OutLaneData'] = GoalLaneDataTemp.center_vertices
                        TurningLaneCount = TurningLaneCount + 1

        # Get middle lane
        for index in TrajectoryList.index:
            for discrete_lane in road_data.discretelanes:
                if discrete_lane.lane_id == TrajectoryList.at[index, 'MiddleLaneID']:
                    TrajectoryList.at[index, 'MiddleLaneData'] = discrete_lane.center_vertices
                    # Get trajectory
                    Trajectory_1 = TrajectoryList.at[index, 'InLaneData']
                    Trajectory_2 = TrajectoryList.at[index, 'MiddleLaneData']
                    Trajectory_3 = TrajectoryList.at[index, 'OutLaneData']
                    Trajectory_12 = np.concatenate((Trajectory_1, Trajectory_2))
                    Trajectory = np.concatenate((Trajectory_12, Trajectory_3))
                    TrajectoryList.at[index, 'Trajectory'] = Trajectory
                    # Ego vehicle in which lane
                    road_structure = utils_cbdes.RoadStructure(road_data)
                    LaneStatus = utils_cbdes.EgoLocation(road_structure, TrajectoryList.at[index, 'InLaneID'])

                    # get turn direction
                    TrajectoryDirection = np.zeros(len(Trajectory) - 1)
                    for pointIndex in range(len(Trajectory) - 1):
                        LastPoint = Trajectory[pointIndex]
                        ThisPoint = Trajectory[pointIndex + 1]
                        TrajectoryDirection[pointIndex] = math.atan2(ThisPoint[1] - LastPoint[1], ThisPoint[0] - LastPoint[0])
                        TrajectoryDirection[pointIndex] = AngleCalibration(TrajectoryDirection[pointIndex])
                    TrajectoryDirectionDelta = np.gradient(TrajectoryDirection)
                    TrajectoryDirectionDelta[abs(TrajectoryDirectionDelta) < 10e-5] = 0
                    PositiveCount = len(np.where(TrajectoryDirectionDelta > 0)[0])
                    ZeroCount = len(np.where(TrajectoryDirectionDelta == 0)[0])
                    NegativeCount = len(np.where(TrajectoryDirectionDelta < 0)[0])
                    if (NegativeCount / len(TrajectoryDirectionDelta)) > 1/3:
                        TurningDirection = 'Right'
                    elif (PositiveCount / len(TrajectoryDirectionDelta)) > 1/3:
                        TurningDirection = 'Left'
                    else:
                        TurningDirection = 'Straight'

                    TrajectoryList.at[index, 'LaneStatus'] = LaneStatus
                    TrajectoryList.at[index, 'StoppingLinePoint'] = TrajectoryList.at[index, 'MiddleLaneData'][0]
                    TrajectoryList.at[index, 'TurningDirection'] = TurningDirection
                    break
        return TrajectoryList
