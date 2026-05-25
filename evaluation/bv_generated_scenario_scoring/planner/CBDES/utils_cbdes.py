from .Bezier import Bezier
import numpy as np
import matplotlib.pyplot as plt


def DoubleLaneChange(P1, P2, P3, dis_k=0.35):
    direction = P2 - P1
    direction = direction / (np.linalg.norm(direction))
    curve1 = LaneChange(P1, P3, dis_k, direction, direction)
    curve2 = LaneChange(P3, P2, dis_k, direction, direction)
    curve = np.concatenate([curve1, curve2])
    return curve


def LaneChange(PS, PE, dis_k, directionS, directionE):
    dis = np.linalg.norm(PE - PS)
    points = np.array([PS, PS + directionS * dis * dis_k, PE - directionE * dis * dis_k, PE])
    t_points = np.arange(0, 1, 0.01)
    curve = Bezier.Curve(t_points, points)
    return curve

def EgoLocation(road_structure,lane_id):
    road = lane_id.split('.')[0]
    lane = int(lane_id.split('.')[2])
    
    if len(road_structure[road]) == 1:
        return 'NoChange'
    elif lane > 0 and lane == min(list(filter(lambda x: x > 0, road_structure[road]))):
        return 'Left'
    elif lane < 0 and lane == max(list(filter(lambda x: x < 0, road_structure[road]))):
        return 'Left'
    elif lane > 0 and lane == max(list(filter(lambda x: x > 0, road_structure[road]))):
        return 'Right'
    elif lane < 0 and lane == min(list(filter(lambda x: x < 0, road_structure[road]))):
        return 'Right'
    else:
        return 'Middle'

def Maptype(observation):
    map_type = observation['test_setting']['map_type']
    return map_type

def RoadStructure(road_data):
    road_structure = {}
    for discrete_lane in road_data.discretelanes:
        road = (discrete_lane.lane_id).split('.')[0]
        lane = int((discrete_lane.lane_id).split('.')[2])
        if road_structure.get(road, False):
            road_structure[road].append(lane)
        else:
            road_structure[road] = [lane]
    
    return road_structure

def cmc_RoadStructure(road_data):
    road_structure = {}
    for discrete_lane in road_data.discretelanes:
        road = (discrete_lane.lane_id).split('.')[0]
        laneSegment = (discrete_lane.lane_id).split('.')[1]
        lane = int((discrete_lane.lane_id).split('.')[2])
        if road_structure.get(road, False):
            if road_structure.get(laneSegment, False):
                road_structure[road][laneSegment].append(lane)
            else:
                road_structure[road][laneSegment] = [lane]
        else:
            road_structure[road] = {}
            road_structure[road][laneSegment] = [lane]
    
    return road_structure

if __name__ == "__main__":
    P1 = np.array([0, 0])
    P2 = np.array([20, 20])
    P3 = np.array([13, 7])
    points = np.array([P1, P3, P2])
    curve = DoubleLaneChange(P1, P2, P3)

    plt.plot(curve[:, 0], curve[:, 1], 'r')
    plt.plot(points[:, 0], points[:, 1], 'yx:')

    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(visible=True, which='major', axis='both')
    plt.show()
