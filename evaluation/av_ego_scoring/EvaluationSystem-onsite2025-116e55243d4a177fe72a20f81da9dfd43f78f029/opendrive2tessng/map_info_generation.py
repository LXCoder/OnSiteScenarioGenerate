
from evaluateUtils.standard_parameter import Parameter
from opendrive2tessng import map_data_process

import json
import xml.dom.minidom
import re
import os


class mapInfoGeneration:
    def __init__(self):
        self.road_dict = {}

    def parse_openScenario(self, path_openScenario):  ##读取试题起终点信息
        """
        解析openScenario地图信息，存储到self.info.openScenario_info
        :param scenario_path: 存放比赛试题的文件位置
        :return:
        """
        # scenario_file = scenario_path
        # path_openScenario = r'C:\Users\Administrator\Desktop\ramp\scenario\1cutin10\1cutin10_exam.xosc'
        opens = xml.dom.minidom.parse(path_openScenario).documentElement
        openScenario_info = {}
        # 读取主车起点信息
        ego_node = opens.getElementsByTagName('Private')[0]
        ego_init = ego_node.childNodes[3].data
        ego_v, ego_x, ego_y, ego_head = [
            float(i.split('=')[1]) for i in ego_init.split(',')]
        for key, value in zip(["initial_x", "initial_y"], [ego_x, ego_y]):
            openScenario_info[key] = value
        # 读取主车终点信息
        goal_init = ego_node.childNodes[5].data
        goal = [float(i) for i in re.findall('-*\d+\.\d+', goal_init)]
        for key, value in zip(["goal_x", "goal_y"], [goal[:2], goal[2:]]):
            openScenario_info[key] = value
        return openScenario_info

    def goalPlaceInfoProcess(self, aim_place_file_path, aim_path_json_file_path=None):
        try:
            goalPlace = self.parse_openScenario(aim_place_file_path)
            # 处理成标准形式，[[左下x，左下y][右上x，右上y]]
            # 这里要注意，一定需要时第一个数字小于第三个数字，第二个数字小于第四个数字
            if goalPlace['goal_x'][0] < goalPlace['goal_x'][1]:
                x1 = goalPlace['goal_x'][0]
                x3 = goalPlace['goal_x'][1]
            else:
                x1 = goalPlace['goal_x'][1]
                x3 = goalPlace['goal_x'][0]
            if goalPlace['goal_y'][0] < goalPlace['goal_y'][1]:
                y2 = goalPlace['goal_y'][0]
                y4 = goalPlace['goal_y'][1]
            else:
                y2 = goalPlace['goal_y'][1]
                y4 = goalPlace['goal_y'][0]
            # standGoalPlace = [[goalPlace['goal_x'][0],goalPlace['goal_y'][0]],[goalPlace['goal_x'][1],goalPlace['goal_y'][1]]]
            standGoalPlace = [[x1,y2],[x3,y4]]
            self.road_dict["goalPlaceInfo"] = standGoalPlace
            print("得到结束位置信息:", standGoalPlace)
        except:
            # 3要评价的目标路径信息
            #print(Parameter.aim_path_json_file_path)
            try:
                #aim_path_json_file_path = Parameter.aim_path_json_file_path
                # 读取 JSON 文件
                with open(aim_path_json_file_path, "r") as aim_path_json_file:
                    # 使用 json.load() 方法加载 JSON 数据
                    self.aimPathJsonInforDict = json.load(aim_path_json_file)
                if "targetPos" in self.aimPathJsonInforDict:
                    targetPos = self.aimPathJsonInforDict["targetPos"]
                    # json文件是右上左下，转换成左下右上
                    self.road_dict["goalPlaceInfo"] = [[targetPos[0][0],targetPos[1][1]],[targetPos[1][0],targetPos[0][1]]]
                    print(self.road_dict["goalPlaceInfo"])
                # else:
                #     if "waypoints" in self.aimPathJsonInforDict:
                #         waypoints = self.aimPathJsonInforDict["waypoints"]
                        # b = len(waypoints)
                        # c = 0
                        # targetPoint = []
                        # for value in waypoints.values():
                        #     c += 1
                        #     if c == b:
                        #         targetPoint = value
                        # self.road_dict["goalPlaceInfo"] = [[targetPoint[0]-5, targetPoint[1]-5],
                        #                                    [targetPoint[0]+5, targetPoint[1]+5]]
            except:
                print("不存在结束位置信息")
                self.road_dict["goalPlaceInfo"] = None

    def mapDataProcess(self, map_file_path):
        # opendriving解析数据
        opendrive, all_road_dict, road_list, all_road_mark, all_junction_info, every_lane_info, header_info = map_data_process.main(map_file_path)

        # 获取停止线数据
        # all_stop_line = get_stop_line(all_road_dict, road_list)
        # crosswalk_points = get_crosswalk_point(road_list)
        # all_stop_line = cal_stop_line(all_stop_line, crosswalk_points)
        # all_stop_line = []
        # 这里需要把交叉口关系也传进来，这样可以对停止线进行一个集合处理，形成停止线字典{交叉口id：[停止线列表]}
        all_stop_line = map_data_process.get_stop_line_withStopLaneInfo(road_list, every_lane_info, all_junction_info)

        # 获取车道规则
        all_lane_access = map_data_process.get_lane_access(road_list)

        # 离线数据的生成及存储
        # road_dict = {}
        # all_road_dict数据结构
        # 道路id：'road_id'
        # 车道id：'lane_id'
        # 车道类型：'type'
        # 车道中心点数据：'center_vertices'
        # 车道左侧点数据：'left_vertices'
        # 车道右侧点数据：'right_vertices'
        # 车道角度：'lane_angle'
        self.road_dict['all_road_dict'] = every_lane_info
        # all_stop_line数据结构
        # 列表，对应的车道的停止线，现在这个是一个dict了，每一个交叉口对应一个key的id，值是一个列表就是停止线的列表
        self.road_dict['all_stop_line'] = all_stop_line
        # all_lane_access数据结构
        # 道路id：'road_id'
        # 车道id：'lane_id'
        # 车道限制：None：没有限制，allow：有限制
        self.road_dict['all_lane_access'] = all_lane_access
        # all_road_mark数据结构
        # 道路id：'road_id'
        # 道路标识：’road_mark_list‘：车道id：'lane_id', 道路标识纵向偏移：‘road_mark_sOffset’, 车道标识类别：‘road_mark_type’
        self.road_dict['all_road_mark'] = all_road_mark
        # all_junction_info数据结构
        # 交叉口id: 'junction_id'
        # 连接段id: 'connection_id', 交叉路口中用于连接的路段id: 'connectingRoad_id', 来向路段id: 'incomingRoad_id'
        self.road_dict['all_junction_info'] = all_junction_info
        # 表头信息，包括名字等数据
        self.road_dict['header_info'] = header_info

        # wth:all_road_info_file_path写在场景同级目录下
        # all_road_info_file = 'testData/1.json'
        directory = os.path.dirname(os.path.abspath(map_file_path))
        all_road_info_file_path = os.path.join(directory, 'all_road_info_file.json')
        Parameter.all_road_info_file = all_road_info_file_path
        road_list_json = json.dumps(self.road_dict)
