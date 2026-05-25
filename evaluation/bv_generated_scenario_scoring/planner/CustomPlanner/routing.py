import heapq
import warnings
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Point, Polygon
from planner.CustomPlanner.local_utils import calculate_distances
from utils.opendrive2discretenet.utils import encode_road_section_lane_width_type_id, decode_road_section_lane_width_type_id

# 变道所产生的惩戒项，进行一个细分，向左变道和向右变道所产生的惩戒
P_LANE_CHANGE_LEFT = 0.15
P_LANE_CHANGE_RIGHT = 0.1
# 连续变道产生的惩罚项
P_CONTINOUS_LANE_CHANGE = 1e5

# 定义车道节点类
class LaneNode:
    def __init__(self, lane_id, g, h, c, parent):
        self.lane_id = lane_id
        self.g = g
        self.c = c
        self.h = h
        self.parent = parent

    def __lt__(self, other):
        return self.g + self.h + self.c < other.g + other.h + other.c


# 定义车道拓扑图
class LaneGraph:
    def __init__(self, discrete_network, start_pos, end_pos, name=None):
        self.discrete_network = discrete_network
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.start_lane_id = None
        self.end_lane_id = None
        self.is_origin_start = True # 起点是否在车道上
        self.is_origin_end = True # 终点是否在车道上
        self.k_argmin_start = 1
        self.k_argmin_end = 1
        self.topo_graph = None
        self.graph_name = name

    def construct_discrete_graph(self, verbose=False):
        """
        将给定的离散车道数据，构建成拓扑图，用于搜索最短路径
        :param verbose: 是否打印debug信息
        :return:
        """

        graph = {}
        # 先将所有lane加入到graph中
        for lane in self.discrete_network.discretelanes:
            graph[lane.lane_id] = {"entity": lane, "neighbors": [], "center_length": 1e5}

        # 再遍历graph，找到每条lane的邻居，用于A*路径搜索，邻居包括:下游衔接lane，左右lane（如有）
        for lane in self.discrete_network.discretelanes:
            # 找到下游车道
            if len(lane.successor) != 0:
                for succ in lane.successor:
                    graph[lane.lane_id]["neighbors"].append(succ)
            # 找到左右车道
            decoded_lane_id = decode_road_section_lane_width_type_id(lane.lane_id)

            left_lane_id = list(decoded_lane_id)
            left_lane_id[2] += 1
            left_lane_id = encode_road_section_lane_width_type_id(*left_lane_id)
            right_lane_id = list(decoded_lane_id)
            right_lane_id[2] -= 1
            right_lane_id = encode_road_section_lane_width_type_id(*right_lane_id)

            if graph.__contains__(left_lane_id):
                graph[lane.lane_id]["neighbors"].append(left_lane_id)

            if graph.__contains__(right_lane_id):
                graph[lane.lane_id]["neighbors"].append(right_lane_id)

        self.topo_graph = graph

        if verbose:
            for key, value in graph.items():
                print(key, value["neighbors"])

    def cal_lanes_length(self):
        """
        计算所有车道的中心线长度
        :return:
        """
        for key, value in self.topo_graph.items():
            center_vertices = value["entity"].center_vertices
            center_length = np.sum(np.sqrt(np.sum(np.square(center_vertices[:-1] - center_vertices[1:]), axis=1)))
            self.topo_graph[key]["center_length"] = center_length

    def cal_lanes_width(self):
        """
        计算所有车道的中心线长度
        :return:
        """
        for key, value in self.topo_graph.items():
            left_vertices = value["entity"].left_vertices
            right_vertices = value["entity"].right_vertices
            mean_width = np.mean(np.sqrt(np.sum(np.square(left_vertices - right_vertices), axis=1)))
            self.topo_graph[key]["width"] = mean_width

    def convert_lanes_to_polygons(self):
        """
        将所有的车道变成多边形，用于确定起点和终点在哪条车道内
        :return:
        """
        for key, value in self.topo_graph.items():
            left_vertices = value["entity"].left_vertices
            right_vertices = value["entity"].right_vertices
            polygon = Polygon(np.r_[left_vertices, right_vertices[::-1]]) # 多边形的边的顺序要统一，顺时针或者逆时针，因此right_vertice的顺序要翻转才能构成四边形
            self.topo_graph[key]["polygon"] = polygon


    def get_start_end_lanes(self, verbose=False):
        """
        获取起点和终点所在的lane
        :param verbose: 是否绘制起点终点示意图
        :return:
        """
        start_point  = Point(self.start_pos)
        end_point  = Point(
            (self.end_pos[0][0] + self.end_pos[1][0]) / 2,
            (self.end_pos[0][1] + self.end_pos[1][1]) / 2)

        for key, value in self.topo_graph.items():
            polygon = value["polygon"]
            # 判断起点，终点是否在polygon里面，如果在，则这个polygon对应的车道就是起点车道or终点车道
            if polygon.contains(start_point):
                self.start_lane_id = key
            if polygon.contains(end_point):
                self.end_lane_id = key
        # 如果起点在车道线外，则需要找离起点最近的车道线的位置
        if self.start_lane_id == None:
            warnings.warn("Warning: the origin deviates from the the selected lane.")
            self.is_origin_start = False
            self.start_lane_id = self._get_argmin_distance_lanes(self.start_pos)

        # 如果终点在车道线外，同样需要找离终点最近的车道线的位置
        if self.end_lane_id == None:
            warnings.warn("Warning: the destination deviates from the the selected lane.")
            self.is_origin_end = False
            self.end_lane_id = self._get_argmin_distance_lanes(self.end_pos)

        print("Start lane is: ", self.start_lane_id, "end lane is: ", self.end_lane_id)
        # 绘制起点，终点和其他路径的地图，便于debug
        if verbose:
            fig, axs = plt.subplots()
            axs.set_aspect('equal', 'datalim')

            for key, value in self.topo_graph.items():
                if key == self.start_lane_id:
                    xs, ys = value["polygon"].exterior.xy
                    axs.fill(xs, ys, alpha=0.5, fc='r', ec='none')
                elif key == self.end_lane_id:
                    xs, ys = value["polygon"].exterior.xy
                    axs.fill(xs, ys, alpha=0.5, fc='b', ec='none')
                else:
                    xs, ys = value["polygon"].exterior.xy
                    axs.fill(xs, ys, alpha=0.5, fc='black', ec='none')

            plt.show()
    def rectify_start_lane_id(self):
        if self.k_argmin_start > len(self.topo_graph):
            raise Exception("Enumerate all lanes as origin lane, but still can not find any routes.")
        self.k_argmin_start += 1
        self.is_origin_start = False
        self.start_lane_id = self._get_k_argmin_distance_lanes(self.start_pos, self.k_argmin_start)
        warnings.warn("Try again %d / %d. The start lane has changed to %s" % (
            self.k_argmin_start, len(self.topo_graph), self.start_lane_id))

    def rectify_end_lane_id(self):
        if self.k_argmin_end > len(self.topo_graph):
            raise Exception("Enumerate all lanes as destination lane, but still can not find any routes.")
        self.k_argmin_end += 1
        self.is_origin_end = False
        self.end_lane_id = self._get_k_argmin_distance_lanes(self.end_pos, self.k_argmin_end)
        warnings.warn("Try again %d / %d. The end lane has changed to %s" % (
            self.k_argmin_end, len(self.topo_graph), self.end_lane_id))

    def reset_lane_id(self):
        self.get_start_end_lanes()

    def _get_argmin_distance_lanes(self, point):
        min_distance = 1e5
        min_lane_id = None
        for key, value in self.topo_graph.items():
            _, lane_min_dis = calculate_distances(point, value["entity"].center_vertices)
            if lane_min_dis < min_distance:
                min_distance = lane_min_dis
                min_lane_id = key

        return min_lane_id

    def _get_argsort_distance_lanes(self, point):
        min_distances = []
        min_lane_ids = []
        for key, value in self.topo_graph.items():
            _, lane_min_dis = calculate_distances(point, value["entity"].center_vertices)
            min_distances.append(lane_min_dis)
            min_lane_ids.append(key)

        argsort_ids = np.argsort(min_distances)
        min_lane_ids = np.array(min_lane_ids)[argsort_ids]
        return min_lane_ids

    def _get_k_argmin_distance_lanes(self, point, k):
        return self._get_argsort_distance_lanes(point)[k-1]


    def generate_reference_line(self, route):
        """
        根据routing信息生成参考线
        :return: 参考线点坐标
        """
        prev_lane_id = None
        prev_decode_lane_id = []
        referene_line = []
        for i, lane_id in enumerate(route):
            decoded_lane_id = decode_road_section_lane_width_type_id(lane_id)
            if i == 0:
                prev_lane_id = lane_id
                prev_decode_lane_id = decoded_lane_id
                referene_line.append(self.topo_graph[lane_id]["entity"].center_vertices)
                continue

            if decoded_lane_id[0] != prev_decode_lane_id[0] or decoded_lane_id[1] != prev_decode_lane_id[1]:  # 说明是直行
                referene_line.append(self.topo_graph[lane_id]["entity"].center_vertices)
            else:
                # TODO: 一个非常简单生成参考线的逻辑，之后可能还需要做修改
                if self.topo_graph[prev_lane_id]["center_length"] <= self.topo_graph[lane_id]["center_length"]:
                    referene_line = referene_line[:-1]
                    referene_line.append(self.topo_graph[lane_id]["entity"].center_vertices)
                else:
                    pass

            prev_lane_id = lane_id
            prev_decode_lane_id = decoded_lane_id


        res = []
        for each in referene_line:
            res.extend(each)

        return [each.tolist() for each in res]


    def plot_route(self, route_ids, start_pos, end_pos):
        """
        :param route_ids: 路径的ID集合
        绘制A*算法求解出来的可行路径
        :return:
        """
        fig, axs = plt.subplots()
        axs.set_aspect('equal', 'datalim')

        for key, value in self.topo_graph.items():
            if key == self.start_lane_id:
                xs, ys = value["polygon"].exterior.xy
                axs.fill(xs, ys, alpha=0.5, fc='r', ec='none')
            elif key == self.end_lane_id:
                xs, ys = value["polygon"].exterior.xy
                axs.fill(xs, ys, alpha=0.5, fc='b', ec='none')
            elif key in route_ids:
                xs, ys = value["polygon"].exterior.xy
                axs.fill(xs, ys, alpha=0.5, fc='g', ec='none')
            else:
                xs, ys = value["polygon"].exterior.xy
                axs.fill(xs, ys, alpha=0.5, fc='black', ec='none')

        axs.scatter(start_pos[0], start_pos[1], color='red', marker='o', s=15, zorder=3)
        axs.annotate('START', (start_pos[0] - 3, start_pos[1] + 2), fontsize=10)\

        format_goal = {
            'x': sorted([item[0] for item in end_pos]),
            'y': sorted([item[1] for item in end_pos])
        }
        axs.add_patch(
            patches.Rectangle(
                xy=(format_goal['x'][0], format_goal['y'][0]),
                width=format_goal['x'][1] - format_goal['x'][0],
                height=format_goal['y'][1] - format_goal['y'][0],
                angle=0,
                color="blue",
                fill=True,
                alpha=0.8,
                zorder=3
            ))

        axs.scatter(*list(zip(*self.generate_reference_line(route_ids))), s=5, color='black', marker='x', alpha=1)
        plt.savefig(self.graph_name + ".png")
        plt.close()
        # plt.show()

    def run(self):
        self.construct_discrete_graph()
        self.cal_lanes_length()
        self.cal_lanes_width()
        self.convert_lanes_to_polygons()
        self.get_start_end_lanes()


class AStarRoute(object):
    def __init__(self, lane_graph):
        self.lane_graph = lane_graph
        self.start_lane_id = self.lane_graph.start_lane_id
        self.end_lane_id = self.lane_graph.end_lane_id
        self.topo_graph = self.lane_graph.topo_graph

    @staticmethod
    def heuristic(lane_node, end_node):
        """
        定义启发式函数，即h(x)，当前车道中心线末尾距离目标车道中心线最近的欧氏距离
        :param lane_node: 当前车道节点
        :param end_node: 终点车道节点
        :return: 当前节点到终点的距离
        """
        return np.linalg.norm(lane_node["entity"].center_vertices[-1] - end_node["entity"].center_vertices[0])

    @staticmethod
    def gx(lane_node, next_node):
        lane_node_ids = decode_road_section_lane_width_type_id(lane_node["entity"].lane_id)
        next_road_ids = decode_road_section_lane_width_type_id(next_node["entity"].lane_id)
        lane_node_road_id = lane_node_ids[0]
        next_road_road_id = next_road_ids[0]
        lane_node_seg_id = lane_node_ids[1]
        next_road_seg_id = next_road_ids[1]
        if lane_node_road_id != next_road_road_id or lane_node_seg_id != next_road_seg_id: # 说明是直行
            return lane_node["center_length"]
        else:
            lane_node_lane_id = lane_node_ids[2]
            next_road_lane_id = next_road_ids[2]
            # 如果下一个车道id小于当前车道id，说明是右转，反之是左转
            if next_road_lane_id < lane_node_lane_id:
                # 这是时候的惩戒设计成两条路的平均长度 * 系数，默认右转为10%，左转为15%
                return (lane_node["center_length"] + next_node["center_length"]) / 2 * 1.1
            elif next_road_lane_id > lane_node_lane_id:
                # 这是时候的惩戒设计成两条路的平均长度 * 系数，默认右转为10%，左转为15%
                return (lane_node["center_length"] + next_node["center_length"]) / 2 * 1.15
            else:
                # 理论上这种情况应该不存在，如果有要查看代码
                raise Exception("改车道既不是左转也不是右转车道，请检查代码")

    @staticmethod
    def penalty(lane_node, next_node):
        """
        评价由于交通规则约束所产生的惩戒项
        :param lane_node: 当前车道节点
        :param next_node: 下一个待链接的车道起点
        :return:惩戒项
        """
        # TODO: 当前只有的惩罚项只涉及连续变道所产生的惩戒项，后续可能会增加新的惩戒项
        return AStarRoute._penalty_continous_lane_change(lane_node, next_node)

    @staticmethod
    def _penalty_continous_lane_change(cur_graph_node, next_node):
        """
        评价连续变道所产生的惩罚项
        :param lane_node: 当前车道节点
        :param next_node: 下一个待链接的车道起点
        :return: 惩戒项
        """
        if cur_graph_node is None or cur_graph_node.parent is None:
            return 0

        parent_graph_node = cur_graph_node.parent
        parent_road_seg_id = decode_road_section_lane_width_type_id(parent_graph_node.lane_id)[1]
        next_road_seg_id = decode_road_section_lane_width_type_id(next_node["entity"].lane_id)[1]
        if parent_road_seg_id == next_road_seg_id:
            return P_CONTINOUS_LANE_CHANGE

        return 0

    def calculate(self, force=False):
        """
        计算路径
        :param force: 当找不到可行路径时，是否强制所有临近的路径
        :return: cost和route
        """
        if self.topo_graph is None or len(self.topo_graph) == 0:
            warnings.warn("Empty topology graph, skip routing.")
            return [], -1
        if self.start_lane_id is None or self.end_lane_id is None:
            warnings.warn("Invalid start/end lane id (None), skip routing.")
            return [], -1
        if self.start_lane_id not in self.topo_graph or self.end_lane_id not in self.topo_graph:
            warnings.warn("Start/end lane id not in topology graph, skip routing.")
            return [], -1

        open_list = []
        closed_set = set()

        # 将起点加入open列表
        heapq.heappush(open_list, LaneNode(self.start_lane_id, 0, 0, 0, None))

        # 循环直到找到终点或open列表为空
        while open_list:
            # 从open列表中选择一个节点
            cur_lane_node = heapq.heappop(open_list)
            cur_lane_id = cur_lane_node.lane_id

            # 如果该节点是终点，则搜索结束，并输出路径
            if cur_lane_id == self.end_lane_id:
                path = []
                cost = cur_lane_node.g
                while cur_lane_node.parent:
                    path.append(cur_lane_id)
                    cur_lane_id = cur_lane_node.parent.lane_id
                    cur_lane_node = cur_lane_node.parent
                path.append(cur_lane_id)
                return path[::-1], cost

            # 将该节点从open list中删除，并将其加入closed set
            closed_set.add(cur_lane_id)
            for next_lane_id in self.topo_graph[cur_lane_id]["neighbors"]:
                # discrete network可能会解析出来一些不存在的lane id，如有直接跳过
                if next_lane_id not in self.topo_graph:
                    continue

                # 如果邻居节点已经在closed列表中，则跳过
                if next_lane_id in closed_set:
                    continue


                # 计算下一车道到终点车道的距离
                h = AStarRoute.heuristic(self.topo_graph[cur_lane_id], self.topo_graph[self.end_lane_id])
                # 计算邻居节点到起的距离
                g = AStarRoute.gx(self.topo_graph[cur_lane_id], self.topo_graph[next_lane_id]) + cur_lane_node.g
                # 计算当前车道到下一车道车道的惩罚项
                c = AStarRoute.penalty(cur_lane_node, self.topo_graph[next_lane_id]) + cur_lane_node.c

                #如果邻居车道不在open list中，则将其加入open list，并记录其父车道节点和到起点车道的距离
                if next_lane_id not in [node.lane_id for node in open_list]:
                    heapq.heappush(open_list, LaneNode(next_lane_id, g, h, c, cur_lane_node))

                # 如果邻居节点已经在open列表中，则比较其到起点的距离，如果新的距离更小，则更新其父节点和到起点的距离
                else:
                    for node in open_list:
                        if node.lane_id == next_lane_id:
                            if g < node.g:
                                node.g = g
                                node.parent = cur_lane_node

        # 如果open列表为空，则搜索失败
        warnings.warn("The algorithm did not find any valid routes. Please try again with different OD.")
        if force:
            return self.recalculate()
        else:
            return None, -1

    def recalculate(self):
        """
        当找不到可行路径时，更改起始和终点重新计算路径
        :return: cost和route
        """
        start_route, start_cost = self.recalculate_start()
        end_route, end_cost = self.recalculate_end()

        start_cost += calculate_distances(self.lane_graph.start_pos, self.topo_graph[start_route[0]]["entity"].center_vertices)[1]
        end_cost += calculate_distances(self.lane_graph.end_pos, self.topo_graph[end_route[-1]]["entity"].center_vertices)[1]

        if start_cost <= end_cost:
            self.start_lane_id = start_route[0]
            self.lane_graph.start_lane_id = start_route[0]
            return start_route, start_cost
        else:
            self.end_lane_id = end_route[-1]
            self.lane_graph.end_lane_id = end_route[-1]
            return end_route, end_cost

    def recalculate_start(self):
        """
        更换起始车道，重新计算路径计算路径
        :return: cost和route
        """
        cost = -1
        route = None
        lane_id = self.start_lane_id

        while cost == -1:
            self.lane_graph.rectify_start_lane_id()
            self.start_lane_id = self.lane_graph.start_lane_id
            route, cost = self.calculate()

        self.start_lane_id = lane_id
        self.lane_graph.start_lane_id = lane_id

        return route, cost

    def recalculate_end(self):
        """
        更换终止车道，重新计算路径计算路径
        :return: cost和route
        """
        cost = -1
        route = None
        lane_id = self.end_lane_id

        while cost == -1:
            self.lane_graph.rectify_end_lane_id()
            self.end_lane_id = self.lane_graph.end_lane_id
            route, cost = self.calculate()

        self.end_lane_id = lane_id
        self.lane_graph.end_lane_id = lane_id

        return route, cost


if __name__ == "__main__":
    from utils.opendrive2discretenet import parse_opendrive
    from utils.visualizer import Visualizer

    xodr_path = "../../scenario/replay/0110follow103/0110follow103.xodr"
    # xodr_path = "../../scenario/serial/maps/TJST/TJST.xodr"

    vis = Visualizer()
    vis.show_task(mode='REPLAY', task='0110follow103')

    road_info = parse_opendrive(xodr_path)
    start_node = vis.scene_info.task_info["startPos"]
    end_node = vis.scene_info.task_info["targetPos"]

    lane_graph = LaneGraph(road_info, start_node, end_node, '0110follow103')
    lane_graph.run()
    a = AStarRoute(lane_graph)
    route, cost = a.calculate(True)
    print(route)
    print(cost)
    lane_graph.generate_reference_line(route)
    lane_graph.plot_route(route, start_node, end_node)