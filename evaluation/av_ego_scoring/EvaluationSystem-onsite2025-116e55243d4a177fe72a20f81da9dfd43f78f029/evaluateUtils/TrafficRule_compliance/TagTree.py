from typing import List
from collections import deque


DEF_DICT = {}

class MetaTag:
    """
    Child Pointer
    """

    def __init__(self, name: str):
        self.name = name
        self.children = []
        self.keys = []
        self.definition = DEF_DICT[name] if name in DEF_DICT.keys() else name  # {key:definition}
        self.status = -1  # -1: not visited, 0: visited but NOT selected, 1: visited and selected
        self.code = None  # code of the tag

    def __repr__(self):
        return f"MetaTag(name={self.name},children=[{','.join([c.name for c in self.children]) if self.children else None}],status={self.status},code={self.code})"

    def _add_child(self, child, key: str):
        """
        Add an attribute(child node) to the given tag.
        """
        self.children.append(child)
        self.keys.append(key)

    def _add_code(self, code: str):
        """
        Add code to the given tag.
        """
        self.code = code

    def _add_parent(self, parent):
        """
        Add parent to the given tag.
        """
        self.parent = parent



class TagTree:
    def __init__(self):
        self.root = MetaTag(name='root')
        self.build_tree()

    def add_leaves(self, node: MetaTag, leaves_names: List[str], key: str):
        """
        Add leaves to a given node.
        """
        for leaf_name in leaves_names:
            leaf = MetaTag(name=leaf_name)
            node._add_child(child=leaf, key=key)
        return node

    def build_RoadTree(self):
        """
        Manually build the road tree.
        "道路(道路类型)": {
            "公路(公路技术等级)":["高速公路","普通公路"],
            "公路(公路地理位置)":["山区公路","非山区公路"],
            "城市道路(城市道路功能等级)":["城市快速路","普通城市道路","小区内部道路"]
        },
        "道路(设施类型)":{
            "路段":[],"匝道":[],"交叉口(交叉口类型)":["普通交叉口","环形交叉口"],
            "桥梁":[],"隧道":[],"铁路道口":[],"停车场":[],"出租车上下站":[],"道路停车泊位":[],"停靠站":[]
        },
        "道路(车道类型)": {
            "机动车道(车道设计速度)":["快车道","慢车道","加速车道","减速车道"],
            "人行道(人行道通行通畅性)":["通行通畅的人行道","有障碍物的人行道","无法通行的人行道"],
            "非机动车道(非机动车道通行通畅性)":["通行通畅的非机动车道","无法通行的非机动车道"],
            "人行横道(人行横道使用情况)":["有行人通行的人行横道","无行人通行的人行横道"],
            "没有划分车道类型":[]
        },

        "道路(车道专属权)":{
            "有车道专属权(专属车道类型)":["客车专用道","公交专用车道","应急车道"],
            "无车道专属权":""
        },
        "道路(车道宽度)":["宽度正常的车道","狭窄的车道"],
        "道路(平面线形)":["急转弯","正常线形"],
        "道路(纵断面特征)":{
            "陡坡(行驶方向)":["上陡坡","下陡坡"],
            "非陡坡":[]
        },
        "道路(单向车道数目)":["单车道","双车道","大于两条车道","大于三条车道"],
        "道路(前方车道数变化)":["前方车道数减少","前方车道数增加","前方车道数不变"],
        "道路(路面情况)":["漫水路","冰雪路面","泥泞路面","正常路面"],
        "道路(主辅路类型)":["主路","辅路"],
        "道路(道路通行方向)":["单向通行","双向通行"]
        "道路(周边设施情况)":["30米以内有公共汽车站","30米以内有急救站","30米以内有加油站","30米以内有消防栓","30米以内有消防队（站）",
                            "50米以内有交叉口","50米以内有铁路道口","50米以内有急弯路","50米以内有宽度不足4米的窄路","50米以内有桥梁","50米以内有陡坡","50米以内有隧道"],
        "道路(车道空间位置)":["左侧车道","中间车道","右侧车道"],
        "道路(道路安全性)":["容易发生危险的道路","安全性良好的道路"]
        """
        Road = MetaTag(name='道路')

        highway = MetaTag(name='公路')
        urbanroad = MetaTag(name='城市道路')
        Road._add_child(child=highway, key='道路类型')
        Road._add_child(child=urbanroad, key='道路类型')
        # 公路(公路技术等级)
        motorway = MetaTag(name='高速公路')
        normal_highway = MetaTag(name='普通公路')
        highway._add_child(child=motorway, key='公路技术等级')
        highway._add_child(child=normal_highway, key='公路技术等级')
        # 公路(公路地理位置)
        mountain_highway = MetaTag(name='山区公路')
        non_mountain_highway = MetaTag(name='非山区公路')
        highway._add_child(child=mountain_highway, key='公路地理位置')
        highway._add_child(child=non_mountain_highway, key='公路地理位置')
        # 城市道路(城市道路功能等级)
        expressway = MetaTag(name='城市快速路')
        normal_urbanroad = MetaTag(name='普通城市道路')
        inner_urbanroad = MetaTag(name='小区内部道路')
        urbanroad._add_child(child=expressway, key='城市道路功能等级')
        urbanroad._add_child(child=normal_urbanroad, key='城市道路功能等级')
        urbanroad._add_child(child=inner_urbanroad, key='城市道路功能等级')
        # 城市道路(城市道路设施类型)
        Road = self.add_leaves(Road, leaves_names=["路段", "匝道", "桥梁", "隧道", "铁路道口", "停车场", "出租车上下站",
                                                   "道路停车泊位", "停靠站"], key='设施类型')
        # 交叉口(交叉口类型)
        intersection = MetaTag(name="交叉口")
        intersection = self.add_leaves(intersection, leaves_names=["普通交叉口", "环形交叉口"], key='交叉口类型')
        Road._add_child(child=intersection, key='设施类型')

        # 道路(车道类型)
        vehicle_lane = MetaTag(name='机动车道')
        vehicle_lane = self.add_leaves(vehicle_lane, leaves_names=["快车道", "慢车道", "加速车道", "减速车道"],
                                       key='车道设计速度')
        pedestrian_lane = MetaTag(name='人行道')
        pedestrian_lane = self.add_leaves(pedestrian_lane,
                                          leaves_names=["通行通畅的人行道", "有障碍物的人行道", "无法通行的人行道"],
                                          key='人行道通行通畅性')
        non_vehicle_lane = MetaTag(name='非机动车道')
        non_vehicle_lane = self.add_leaves(non_vehicle_lane,
                                           leaves_names=["通行通畅的非机动车道", "无法通行的非机动车道"],
                                           key='非机动车道通行通畅性')
        crosswalk = MetaTag(name='人行横道')
        crosswalk = self.add_leaves(crosswalk, leaves_names=["有行人通行的人行横道", "无行人通行的人行横道"],
                                    key='人行横道使用情况')
        Road._add_child(child=vehicle_lane, key='车道类型')
        Road._add_child(child=pedestrian_lane, key='车道类型')
        Road._add_child(child=non_vehicle_lane, key='车道类型')
        Road._add_child(child=crosswalk, key='车道类型')
        Road._add_child(child=MetaTag(name='没有划分车道类型'), key='车道类型')
        # 道路(车道专属权)
        access = MetaTag(name='有车道专属权')
        access = self.add_leaves(access, leaves_names=["客车专用道", "公交专用车道", "应急车道"], key='专属车道类型')
        Road._add_child(child=access, key='车道专属权')
        Road = self.add_leaves(Road, leaves_names=["无车道专属权"], key='车道专属权')
        # 道路(车道宽度)
        Road = self.add_leaves(Road, leaves_names=["宽度正常的车道", "狭窄的车道"], key='车道宽度')
        # 道路(平面线形)
        Road = self.add_leaves(Road, leaves_names=["急转弯", "正常线形"], key='平面线形')
        # 道路(纵断面特征)
        steep_slope = MetaTag(name='陡坡')
        steep_slope = self.add_leaves(steep_slope, leaves_names=["上陡坡", "下陡坡"], key='行驶方向')
        Road._add_child(child=steep_slope, key='纵断面特征')
        Road = self.add_leaves(Road, leaves_names=["非陡坡"], key='纵断面特征')
        # 道路(单向车道数目)
        Road = self.add_leaves(Road, leaves_names=["单车道", "双车道", "大于两条车道", "大于三条车道"],
                               key='单向车道数目')
        # 道路(前方车道数变化)
        Road = self.add_leaves(Road, leaves_names=["前方车道数减少", "前方车道数增加", "前方车道数不变"],
                               key='前方车道数变化')
        # 道路(路面情况)
        Road = self.add_leaves(Road, leaves_names=["漫水路", "冰雪路面", "泥泞路面", "正常路面"], key='路面情况')
        # 道路(主辅路类型)
        Road = self.add_leaves(Road, leaves_names=["主路", "辅路"], key='主辅路类型')
        # 道路(道路通行方向)
        Road = self.add_leaves(Road, leaves_names=["单向通行", "双向通行"], key='道路通行方向')
        # 道路(周边设施情况)
        Road = self.add_leaves(Road, leaves_names=["30米以内有公共汽车站", "30米以内有急救站", "30米以内有加油站",
                                                   "30米以内有消防栓", "30米以内有消防队（站）",
                                                   "50米以内有交叉口", "50米以内有铁路道口", "50米以内有急弯路",
                                                   "50米以内有宽度不足4米的窄路", "50米以内有桥梁", "50米以内有陡坡",
                                                   "50米以内有隧道"],
                               key='周边设施情况')
        # 道路(车道空间位置)
        Road = self.add_leaves(Road, leaves_names=["左侧车道", "中间车道", "右侧车道"], key='车道空间位置')
        # 道路(道路安全性)
        Road = self.add_leaves(Road, leaves_names=["容易发生危险的道路", "安全性良好的道路"], key='道路安全性')

        self.RoadTreeRootNode = Road
        return Road

    def build_InfrastructureTree(self):
        """
        "基础设施(交通标线设置情况)":{
            "有交通标线(交通标线类型)":{
                "白色实线(标线位置)":["路中白色实线","停车位白色实线","车行道边缘白色实线","停止线"],
                "白色虚线":"",
                "黄色实线(标线位置)":["路中黄色实线","路侧黄色实线","停车位黄色实线"],
                "黄色虚线(标线位置)":["路中黄色虚线","路侧黄色虚线","交叉口黄色虚线"],
                "双白虚线":"",
                "双白实线":"",
                "白色虚实线(左虚右实)":"",
                "白色虚实线(左实右虚)":"",
                "双黄实线":"",
                "双黄虚线":"",
                "黄色虚实线(左虚右实)":"",
                "黄色虚实线(左实右虚)":"",
                "可变导向车道线":"",
                "网状线":"",
                "导流线":"",
                "中心圈":"",
                "黄色填充线":"",
                "人行横道线":"",
                "限速标线":"",
                "车行道分界线":"",
                "停车让行线":"",
                "减速让行线":"",
                "公交专用车道线":"",
                "小型车专用车道线":"",
                "大型车道标线":"",
                "多乘员车辆专用车道线":"",
                "接近障碍物标线":"",
                "铁路平交道口标线":"",
                "减速标线":"",
                "立面标记":"",
                "实体标记":"",
                "橙色虚(实)线":"",
                "蓝色虚(实)线":"",
                "导向箭头":""
            },
            "缺少交通标线(交通标线类型)":["无限速标线","无禁止停车标线","无禁止左转标线","无禁止掉头标线","无交通标线"]
            },
        "基础设施(交通信号灯控制情况)":{
            "有交通信号灯控制(交通信号灯颜色)":["红灯","绿灯","黄灯"],
            "有交通信号灯控制(交通信号灯形状)":["直行灯","左转灯","右转灯","掉头灯","圆形灯"],
            "无交通信号灯控制":[]
        },
        "基础设施(交通标志设置情况)":{
            "有交通标志(交通标志类型)":["禁止掉头","禁止左转","禁止停车","禁止鸣喇叭","限速标志","允许掉头标志","警示标志"],
            "缺少交通标志(交通标志类型)":["无禁止掉头标志","无禁止左转标志","无禁止鸣喇叭标志","无限速标志","无交通标志"]
        },
        "基础设施(道路中央隔离设施设置情况)":["道路中心线","中央分隔带","无道路中央隔离设施"],
        """
        Infr = MetaTag(name='基础设施')

        wMarker = MetaTag(name='有交通标线')

        white_solid_line = MetaTag(name='白色实线')
        white_solid_line = self.add_leaves(node=white_solid_line,
                                           leaves_names=["路中白色实线", "停车位白色实线", "车行道边缘白色实线",
                                                         "停止线"], key='标线位置')
        wMarker._add_child(child=white_solid_line, key='交通标线类型')

        yellow_solid_line = MetaTag(name='黄色实线')
        yellow_solid_line = self.add_leaves(node=yellow_solid_line,
                                            leaves_names=["路中黄色实线", "路侧黄色实线", "停车位黄色实线"],
                                            key='标线位置')
        wMarker._add_child(child=yellow_solid_line, key='交通标线类型')

        yellow_dashed_line = MetaTag(name='黄色虚线')
        yellow_dashed_line = self.add_leaves(node=yellow_dashed_line,
                                             leaves_names=["路中黄色虚线", "路侧黄色虚线", "交叉口黄色虚线"],
                                             key='标线位置')
        wMarker._add_child(child=yellow_dashed_line, key='交通标线类型')

        wMarker = self.add_leaves(node=wMarker, leaves_names=[
            "白色虚线", "双白虚线", "双白实线", "白色虚实线(左虚右实)", "白色虚实线(左实右虚)",
            "双黄实线", "双黄虚线", "黄色虚实线(左虚右实)", "黄色虚实线(左实右虚)", "可变导向车道线",
            "网状线", "导流线", "中心圈", "黄色填充线", "人行横道线",
            "限速标线", "车行道分界线", "停车让行线", "减速让行线", "公交专用车道线",
            "小型车专用车道线", "大型车道标线", "多乘员车辆专用车道线", "接近障碍物标线", "铁路平交道口标线",
            "减速标线", "立面标记", "实体标记", "橙色虚(实)线", "蓝色虚(实)线", "导向箭头"
        ],
                                  key='交通标线类型')

        woMarker = MetaTag(name='缺少交通标线')
        woMarker = self.add_leaves(node=woMarker,
                                   leaves_names=["无限速标线", "无禁止停车标线", "无禁止左转标线", "无禁止掉头标线",
                                                 "无交通标线"], key='交通标线类型')
        Infr._add_child(child=woMarker, key='交通标线设置情况')
        Infr._add_child(child=wMarker, key='交通标线设置情况')

        wSignal = MetaTag(name='有交通信号灯')
        wSignal = self.add_leaves(node=wSignal, leaves_names=["红灯", "绿灯", "黄灯"], key='交通信号灯颜色')
        wSignal = self.add_leaves(node=wSignal, leaves_names=["直行灯", "左转灯", "右转灯", "掉头灯", "圆形灯"],
                                  key='交通信号灯形状')
        woSignal = MetaTag(name='无交通信号灯')
        Infr._add_child(child=woSignal, key='交通信号灯控制情况')
        Infr._add_child(child=wSignal, key='交通信号灯控制情况')

        wSign = MetaTag(name='有交通标志')
        wSign = self.add_leaves(node=wSign, leaves_names=["禁止掉头", "禁止左转", "禁止停车", "禁止鸣喇叭", "限速标志",
                                                          "允许掉头标志", "警示标志"], key='交通标志类型')
        woSign = MetaTag(name='缺少交通标志')
        woSign = self.add_leaves(node=woSign,
                                 leaves_names=["无禁止掉头标志", "无禁止左转标志", "无禁止鸣喇叭标志", "无限速标志",
                                               "无交通标志"], key='交通标志类型')
        Infr._add_child(child=woSign, key='交通标志设置情况')
        Infr._add_child(child=wSign, key='交通标志设置情况')

        Infr = self.add_leaves(node=Infr, leaves_names=["道路中心线", "中央分隔带", "无道路中央隔离设施"],
                               key='道路中央隔离设施设置情况')

        self.InfrTreeRootNode = Infr
        return Infr

    def build_ManagementTree(self):
        """
        "交通管理(交通管理情况)":{
            "交通事件(交通事件类型)":["道路施工","交通事故","路面有障碍物","车辆故障"],
            "交通管制(交通管制类型)":["限制通行","禁止通行","交通疏导"],
            "交通警察指挥":[],
            "无交通警察指挥":[],
            "管理人员指挥":[]
        },
        """
        Man = MetaTag(name='交通管理')
        # 交通管理(类型)
        TrafficEvent = MetaTag(name='交通事件')
        TrafficControl = MetaTag(name='交通管制')
        TrafficPolice = MetaTag(name='交通警察指挥')
        woTrafficPolice = MetaTag(name='无交通警察指挥')
        Administrator = MetaTag(name='管理人员指挥')
        # 交通事件
        TrafficEvent = self.add_leaves(node=TrafficEvent,
                                       leaves_names=["道路施工", "交通事故", "路面有障碍物", "车辆故障"],
                                       key='交通事件类型')
        # 交通管制
        TrafficControl = self.add_leaves(node=TrafficControl, leaves_names=["限制通行", "禁止通行", "交通疏导"],
                                         key='交通管制类型')
        Man._add_child(child=TrafficEvent, key='交通管理情况')
        Man._add_child(child=TrafficControl, key='交通管理情况')
        Man._add_child(child=TrafficPolice, key='交通管理情况')
        Man._add_child(child=woTrafficPolice, key='交通管理情况')
        Man._add_child(child=Administrator, key='交通管理情况')
        self.ManTreeRootNode = Man
        return Man

    def build_ParticipantTree(self):
        """
        "交通参与者(交通参与者类型)":{
            "机动车(机动车车辆类型)":["小轿车","三轮汽车","三轮摩托车","拖拉机","摩托车","牵挂车","校车","轮式自行机械","铰链车"],
            "机动车(机动车车牌归属地)":["本省市车牌","外省市车牌"],
            "机动车(机动车车牌性质)":["正式车牌","临时车牌","教练车牌"],
            "机动车(机动车车牌悬挂情况)":["正常悬挂","未悬挂车牌","故意遮挡车牌","故意污损车牌","车牌不清晰"],
            "机动车(机动车营运性质)":{
                "货运(货物运输类别)":["危险品货运","非危险品货运","超限物品货运"],
                "客运(客运车辆类型)":["公共汽车","出租车"],
                "非营运":[]
            },
            "机动车(机动车车辆尺寸)":["小型车","中型车","大型车"],
            "机动车(机动车设计速度)":["设计最高时速小于六十公里","设计最高时速大于六十公里","低速"],
            "机动车(机动车车速)":["缓慢行驶","正常行驶","静止"],
            "机动车(机动车行为)":[{"转向(转向类型)":["左转","右转","掉头"]},"直行","超车","会车","倒车","使用灯光","跟驰","换道","行车","鸣喇叭","停车"],
            "机动车(机动车注册登记情况)":["已注册登记","未注册登记"],
            "机动车(机动车载人情况)":["载人","未载人"],
            "机动车(机动车人员类型)":{
                "机动车驾驶人(机动车驾驶人安全带佩戴情况)":["已佩戴安全带","未佩戴安全带"],
                "机动车驾驶人(机动车驾驶人状态)":["醉酒","疲劳","接打手持电话","浏览电子设备","药驾","患病"],
                "机动车驾驶人(机动车驾驶人驾驶证情况)":["持有驾驶证且非实习期","无驾驶证","实习期","学习驾驶期"],
                "机动车乘客(机动车乘客安全带佩戴情况)":["已佩戴安全带","未佩戴安全带"],
                "机动车乘客(机动车乘客年龄)":["未满四周岁","未成年","成年"]
            },
            "机动车(机动车车况)":["车况正常","车辆发生故障"],

            "非机动车(非机动车车辆类型)":{
                "残疾人轮椅车(残疾人证携带情况)":["携带本市残疾人证","未携带本市残疾人证"],
                "残疾人轮椅车(驾驶人下肢健康情况)":["下肢残疾","下肢健康"],
                "自行车":[],
                "电动自行车":[],
                "畜力车":[],
                "三轮车":[]
            },
            "非机动车(非机动车转向灯装备情况)":["设有转向灯","未设有转向灯"],
            "非机动车(非机动车注册登记情况)":["已注册登记","未注册登记"],
            "非机动车(非机动车行为)":["直行","左转","右转","停车"],
            "非机动车(非机动车驾驶人年龄)":["成年","未成年"],
            "非机动车(非机动车驾驶人状态)":["醉酒","疲劳","接打手持电话","浏览电子设备","药驾,"患病"],
            "非机动车(非机动车驾驶人安全头盔佩戴情况)":["已佩戴安全头盔","未佩戴安全头盔"],

            "特种车辆(特种车辆类型)":["警车","消防车","救护车","工程救险车","工程作业车","道路养护车","洒水车","清扫车"],
            "特种车辆(特种车辆执行任务情况)":["正在执行任务","未执行任务"],
            
            "行人(行人的滑行工具使用情况)":["未使用滑行工具","使用滑板车","使用平衡车","使用旱冰鞋","使用滑轮"]
        },

        """
        Ptc = MetaTag(name='交通参与者')
        # 交通参与者(类型)
        MotorVehicle = MetaTag(name='机动车')
        Ptc._add_child(child=MotorVehicle, key='交通参与者类型')
        MotorVehicle = self.add_leaves(node=MotorVehicle,
                                       leaves_names=["小轿车", "三轮汽车", "三轮摩托车", "拖拉机", "摩托车", "牵挂车",
                                                     "校车", "轮式自行机械", "铰链车"], key='机动车车辆类型')
        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["本省市车牌", "外省市车牌"],
                                       key='机动车车牌归属地')
        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["正式车牌", "临时车牌", "教练车牌"],
                                       key='机动车车牌性质')
        MotorVehicle = self.add_leaves(node=MotorVehicle,
                                       leaves_names=["正常悬挂", "未悬挂车牌", "故意遮挡车牌", "故意污损车牌",
                                                     "车牌不清晰"], key='机动车车牌悬挂情况')
        # 机动车(机动车营运性质)
        freight = MetaTag(name='货运')
        freight = self.add_leaves(freight, leaves_names=["运输危险品", "运输非危险品", "运输超限物品"],
                                  key='货物运输类别')
        MotorVehicle._add_child(child=freight, key='机动车营运性质')
        Passenger_transport = MetaTag(name='客运')
        Passenger_transport = self.add_leaves(Passenger_transport, leaves_names=["公共汽车", "出租车"],
                                              key='客运车辆类型')
        MotorVehicle._add_child(child=Passenger_transport, key='机动车营运性质')
        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["非营运"], key='机动车营运性质')

        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["小型车", "中型车", "大型车"],
                                       key='机动车车辆尺寸')
        MotorVehicle = self.add_leaves(node=MotorVehicle,
                                       leaves_names=["低速", "设计最高时速小于六十公里", "设计最高时速大于六十公里"],
                                       key='机动车设计速度')
        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["缓慢行驶", "正常行驶", "静止"],
                                       key='机动车车速')
        # 机动车(机动车行为)
        MotorVehicle = self.add_leaves(node=MotorVehicle,
                                       leaves_names=["直行", "超车", "会车", "倒车", "使用灯光", "跟驰", "换道", "行车",
                                                     "鸣喇叭", "停车"], key='机动车行为')
        turning = MetaTag(name='转向')
        turning = self.add_leaves(turning, leaves_names=["左转", "右转", "掉头"], key='转向类型')
        MotorVehicle._add_child(child=turning, key='机动车行为')

        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["已注册登记", "未注册登记"],
                                       key='机动车注册登记情况')
        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["载人", "未载人"], key='机动车载人情况')

        # 机动车(机动车人员类型)
        Driver = MetaTag(name='机动车驾驶人')
        MotorVehicle._add_child(child=Driver, key='机动车人员类型')
        Driver = self.add_leaves(node=Driver, leaves_names=["已佩戴安全带", "未佩戴安全带"],
                                 key='机动车驾驶人安全带佩戴情况')
        Driver = self.add_leaves(node=Driver,
                                 leaves_names=["醉酒", "疲劳", "接打手持电话", "浏览电子设备", "药驾", "患病"],
                                 key='机动车驾驶人状态')
        Driver = self.add_leaves(node=Driver, leaves_names=["持有驾驶证且非实习期", "无驾驶证", "实习期", "学习驾驶期"],
                                 key='机动车驾驶人驾驶证情况')
        Passenger = MetaTag(name='机动车乘客')
        MotorVehicle._add_child(child=Passenger, key='机动车人员类型')
        Passenger = self.add_leaves(node=Passenger, leaves_names=["已佩戴安全带", "未佩戴安全带"],
                                    key='机动车乘客安全带佩戴情况')
        Passenger = self.add_leaves(node=Passenger, leaves_names=["未满四周岁", "未成年", "成年"], key='机动车乘客年龄')

        MotorVehicle = self.add_leaves(node=MotorVehicle, leaves_names=["车况正常", "车辆发生故障"], key='机动车车况')
        # 非机动车(非机动车车辆类型)
        NonMotorVehicle = MetaTag(name='非机动车')
        Ptc._add_child(child=NonMotorVehicle, key='交通参与者类型')
        disabled = MetaTag(name='残疾人轮椅车')
        disabled = self.add_leaves(disabled, leaves_names=["携带本市残疾人证", "未携带本市残疾人证"],
                                   key='残疾人证携带情况')
        disabled = self.add_leaves(disabled, leaves_names=["下肢残疾", "下肢健康"], key='驾驶人下肢健康情况')
        NonMotorVehicle._add_child(child=disabled, key='非机动车车辆类型')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle,
                                          leaves_names=["自行车", "电动自行车", "畜力车", "三轮车"],
                                          key='非机动车车辆类型')

        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle, leaves_names=["设有转向灯", "未设有转向灯"],
                                          key='非机动车转向灯装备情况')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle, leaves_names=["已注册登记", "未注册登记"],
                                          key='非机动车注册登记情况')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle, leaves_names=["直行", "左转", "右转", "停车"],
                                          key='非机动车行为')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle, leaves_names=["成年", "未成年"],
                                          key='非机动车驾驶人年龄')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle,
                                          leaves_names=["醉酒", "疲劳", "接打手持电话", "浏览电子设备", "药驾", "患病"],
                                          key='非机动车驾驶人状态')
        NonMotorVehicle = self.add_leaves(node=NonMotorVehicle, leaves_names=["已佩戴安全头盔", "未佩戴安全头盔"],
                                          key='非机动车驾驶人安全头盔佩戴情况')

        # 特种车辆
        SpecialVehicle = MetaTag(name='特种车辆')
        SpecialVehicle = self.add_leaves(SpecialVehicle,
                                         leaves_names=["警车", "消防车", "救护车", "工程救险车", "工程作业车",
                                                       "道路养护车", "洒水车", "清扫车"], key='特种车辆类型')
        SpecialVehicle = self.add_leaves(SpecialVehicle, leaves_names=["正在执行任务", "未执行任务"],
                                         key='特种车辆执行任务情况')
        Ptc._add_child(child=SpecialVehicle, key='交通参与者类型')
        # 行人
        Pedestrian = MetaTag(name='行人')
        Pedestrian = self.add_leaves(Pedestrian,
                                     leaves_names=["未使用滑行工具", "使用滑板车", "使用平衡车", "使用旱冰鞋",
                                                   "使用滑轮"], key='行人滑行工具使用情况')
        Ptc._add_child(child=Pedestrian, key='交通参与者类型')

        self.PtcTreeRootNode = Ptc
        return Ptc

    def build_EnvironmentTree(self):
        """
        "环境(天气)": ["无极端天气","雾天","雨天","雪天","沙尘天","冰雹天"],
        "环境(时间)": ["白天","夜晚"],
        "环境(出行时段)":["早高峰","晚高峰","平峰","规定时段"],
        "环境(工作日情况)":["工作日","非工作日","双休日","全体公民放假节日"],
        "环境(能见度)":{
            "能见度正常":[],
            "低能见度(能见距离)":["能见度小于200米","能见度小于100米","能见度小于50米"]
        }
        """
        Env = MetaTag(name='环境')
        # 环境(天气)
        Env = self.add_leaves(node=Env, leaves_names=["无极端天气", "雾天", "雨天", "雪天", "沙尘天", "冰雹天"],
                              key='天气')
        # 环境(时间)
        Env = self.add_leaves(node=Env, leaves_names=["白天", "夜晚"], key='时间')
        # 环境(出行时段)
        Env = self.add_leaves(node=Env, leaves_names=["早高峰", "晚高峰", "平峰", "规定时段", ], key='出行时段')
        # 环境(工作日情况)
        Env = self.add_leaves(node=Env, leaves_names=["工作日", "非工作日", "双休日", "全体公民放假节日"],
                              key='工作日情况')
        # 环境(能见度)
        Env._add_child(child=MetaTag(name='能见度正常'), key='能见度')
        low_visibility = MetaTag(name='低能见度')
        low_visibility = self.add_leaves(low_visibility,
                                         leaves_names=["能见度小于200米", "能见度小于100米", "能见度小于50米"],
                                         key='能见距离')
        Env._add_child(child=low_visibility, key='能见度')

        self.EnvTreeRootNode = Env
        return Env

    @staticmethod
    def build_code(node: MetaTag, parent_code: str):
        node._add_code(code=parent_code + '/' + node.name)
        for child in node.children:
            TagTree.build_code(node=child, parent_code=node.code)

    def build_parent(self):
        # import warnings
        # warnings.simplefilter('once',FutureWarning)
        # warnings.warn("This function will be added to the initialization function", FutureWarning)
        self.root.parent = None
        queue = deque([self.root])
        while queue:
            current_node = queue.popleft()
            for child in current_node.children:
                child._add_parent(current_node)
                queue.append(child)

    def build_tree(self):
        Road = self.build_RoadTree()
        Infr = self.build_InfrastructureTree()
        Man = self.build_ManagementTree()
        Ptc = self.build_ParticipantTree()
        Env = self.build_EnvironmentTree()
        self.root._add_child(child=Road, key='Pegasus')
        self.root._add_child(child=Infr, key='Pegasus')
        self.root._add_child(child=Man, key='Pegasus')
        self.root._add_child(child=Ptc, key='Pegasus')
        self.root._add_child(child=Env, key='Pegasus')
        Action = MetaTag(name='通行规则')
        self.ActionNode = Action
        self.root._add_child(child=Action, key='Pegasus')

        TagTree.build_code(node=self.root, parent_code='')
        self.build_parent()

    @staticmethod
    def get_children_with_same_key(tag: MetaTag, key) -> List[MetaTag]:
        indices = [i for i, x in enumerate(tag.keys) if x == key]
        children_with_same_key = [tag.children[i] for i in indices]
        return children_with_same_key

    def recursive_print(self, tag: MetaTag, level=0):
        print('    ' * level + repr(tag))
        if tag.children:
            for k in list(set(tag.keys)):
                children_with_same_key = TagTree.get_children_with_same_key(tag=tag, key=k)
                for child in children_with_same_key:
                    self.recursive_print(child, level + 1)

    def print_tree(self):
        """
        Print a Child Pointer tree
        """
        self.recursive_print(self.root)

    def find_tag(self, target_name) -> List[MetaTag]:
        """
        traverse the tree and return the tag(s) with the given name
        """
        target_tag = []
        queue = deque([self.root])
        while queue:
            current_node = queue.popleft()
            if current_node.name == target_name:
                target_tag.append(current_node)
            for child in current_node.children:
                queue.append(child)
        return target_tag

    def find_tag_by_code(self, target_code) -> MetaTag:
        """
        traverse the tree and return the tag with the given code.
        """
        target_tag = None
        queue = deque([self.root])
        while queue:
            current_node = queue.popleft()
            if current_node.code == target_code:
                target_tag = current_node
                break
            for child in current_node.children:
                queue.append(child)
        if target_tag is None:
            raise ValueError(f"The given code {target_code} is not in the tree.")
            # print(f"The given code {target_code} is not in the tree.")
        return target_tag


