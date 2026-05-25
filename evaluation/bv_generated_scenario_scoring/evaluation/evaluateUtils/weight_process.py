import sys

class sceneAndScoreWeightProcess:

    # 记住weightData是处理过后，聚合好的一个字典，是我自己处理的
    def __init__(self, weightData, sceneTypeList):
        self.indexWeight = weightData["indexWeightDict"]
        self.scoreWeight = {}
        self.safeDitalWeight = {}
        self.comfortableDitalWeight = {}
        self.efficiencyDitalWeight = {}
        # 场景的权重列表
        self.sceneWeightDict = weightData["sceneWeightDict"]
        # 场景名称列表
        self.sceneNameDict = weightData["sceneNameDict"]
        # 当前场景的名称列表,这个是万集给的，每个场景有个对应的名称
        self.sceneTypeList = sceneTypeList
        # 记录处理好的场景间权重
        self.sceneWeight = {}

    def weightProcess(self):
        self.sceneWeightProcess()
        self.indexWeightProcess()
        self.scoreWeightProcess()

    def sceneWeightProcess(self):
        # 找场景的权重
        sceneID = 0
        self.sceneWeight = {}
        totalWeight = 0

        # 读场景标签的名称，用这个名称和警警数据库二级小标题做对比
        for name in self.sceneTypeList:
            sceneID += 1
            aimType = None
            # 如果是普通场景就是这里这样写，如果是self.sceneNameDict == none，就是证明场景都是自定义的，完全数据对应
            if self.sceneNameDict:
                for key in self.sceneNameDict:
                    if name in self.sceneNameDict[key]:
                        # 记录下来二级小标题对应的一级大标题
                        aimType = key
            else:
                # name就是不同的场景类型名称，万集自定义的名称，自定义的名称，一定会和self.sceneWeightDict中key的自定义名称是一样的
                aimType = name
            # 找到一级大标题对应的权重比
            # 这里是判断等于0的情况，如果一个场景权重是0.这里要额外处理一下，否则回头后面会报错
            if self.sceneWeightDict[aimType]:
                self.sceneWeight[sceneID] = self.sceneWeightDict[aimType]
                totalWeight += self.sceneWeightDict[aimType]
            else:
                self.sceneWeight[sceneID] = sys.float_info.epsilon
                totalWeight += sys.float_info.epsilon

        # 更新权重
        for key in self.sceneWeight:
            self.sceneWeight[key] = self.sceneWeight[key]/totalWeight

    def scoreWeightProcess(self):
        self.scoreWeight = {'safe': self.indexWeight["10000"], 'comfortable': self.indexWeight["20000"], 'efficiency': self.indexWeight["30000"]}

    def indexWeightProcess(self):
        # 补充没有给到的数据
        for i in range(1, 4):
            for j in range(0, 15):
                a = i * 10000 + j
                self.fixData(str(a))

        self.safeDitalWeight = {"crash": self.indexWeight["10001"]*100,
                                "TTC": self.indexWeight["10002"]*100,
                                "inSubtendRoad": self.indexWeight["10003"]*100,
                                "onLaneMarking": self.indexWeight["10004"]*100,
                                "overSpeed": self.indexWeight["10005"]*100,
                                "outOfLane": self.indexWeight["10006"]*100,
                                "transverseDistance": self.indexWeight["10007"]*100,
                                "inForbiddenArea": self.indexWeight["10008"]*100,
                                "breakSignal": self.indexWeight["10009"]*100,
                                "drivingInDesignatedLane": self.indexWeight["10010"]*100,
                                "stopAtStopLine": self.indexWeight["10011"]*100,}
        self.comfortableDitalWeight = {"overLateralAcce": self.indexWeight["20001"]*100,
                                       "overLateralJerk": self.indexWeight["20002"]*100,
                                       "overAcce": self.indexWeight["20003"]*100,
                                       "overJerk": self.indexWeight["20004"]*100,
                                       "unstableSteeringAngle": self.indexWeight["20005"]*100,}
        self.efficiencyDitalWeight = {"usedTime": self.indexWeight["30001"]*100,
                                      "averageSpeed": self.indexWeight["30002"]*100,
                                      "gapRefuse": self.indexWeight["30003"]*100,
                                      "missionAccomplish": self.indexWeight["30004"]*100,}

    def fixData(self, key):
        if key not in self.indexWeight:
            self.indexWeight[key] = 0

