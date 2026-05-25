import sys

from evaluateUtils.standard_parameter import Parameter


class sceneAndScoreWeightProcess:

    # 记住weightData是处理过后，聚合好的一个字典，是我自己处理的
    def __init__(self, weightData, sceneTypeList):
        self.indexWeight = weightData["indexWeightDict"]
        self.fullScore = weightData["fullScoreDict"]
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
        self.fullScoreProcess()

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

            # 避免 aimType 不在 self.sceneWeightDict 导致错误
            if aimType not in self.sceneWeight:
                aimType = Parameter.defaultAimType
                # 这里传进来的名称肯定不在列表里面，所以这里的权重对应的名称会全部都变成默认的onsite
                # 他们在下面的权重都会是1，所有场景的权重都是一致的

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
        self.scoreWeight = {'safe': self.indexWeight["10000"],
                            'comfortable': self.indexWeight["20000"],
                            'efficiency': self.indexWeight["30000"],
                            'coordination': self.indexWeight['40000'],
                            'compliance': self.indexWeight['50000']}

    def fullScoreProcess(self):
        self.fullScore = {'safe': self.fullScore["1"],
                          'comfortable': self.fullScore["2"],
                          'efficiency': self.fullScore["3"],
                          'coordination': self.fullScore['4'],
                          'compliance': self.fullScore['5']}

    def indexWeightProcess(self):
        # 补充没有给到的数据
        for i in range(1, 6):
            for j in range(0, 15):
                a = i * 10000 + j
                self.fixData(str(a))

        # 这里还是按照三个维度来做，其中交通协调性归属到效率里面，交规符合性归属到安全里面
        self.safeDitalWeight = {"crash": self.indexWeight["10001"],
                                "TTC": self.indexWeight["10002"],
                                "inSubtendRoad": self.indexWeight["10003"],
                                "onLaneMarking": self.indexWeight["10004"],
                                "overSpeed": self.indexWeight["10005"],
                                "outOfLane": self.indexWeight["10006"],
                                "transverseDistance": self.indexWeight["10007"],
                                "inForbiddenArea": self.indexWeight["10008"],
                                "breakSignal": self.indexWeight["10009"],
                                "drivingInDesignatedLane": self.indexWeight["10010"],
                                "stopAtStopLine": self.indexWeight["10011"],
                                "isSecurityInvolved": self.indexWeight["10012"],
                                "onDottedLaneMarking": self.indexWeight["10013"],
                                "followStopSignal": self.indexWeight["10014"],}
        self.comfortableDitalWeight = {"overLateralAcce": self.indexWeight["20001"],
                                       "overLateralJerk": self.indexWeight["20002"],
                                       "overAcce": self.indexWeight["20003"],
                                       "overJerk": self.indexWeight["20004"],
                                       "unstableSteeringAngle": self.indexWeight["20005"],}
        self.efficiencyDitalWeight = {"usedTime": self.indexWeight["30001"],
                                      "averageSpeed": self.indexWeight["30002"],
                                      "gapRefuse": self.indexWeight["30003"],
                                      "missionAccomplish": self.indexWeight["30004"],
                                      "reverseCar": self.indexWeight["30005"],
                                      "coordination": self.indexWeight["40001"]}

    def fixData(self, key):
        if key not in self.indexWeight:
            self.indexWeight[key] = 0

