class SelfDrivingCarInfo:
    _instance = None

    def __init__(self):
        # 起点坐标
        self.startPos = 0
        # # 获取到起点坐标后上锁，并缓存
        # self.startPosGetLock = 0

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = object.__new__(cls)
        return cls._instance


class ScenarioLock(object):
    _instance = None

    def __init__(self):
        # 场景1时间锁
        self.scenario1StartTimeLock = 0
        self.scenario1EndTimeLock = 0
        # 场景2时间锁
        self.scenario2StartTimeLock = 0
        self.scenario2EndTimeLock = 0
        # 场景3时间锁
        self.scenario3StartTimeLock = 0
        self.scenario3EndTimeLock = 0
        # 场景4时间锁
        self.scenario4StartTimeLock = 0
        self.scenario4EndTimeLock = 0
        # 场景5时间锁
        self.scenario5StartTimeLock = 0
        self.scenario5EndTimeLock = 0
        # 场景6时间锁
        self.scenario6StartTimeLock = 0
        self.scenario6EndTimeLock = 0
        """
        0515设想，场景锁只要一对?因为只会同时开始一个场景
        同时场景开始时间与场景结束时间也同时只有一对
        """

        self.scenarioStartTimeLock = 0
        self.scenarioEndTimeLock = 0

        self.scenarioStartTime = 0
        self.scenarioEndTime = 0

class Scenario(object):
    def __init__(self):
        self.scenarioId = 0

        self.lastTime = 0

        self.Scenario1Start = 0
        self.Scenario1End = 0

        self.Scenario2Start = 0
        self.Scenario2End = 0

        self.Scenario3Start = 0
        self.Scenario3End = 0
        self.Scenario3startTime = 0
        self.Scenario3endTime = 0

        self.Scenario4Start = 0
        self.Scenario4End = 0
        self.Scenario4startTime = 0
        self.Scenario4endTime = 0

        self.Scenario5Start = 0
        self.Scenario5End = 0
        self.Scenario5startTime = 0
        self.Scenario5endTime = 0

        self.Scenario6Start = 0
        self.Scenario6End = 0
        self.Scenario6startTime = 0
        self.Scenario6endTime = 0

        self.RouteConn1 = 0
        self.RouteConn3 = 0
        self.RouteConn6 = 0

        self.Scenario3Section = 0
        self.Scenario4Section = 0
        self.Scenario5Section = 0
        self.Scenario6Section = 0

        self.Scenario3EndSection = 0
        self.Scenario4EndSection = 0
        self.Scenario5EndSection = 0
        self.Scenario6EndSection = 0

        # 处于场景内
        self.InScenario = 0

    def send(self):
        if self.lastTime != 0:
            pass
