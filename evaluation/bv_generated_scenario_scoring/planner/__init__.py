# 对 HMAPLANNER 加一层容错代理：如果原始 HMAPLANNER 崩溃，则返回安全默认动作，避免整个流程中断。
try:
    from planner.HMAPlanner.hmaPlanner import HMAPLANNER as _HMAPLANNER_ORIG
except Exception:
    _HMAPLANNER_ORIG = None


class _SafePlannerProxy:
    def __init__(self, impl_cls, *args, **kwargs):
        self._enabled = impl_cls is not None
        self._impl = None
        if self._enabled:
            try:
                self._impl = impl_cls(*args, **kwargs)
            except Exception:
                self._enabled = False
                self._impl = None

    def init(self, scene):
        if not self._enabled or self._impl is None:
            return
        try:
            return self._impl.init(scene)
        except Exception:
            self._enabled = False
            return

    def act(self, observation):
        if not self._enabled or self._impl is None:
            return [0.0, 0.0]
        try:
            return self._impl.act(observation)
        except Exception:
            self._enabled = False
            return [0.0, 0.0]

class HMAPLANNER:
    def __init__(self, *args, **kwargs):
        self._proxy = _SafePlannerProxy(_HMAPLANNER_ORIG, *args, **kwargs)

    def init(self, scene):
        return self._proxy.init(scene)

    def act(self, observation):
        return self._proxy.act(observation)

try:
    from planner.CBDES.cbdes import CBDES  # optional (depends on cvxpy)
except Exception:
    CBDES = None

try:
    from planner.CustomPlanner_rank3.main_planner import PublicRoadPlanner as _PublicRoadPlanner_ORIG
except Exception:
    _PublicRoadPlanner_ORIG = None

try:
    from planner.IDM.idm import IDM as _IDM_ORIG
except Exception:
    _IDM_ORIG = None
# from planner.CVaR.cvar_plannner import CVaR
try:
    from planner.CustomPlanner.main_planner import PublicRoadPlanner as _WanJiPlanner_ORIG
except Exception:
    _WanJiPlanner_ORIG = None


class PublicRoadPlanner:
    def __init__(self, *args, **kwargs):
        self._proxy = _SafePlannerProxy(_PublicRoadPlanner_ORIG, *args, **kwargs)

    def init(self, scene):
        return self._proxy.init(scene)

    def act(self, observation):
        return self._proxy.act(observation)


class IDM:
    def __init__(self, *args, **kwargs):
        self._proxy = _SafePlannerProxy(_IDM_ORIG, *args, **kwargs)

    def init(self, scene):
        return self._proxy.init(scene)

    def act(self, observation):
        return self._proxy.act(observation)


class WanJiPlanner:
    def __init__(self, *args, **kwargs):
        self._proxy = _SafePlannerProxy(_WanJiPlanner_ORIG, *args, **kwargs)

    def init(self, scene):
        return self._proxy.init(scene)

    def act(self, observation):
        return self._proxy.act(observation)

# PLANNERS = [HMAPLANNER, CBDES, PublicRoadPlanner, IDM, CVaR]
# PLANNERS = [HMAPLANNER, CBDES, PublicRoadPlanner, IDM]
PLANNERS = [HMAPLANNER, PublicRoadPlanner, IDM, WanJiPlanner]
