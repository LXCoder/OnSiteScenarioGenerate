from abc import ABC, abstractmethod

from AutoPilot.Player.VehicleState import VehicleState
from AutoPilot.Player.Observation import Observation


class BasePlayer(ABC):

    def __init__(self, player_id: str):
        self.player_id = player_id

    @abstractmethod
    def act(self) -> VehicleState:
        """输出当前帧的车辆状态"""
        ...

    @abstractmethod
    def predict(self, obs: Observation) -> None:
        """接收观测信息，更新内部决策"""
        ...
