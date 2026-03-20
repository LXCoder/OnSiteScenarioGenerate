from AutoPilot.Player.BasePlayer import BasePlayer
from AutoPilot.Player.VehicleState import VehicleState
from AutoPilot.Player.Observation import Observation


class TestPlayer(BasePlayer):

    def __init__(self, player_id: str = "test"):
        super().__init__(player_id)

    def act(self) -> VehicleState:
        return VehicleState(x=-653.0, y=-140.0, heading=0.0, speed=0.0)

    def predict(self, obs: Observation) -> None:
        pass
