import os
from typing import Dict, Optional

from AutoPilot.Player.BasePlayer import BasePlayer
from AutoPilot.Player.VehicleState import VehicleState
from AutoPilot.Player.Observation import Observation
from AutoPilot.PlayerFactory import PlayerFactory

def _find_config():
    search_dir = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        candidate = os.path.join(search_dir, "players_config.json")
        if os.path.isfile(candidate):
            return candidate
        search_dir = os.path.dirname(search_dir)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "players_config.json")

_DEFAULT_CONFIG = _find_config()


class PlayerManager:

    def __init__(self, config_path: str = _DEFAULT_CONFIG):
        self._factory = PlayerFactory(config_path)
        self._players: Dict[str, BasePlayer] = {}

    def load_all(self):
        self._players = self._factory.create_all()
        print(f"[PlayerManager] 已加载 {len(self._players)} 个选手:")
        for name, player in self._players.items():
            print(f"  {name} -> {player.__class__.__name__}")

    def step_all(
        self,
        observations: Optional[Dict[str, Observation]] = None,
    ) -> Dict[str, VehicleState]:
        results: Dict[str, VehicleState] = {}
        for name, player in self._players.items():
            if observations and name in observations:
                player.predict(observations[name])
            results[name] = player.act()
        return results

    def get_player(self, team_name: str) -> BasePlayer:
        return self._players[team_name]

    @property
    def player_names(self):
        return list(self._players.keys())