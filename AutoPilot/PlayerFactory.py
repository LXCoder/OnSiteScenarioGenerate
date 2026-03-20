import json
import importlib.util
from typing import Dict, Any

from AutoPilot.Player.BasePlayer import BasePlayer


class PlayerFactory:

    def __init__(self, config_path: str):
        with open(config_path, "r", encoding="utf-8") as f:
            self.config: Dict[str, Any] = json.load(f)

    def create(self, team_name: str) -> BasePlayer:
        if team_name not in self.config:
            raise KeyError(f"配置中不存在选手 '{team_name}'")

        entry = self.config[team_name]
        file_path = entry["file"]
        class_name = entry["class"]
        player_id = entry.get("player_id", team_name)

        spec = importlib.util.spec_from_file_location(team_name, file_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        cls = getattr(module, class_name)

        if not issubclass(cls, BasePlayer):
            raise TypeError(f"'{class_name}' in {file_path} 没有继承 BasePlayer")

        return cls(player_id=player_id)

    def create_all(self) -> Dict[str, BasePlayer]:
        return {name: self.create(name) for name in self.config}
