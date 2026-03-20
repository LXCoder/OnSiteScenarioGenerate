from dataclasses import dataclass, field
from typing import List


@dataclass
class Observation:
    lateral_offset: float = 0.0
    angle_diff: float = 0.0
    speed: float = 0.0
    nearby_vehicles: List = field(default_factory=list)
