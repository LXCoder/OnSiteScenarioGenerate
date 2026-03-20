from dataclasses import dataclass


@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    length: float = 476.0
    width: float = 190.0
    height: float = 150.0
