from dataclasses import dataclass
from typing import List
class Body:
    name: str 
    IsInQuadtree: bool
    position: List[float]
    velocity: List[float]
    mass: float
    radius: float
    