"""Callback functions for drone api  components"""
from dataclasses import dataclass, field
from typing import Iterator


class APIUserType:
    """Location of the object understandable with the robot."""

    def __init__(self, name: str, parameters: dict) -> None:
        self.name = name
        self.parameters = parameters

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.name})"

    def __dict__(self) -> dict:
        return self.parameters

    def __eq__(self, other: object) -> bool:
        return isinstance(other, self.__class__) and self.parameters == other.parameters

    def __hash__(self) -> int:
        return hash(self.name)

    def __str__(self) -> str:
        return self.name

    def keys(self) -> Iterator[str]:
        return self.parameters.keys()

    def values(self) -> Iterator[str]:
        return self.parameters.values()

    def __getitem__(self, key: str) -> str:
        return self.parameters[key]


@dataclass
class Location(APIUserType):
    """Location of the object understandable with the robot."""

    name: str = field(default="", compare=False)
    x: float = field(default=0.0, compare=True)
    y: float = field(default=0.0, compare=True)
    z: float = field(default=0.0, compare=True)
    yaw: float = field(default=0.0, compare=True)

    parameters = {"x": x, "y": y, "z": z, "yaw": yaw}


@dataclass
class Area(APIUserType):
    """Area of the object understandable with the robot."""

    name: str = field(default="", compare=False)
    xmin: float = field(default=0.0, compare=True)
    xmax: float = field(default=0.0, compare=True)
    ymin: float = field(default=0.0, compare=True)
    ymax: float = field(default=0.0, compare=True)
    z: float = field(default=0.0, compare=True)
    yaw: float = field(default=0.0, compare=True)

    parameters = {
        "xmin": xmin,
        "xmax": xmax,
        "ymin": ymin,
        "ymax": ymax,
        "z": z,
        "yaw": yaw,
    }
