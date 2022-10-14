# Copyright 2022 Selvakumar H S, LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Callback functions for drone api  components"""
from typing import Iterator


class APIUserType:
    """Location of the object understandable with the robot."""

    def __init__(self, name: str, **kwargs: dict) -> None:
        self.name = name
        self.parameters = kwargs

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


class Location(APIUserType):
    """Location of the object understandable with the robot."""

    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    parameters = {"x": x, "y": y, "z": z, "yaw": yaw}


class Area(APIUserType):
    """Area of the object understandable with the robot."""

    name: str = ""
    xmin: float = 0.0
    xmax: float = 0.0
    ymin: float = 0.0
    ymax: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    parameters = {
        "xmin": xmin,
        "xmax": xmax,
        "ymin": ymin,
        "ymax": ymax,
        "z": z,
        "yaw": yaw,
    }
