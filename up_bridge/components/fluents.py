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

"""Fluent representation."""
from drone_api.functions import get_battery_level, get_robot_pose
from up_bridge.components.user_types import Area, Location
from typing import Any, Dict
from drone_api.utils import Singleton


class Facts(Singleton):
    is_surveyed: Dict[Any, bool] = {}
    is_location_surveyed: Dict[Any, bool] = {}


class Fluents:
    components = None

    def robot_at(self, location: Location, **kwargs) -> bool:
        assert isinstance(location, Location), f"{location} is not a Location"
        result = get_robot_pose(self.components)
        if not isinstance(location, dict):
            location = location.__dict__()

        def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
            return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

        for key in location:
            if key in ["x", "y", "z"] and not isclose(result[key], location[key]):
                return False

        return True

    def verify_station_at(self, location: Location, **kwargs) -> bool:
        assert isinstance(location, Location), f"{location} is not a Location"

        # TODO: Implement this function
        # Check captured photo

        return self.robot_at(location)

    def is_surveyed(self, area: Area, **kwargs) -> bool:
        # TODO: Implement this function with actual validataion
        assert isinstance(area, Area), f"{area} is not a Area"
        Facts.is_surveyed[area] = (
            kwargs["expected_value"] if kwargs["expected_value"] else False
        )
        return Facts.is_surveyed[area]

    def is_location_surveyed(self, area: Area, location: Location, **kwargs) -> bool:
        # TODO: Implement this function with actual validataion
        assert isinstance(area, Area), f"{area} is not a Area"
        assert isinstance(location, Location), f"{location} is not a Location"
        Facts.is_location_surveyed[location] = (
            kwargs["expected_value"] if kwargs["expected_value"] else False
        )
        return Facts.is_location_surveyed[location]

    def is_within_area(self, area: Area, location: Location, **kwargs) -> bool:
        assert isinstance(area, Area), f"{area} is not a Area"
        assert isinstance(location, Location), f"{location} is not a Location"
        if not isinstance(area, dict):
            area = area.__dict__()
        if not isinstance(location, dict):
            location = location.__dict__()

        result = (area["xmin"] <= location["x"] <= area["xmax"]) and (
            area["ymin"] <= location["y"] <= area["ymax"]
        )
        return bool(result)

    def battery_at(self, level: int, **kwargs) -> bool:
        assert isinstance(level, int), f"{level} is not an int"

        return bool(get_battery_level(self.components) == level)
