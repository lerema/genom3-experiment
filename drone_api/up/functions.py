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

"""Functions to be called for estimating the drone and environment state."""
from drone_api.up.user_types import Area, Location


def get_battery_level(components):
    """Return the battery level of the drone."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"]


def get_robot_pose(components):
    """Return the robot pose."""

    pom = components["maneuver"].component
    data = pom.get_reference()

    return data["reference"]["pos"]


def monitor_battery(components):
    """Return True if the battery is low."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"] > 1


def plate_found(components):
    """Return True if the plate is found."""

    try:
        _ = components["CT_drone"].component.TargetPose()
        return True
    except Exception:
        return False


def detect_aruco_marker(components, marker):
    """Return True if the aruco marker is found."""

    data = components["arucotag"].pose(str(marker))

    if (
        data["pos"]["x"] is not None
        and data["pos"]["y"] is not None
        and data["pos"]["z"] is not None
    ):
        return True
    else:
        return False


def get_aruco_pose(components, marker):
    """Return the aruco marker pose."""

    data = components["arucotag"].pose(str(marker))
    return data["pos"]


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
        pass

    def is_location_surveyed(self, area: Area, location: Location, **kwargs) -> bool:
        # TODO: Implement this function with actual validataion
        pass

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
