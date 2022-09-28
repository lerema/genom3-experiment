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

        return bool(result == dict(location))

    def verify_station_at(self, location: Location, **kwargs) -> bool:
        assert isinstance(location, Location), f"{location} is not a Location"

        # TODO: Implement this function
        # Check captured photo

        return self.robot_at(location)

    def is_surveyed(self, area: Area, **kwargs) -> bool:
        assert isinstance(area, Area), f"{area} is not a Area"
        Facts.is_surveyed[area] = (
            kwargs["expected_value"] if kwargs["expected_value"] else False
        )
        return Facts.is_surveyed[area]

    def is_location_surveyed(self, area: Area, location: Location, **kwargs) -> bool:
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
