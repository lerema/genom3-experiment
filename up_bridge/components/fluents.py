"""Fluent representation."""
from drone_api.functions import get_battery_level, get_robot_pose
from up_bridge.components.user_types import Area, Location


class Fluents:
    components = None

    def robot_at(self, position: Location) -> bool:
        assert isinstance(position, Location), f"{position} is not a Location"
        result = get_robot_pose(self.components)

        return bool(result == dict(position))

    def verify_station_at(self, position: Location) -> bool:
        assert isinstance(position, Location), f"{position} is not a Location"

        # TODO: Implement this function
        # Check captured photo

        return self.robot_at(position)

    def is_surveyed(self, area: Area) -> bool:
        assert isinstance(area, Area), f"{area} is not a Area"
        return True

    def is_location_surveyed(self, area: Area, position: Location) -> bool:
        assert isinstance(area, Area), f"{area} is not a Area"
        assert isinstance(position, Location), f"{position} is not a Location"
        return True

    def is_within_area(self, area: Area, position: Location) -> bool:
        assert isinstance(area, Area), f"{area} is not a Area"
        assert isinstance(position, Location), f"{position} is not a Location"

        area = dict(area)
        position = dict(position)

        result = (
            area["xmin"] <= position["x"] <= area["xmax"]
            and area["ymin"] <= position["y"] <= area["ymax"]
        )
        return bool(result)

    def battery_at(self, level: int) -> bool:
        assert isinstance(level, int), f"{level} is not an int"

        return bool(get_battery_level(self.components) == level)
