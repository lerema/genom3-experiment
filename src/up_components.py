from typing import Any, Dict

from drone_api.actions import Actions
from drone_api.up.functions import get_battery_level, get_robot_pose
from drone_api.utils import Singleton
from up_bridge.components import ActionDefinition, UserTypeDefinition


class Facts(Singleton):
    is_surveyed: Dict[Any, bool] = {}
    is_location_surveyed: Dict[Any, bool] = {}


####################################################################################################
###########################################  UserTypes  ############################################
####################################################################################################
class Location(UserTypeDefinition):
    """Location of the object understandable with the robot."""

    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    parameters = {"x": x, "y": y, "z": z, "yaw": yaw}


class Area(UserTypeDefinition):
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


####################################################################################################
###########################################  Fluents  ##############################################
####################################################################################################


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


####################################################################################################
###########################################  Actions  #############################################
####################################################################################################
class Move(ActionDefinition, Fluents):
    """Move action"""

    area = Area
    l_from = Location
    l_to = Location

    def __init__(self, **kwargs):
        kwargs["area"] = self.area
        kwargs["l_from"] = self.l_from
        kwargs["l_to"] = self.l_to
        self.api = ActionDefinition(name="move", **kwargs)
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects
        self.components = kwargs["components"]

    def __call__(self, area: Area, l_from: Location, l_to: Location):
        # Preconditions
        self.api.add_precondition(self.robot_at, True, location=l_from)
        self.api.add_precondition(self.robot_at, False, location=l_to)
        self.api.add_precondition(self.is_surveyed, True, area=area)
        self.api.add_precondition(
            self.is_location_surveyed, True, area=area, location=l_to
        )

        # Effects
        self.api.add_effect(self.robot_at, True, location=l_to)
        self.api.add_effect(self.robot_at, False, location=l_from)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).move(area, l_from, l_to)
        self._execute_effects(self.effects)


class Survey(ActionDefinition, Fluents):
    """Survey action"""

    area = Area

    def __init__(self, **kwargs):
        kwargs["area"] = self.area
        self.api = ActionDefinition(name="survey", **kwargs)
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects
        self.components = kwargs["components"]

    def __call__(self, area: Area):
        result = False

        # Preconditions
        self.api.add_precondition(self.is_surveyed, False, area=area)

        # Effects
        self.api.add_effect(self.is_surveyed, True, area=area)

        result = self._check_preconditions(self.preconditions)
        result = Actions(components=self.components).survey(area)
        result = self._execute_effects(self.effects)

        return bool(result)


class CapturePhoto(ActionDefinition, Fluents):
    """CapturePhoto action"""

    area = Area
    location = Location

    def __init__(self, **kwargs):
        kwargs["area"] = self.area
        kwargs["location"] = self.location
        self.api = ActionDefinition(name="capture_photo", **kwargs)
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects
        self.components = kwargs["components"]

    def __call__(self, area: Area, location: Location):
        # Preconditions
        self.api.add_precondition(self.robot_at, True, location=location)
        self.api.add_precondition(self.is_surveyed, True, area=area)
        self.api.add_precondition(
            self.is_location_surveyed, True, area=area, location=location
        )

        # Effects
        self.api.add_effect(self.verify_station_at, True, location=location)
        self.api.add_effect(self.robot_at, True, location=location)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).capture_photo(area, location)
        self._execute_effects(self.effects)


class GatherInfo(ActionDefinition, Fluents):
    """GatherInfo action"""

    area = Area
    location = Location

    def __init__(self, **kwargs):
        kwargs["area"] = self.area
        kwargs["location"] = self.location
        self.api = ActionDefinition(name="send_info", **kwargs)
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects
        self.components = kwargs["components"]

    def __call__(self, area: Area, location: Location):
        # Preconditions
        self.api.add_precondition(self.is_surveyed, True, area=area)
        self.api.add_precondition(
            self.is_within_area, True, area=area, location=location
        )

        # Effects
        self.api.add_effect(
            self.is_location_surveyed, True, area=area, location=location
        )

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).gather_info(area, location)
        self._execute_effects(self.effects)
