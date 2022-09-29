"""Action representation for the UP project."""
from typing import Callable

from drone_api.actions import Actions
from up_bridge.components.fluents import Fluents
from up_bridge.components.user_types import Area, Location


class ActionDefinition:
    """Create a Action Bridge object."""

    def __init__(self, name, **kwargs):
        self.name = name
        self.parameters = kwargs
        self.preconditions = []
        self.effects = []
        self.duration = 0
        self._execute_action: Callable = None

    def add_preconditions(self, preconditions):
        self.preconditions = preconditions

    def add_effects(self, effects):
        self.effects = effects

    def add_precondition(self, _callable: Callable, output=None, **kwargs):
        self.preconditions.append((_callable, output, kwargs))

    def add_effect(self, _callable: Callable, output=None, **kwargs):
        self.effects.append((_callable, output, kwargs))

    def set_duration(self, duration):
        self.duration = duration

    def _check_preconditions(self, preconditions=[]):
        ret = False
        for condition in preconditions:
            precondition, value, args = condition
            assert (
                precondition(expected_value=value, **args) == value
            ), f"{precondition} != {value}. Failed precondition."

            ret = True
        return ret

    def _execute_effects(self, effects=[]):
        ret = False
        for effect in effects:
            eff, value, args = effect
            result = eff(expected_value=value, **args)

            assert result == value, f"{result} != {value}. Failed effect."
            ret = True

        return ret

    def __call__(self, *args, **kwds):
        self._check_preconditions(self.preconditions)
        self._execute_effects(self.effects)
        self._execute_action(*args, **kwds)


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
