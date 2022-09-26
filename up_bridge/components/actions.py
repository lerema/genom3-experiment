"""Action representation for the UP project."""
from typing import Callable

from drone_api.actions import Actions
from up_bridge.components.fluents import Fluents
from up_bridge.components.user_types import Area, Location


class BridgeAction:
    """Create a Action Bridge object."""

    def __init__(self, name, components, **kwargs):
        self.name = name
        self.components = components # TODO: Remove this
        self.parameters = kwargs
        self.preconditions = []
        self.effects = []
        self.duration = 0
        self._execute_action: Callable = None

    def add_preconditions(self, preconditions):
        self.preconditions = preconditions

    def add_effects(self, effects):
        self.effects = effects

    def add_precondition(self, _callable: Callable, output=None):
        self.preconditions.append({_callable: output})

    def add_effect(self, _callable: Callable):
        self.effects.append(_callable)

    def set_duration(self, duration):
        self.duration = duration

    def _check_preconditions(self, preconditions=[]):
        for (precondition, value) in preconditions:
            assert (
                precondition() == value
            ), f"{precondition} != {value}. Failed precondition."

    def _execute_effects(self, effects=[]):
        for effect in effects:
            effect()

    def __call__(self, *args, **kwds):
        self._check_preconditions(self.preconditions)
        self._execute_effects(self.effects)
        self._execute_action(*args, **kwds)


class Move:
    """Move action"""

    area = Area
    l_from = Location
    l_to = Location

    def __init__(self, **kwargs):
        self.api = BridgeAction(name="move", components=kwargs["components"], **kwargs)
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects

    def __call__(self, area: Area, l_from: Location, l_to: Location):
        # Preconditions
        self.api.add_precondition(Fluents.robot_at(l_to), True)
        self.api.add_precondition(Fluents.robot_at(l_to), False)
        self.api.add_precondition(Fluents.is_surveyed(area), True)
        self.api.add_precondition(Fluents.is_location_surveyed(area), True)

        # Effects
        self.api.add_effect(Fluents.robot_at(l_to), True)
        self.api.add_effect(Fluents.robot_at(l_from), False)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).move(area, l_from, l_to)
        self._execute_effects(self.effects)


class Survey(BridgeAction):
    """Survey action"""

    area = Area

    def __init__(self, **kwargs):
        self.api = BridgeAction(
            name="survey", components=kwargs["components"], area=self.rea
        )
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects

    def __call__(self, area: Area):
        # Preconditions
        self.api.add_precondition(Fluents.is_surveyed(area), False)

        # Effects
        self.api.add_effect(Fluents.is_surveyed(area), True)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).survey(area)
        self._execute_effects(self.effects)


class CapturePhoto(BridgeAction):
    """CapturePhoto action"""

    area = Area
    location = Location

    def __init__(self, **kwargs):
        self.api = BridgeAction(
            name="capture_photo", components=kwargs["components"], **kwargs
        )
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects

    def __call__(self, area: Area, location: Location):
        # Preconditions
        self.api.add_precondition(Fluents.robot_at(location), True)
        self.api.add_precondition(Fluents.is_surveyed(area), True)
        self.api.add_precondition(Fluents.is_location_surveyed(area), True)

        # Effects
        self.api.add_effect(Fluents.verify_station_at(location), True)
        self.api.add_effect(Fluents.robot_at(location), True)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).capture_photo(area, location)
        self._execute_effects(self.effects)


class SendInfo(BridgeAction):
    """SendInfo action"""

    area = Area
    location = Location

    def __init__(self, **kwargs):
        self.api = BridgeAction(
            name="send_info", components=kwargs["components"], **kwargs
        )
        self.preconditions = self.api.preconditions
        self.effects = self.api.effects

    def __call__(self, area: Area, location: Location):
        # Preconditions
        self.api.add_precondition(Fluents.is_surveyed(area), True)
        self.api.add_precondition(Fluents.is_within_area(area, location), True)

        # Effects
        self.api.add_effect(Fluents.is_location_surveyed(area), True)

        self._check_preconditions(self.preconditions)
        Actions(components=self.components).send_info(area, location)
        self._execute_effects(self.effects)
