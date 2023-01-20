"""Drone API actions."""
from .capture_photo import CapturePhoto
from .takeoff import Takeoff
from .gather_info import GatherInfo
from .land import Land
from .localize_plates import LocalizePlates
from .move import Move
from .stop import Stop
from .survey import SurveyX


class Actions:
    def __init__(self, components):
        self.land = Land(components)
        self.takeoff = Takeoff(components)
        self.move = Move(components)
        self.stop = Stop(components)
        self.survey = SurveyX(components)
        self.gather_info = GatherInfo(components)
        self.capture_photo = CapturePhoto(components)
        self.localize_plates = LocalizePlates(components)
