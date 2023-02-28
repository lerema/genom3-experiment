"""Drone API actions."""
from .capture_photo import CapturePhoto
from .takeoff import Takeoff
from .gather_info import GatherInfo
from .land import Land
from .localize_plates import LocalizePlates
from .move import Move
from .stop import Stop
from .survey import Survey
from .inspect_aruco import DetectArucotag


class Actions:
    def __init__(self, components, robot_id=0):
        self._land = Land(components, robot_id)
        self._takeoff = Takeoff(components, robot_id)
        self._move = Move(components, robot_id)
        self._stop = Stop(components, robot_id)
        self._survey = Survey(components, robot_id)
        self._gather_info = GatherInfo(components, robot_id)
        self._capture_photo = CapturePhoto(components, robot_id)
        self._localize_plates = LocalizePlates(components, robot_id)
        self._detect_arucotag = DetectArucotag(components, robot_id)

    def land(self, **kwargs):
        return self._land(**kwargs)

    def takeoff(self, **kwargs):
        return self._takeoff(**kwargs)

    def move(self, **kwargs):
        return self._move(**kwargs)

    def stop(self, **kwargs):
        return self._stop(**kwargs)

    def survey(self, **kwargs):
        return self._survey(**kwargs)

    def gather_info(self, **kwargs):
        return self._gather_info(**kwargs)

    def capture_photo(self, **kwargs):
        return self._capture_photo(**kwargs)

    def localize_plates(self, **kwargs):
        return self._localize_plates(**kwargs)

    def detect_arucotag(self, **kwargs):
        return self._detect_arucotag(**kwargs)
