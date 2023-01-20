"""Capture photo action.

Stores a photo in the drone.
"""
import logging
import time

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class CapturePhoto:
    """Takeoff action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, area: dict = None, location: dict = None):
        assert area is not None, "area is not defined"
        assert location is not None, "l_to is not defined"

        if not isinstance(area, dict):
            area = area.__dict__()
        if not isinstance(location, dict):
            location = location.__dict__()
        height = location.get("z", 0.15)
        # TODO: check if the location is in the area

        logger.info(f"Capturing photo off at {location}")
        result = self.maneuver.take_off(height=height, duration=0)
        result = self.maneuver.take_off(height=0.15, duration=0)
        time.sleep(2)
        # TODO: Capture photo
        result = self.maneuver.take_off(height=height, duration=0, ack=self.ack)
        result = self.maneuver.wait()
        return result
