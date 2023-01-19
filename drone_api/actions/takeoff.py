"""Takeoff action."""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)

class Takeoff:
    """Takeoff action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, **kwargs):
        height = kwargs.get("height", 0.15)
        duration = kwargs.get("duration", 0)
        logger.info(f"Taking off to {height}")
        result = self.maneuver.take_off(height=height, duration=duration, ack=self.ack)
        result = self.maneuver.wait()
        return result
