"""Takeoff action."""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Takeoff:
    """Takeoff action for the drone"""

    def __init__(self, components):
        self._components = ["maneuver"]
        self.maneuver = components["maneuver"].component
        self.ack = True
        self._status = None

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        height = kwargs.get("height", 0.15)
        duration = kwargs.get("duration", 0)
        logger.info(f"Taking off to {height}")
        result = self.maneuver.take_off(
            height=height,
            duration=duration,
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        return result

    def callback(self, request):
        self._status = request.status
