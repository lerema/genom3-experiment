"""Move action."""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Move:
    """Move action for the drone"""

    def __init__(self, components):
        self._components = ["maneuver"]
        self.maneuver = components["maneuver"].component
        self.ack = True
        self._status = None

    @property
    def components(self):
        return self._components

    def __call__(
        self, area: dict = None, l_from: dict = None, l_to: dict = None, **kwargs
    ):

        assert area is not None, "Area is not defined"
        assert l_from is not None, "l_from is not defined"
        assert l_to is not None, "l_to is not defined"

        # TODO: Check if the drone is in the area and warn if not

        if not isinstance(l_from, dict):
            l_from = l_from.__dict__()
        if not isinstance(l_to, dict):
            l_to = l_to.__dict__()

        self.x = l_to.get("x", 0.0)
        self.y = l_to.get("y", 0.0)
        self.z = l_to.get("z", 0.15)
        self.yaw = l_to.get("yaw", 0.0)

        logger.info(f"Moving to ({self.x}, {self.y}, {self.z}, {self.yaw})")

        result = self.maneuver.goto(
            {"x": self.x, "y": self.y, "z": self.z, "yaw": self.yaw, "duration": 0},
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        return result

    def callback(self, request):
        self._status = request.status
