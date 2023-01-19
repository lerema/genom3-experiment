"""Move action."""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)

class Move:
    """Move action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, area: dict = None, l_from: dict = None, l_to: dict = None):

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

        result = self.maneuver.waypoint(
            {
                "x": self.x,
                "y": self.y,
                "z": self.z,
                "yaw": self.yaw,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "wz": 0,
                "ax": 0,
                "ay": 0,
                "az": 0,
                "duration": 0,
            },
            ack=self.ack,
        )

        return result