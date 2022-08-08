"""Actions for the drone."""
import logging
from drone_api import Connector

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Land:
    """Land action for the drone"""

    _is_robot = False

    def __init__(self, components):
        self.rotorcraft, self.maneuver = (
            components["rotorcraft"].component,
            components["maneuver"].component,
        )
        self.ack = True

    def __call__(self):
        logger.info("Landing")

        # self.rotorcraft.set_velocity(
        #     {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        # )

        result = self.maneuver.take_off(
            {"height": 0.15 if self._is_robot else 0.05, "duration": 0}, ack=self.ack
        )

        return result


class Stop:
    """Stop action for the drone"""

    logger.info("Stopping the controller")

    def __init__(self, components, **kwargs):
        self.rotorcraft = components["rotorcraft"].component
        self.ack = True

    def __call__(self):
        self.rotorcraft.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, ack=self.ack
        )
        result = self.rotorcraft.stop()

        return result


class Move:
    """Move action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, **kwargs):
        self.x = kwargs.get("x", 0.0)
        self.y = kwargs.get("y", 0.0)
        self.z = kwargs.get("z", 0.15)
        self.yaw = kwargs.get("yaw", 0.0)

        logger.info(f"Moving to ({self.x}, {self.y}, {self.z}, {self.yaw})")

        # self.maneuver.set_bounds(
        #     {
        #         "xmin": -20,
        #         "xmax": 20,
        #         "ymin": -20,
        #         "ymax": 20,
        #         "zmin": -2,
        #         "zmax": 10,
        #         "yawmin": -3.14,
        #         "yawmax": 3.14,
        #     },
        #     ack=self.ack,
        # )
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


class Takeoff:
    """Takeoff action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, **kwargs):
        height = kwargs.get("height", 0.15)
        logger.info(f"Taking off to {height}")
        result = self.maneuver.take_off(height=height, duration=0, ack=self.ack)

        return result
