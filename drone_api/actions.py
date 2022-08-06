"""Actions for the drone."""
import logging
import time
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

    def __call__(self):
        logger.info("Landing")

        self.rotorcraft.set_velocity(
            {
                "desrired": {
                    "1": 0,
                    "2": 0,
                    "3": 0,
                    "4": 0,
                    "5": 0,
                    "6": 0,
                    "7": 0,
                }
            }
        )

        self.maneuver.take_off(
            {"height": 0.15 if self._is_robot else 0.05, "duration": 0}
        )


class Stop:
    """Stop action for the drone"""

    def __init__(self, components, **kwargs):
        self.rotorcraft = components["rotorcraft"].component

    def __call__(self):
        self.rotorcraft.set_velocity(
            {
                "desrired": {
                    "1": 0,
                    "2": 0,
                    "3": 0,
                    "4": 0,
                    "5": 0,
                    "6": 0,
                    "7": 0,
                }
            }
        )
        self.rotorcraft.stop()


class Move:
    """Move action for the drone"""

    def __init__(self, components, **kwargs):
        self.maneuver = components["maneuver"].component
        help(self.maneuver)

    def __call__(self):
        self.maneuver.set_bounds(
            {
                "xmin": -20,
                "xmax": 20,
                "ymin": -20,
                "ymax": 20,
                "zmin": -2,
                "zmax": 10,
                "yawmin": -3.14,
                "yawmax": 3.14,
            }
        )
        self.maneuver.waypoint(
            {
                "x": 1.5,
                "y": 0,
                "z": 4,
                "yaw": 0,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "wz": 0,
                "ax": 0,
                "ay": 0,
                "az": 0,
                "duration": 0,
            }
        )
        self.maneuver.waypoint(
            {
                "x": 0,
                "y": 0,
                "z": 4,
                "yaw": 0,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "wz": 0,
                "ax": 0,
                "ay": 0,
                "az": 0,
                "duration": 0,
            }
        )
        self.maneuver.waypoint(
            {
                "x": -1.5,
                "y": 0,
                "z": 4,
                "yaw": 0,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "wz": 0,
                "ax": 0,
                "ay": 0,
                "az": 0,
                "duration": 0,
            }
        )
        self.maneuver.waypoint(
            {
                "x": 0,
                "y": 0,
                "z": 4,
                "yaw": 0,
                "vx": 0,
                "vy": 0,
                "vz": 0,
                "wz": 0,
                "ax": 0,
                "ay": 0,
                "az": 0,
                "duration": 0,
            }
        )
        self.maneuver.wait()


def main():
    connect = Connector()
    connect.start()
    Land(connect.components)()


if __name__ == "__main__":
    main()
