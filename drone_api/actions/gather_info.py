"""gather info action.

Basically moves to the home station and sends the info to the server.
"""
import logging
import time

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class GatherInfo:
    """Sending Info by Move action for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ack = True

    def __call__(self, area: dict = None, location: dict = None):

        assert area is not None, "area is not defined"
        assert location is not None, "l_to is not defined"

        # TODO: check if the location is in the area

        if not isinstance(area, dict):
            area = area.__dict__()
        if not isinstance(location, dict):
            location = location.__dict__()

        self.x = location.get("x", 0.0)
        self.y = location.get("y", 0.0)
        self.z = location.get("z", 0.15)
        self.yaw = location.get("yaw", 0.0)

        logger.info(
            f"Send Gathered Info from ({self.x}, {self.y}, {self.z}, {self.yaw})"
        )

        self.maneuver.set_bounds(
            {
                "xmin": area["xmin"],
                "xmax": area["xmax"],
                "ymin": area["ymin"],
                "ymax": area["ymax"],
                "zmin": -2,
                "zmax": 10,
                "yawmin": -3.14,
                "yawmax": 3.14,
            }
        )

        result = self.maneuver.goto(
            {"x": 0.0, "y": 0.0, "z": self.z, "yaw": self.yaw, "duration": 0}
        )
        time.sleep(
            1
        )  # TODO: Remove this once actual image is being sent or as simulated

        return result
