"""Survey action."""
import logging
import math

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)

# FIXME: The survey action is not continuous and waits at some point for the drone to reach the next waypoint
# Possibly due to duration argument
class SurveyX:
    """Survey action along x-axis for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ct_drone = components["CT_drone"].component
        self.ack = True
        self._step_size = 1.0
        self.speed = 1.0

    def __call__(self, area: dict = None):

        assert area is not None, "Area is not defined"

        if not isinstance(area, dict):
            area = area.__dict__()

        xmin = area.get("xmin", -5)
        ymin = area.get("ymin", -5)
        xmax = area.get("xmax", 5)
        ymax = area.get("ymax", 5)
        z = area.get("z", 1)
        yaw = area.get("yaw", 0)
        logger.info(f"Surveying from ({xmin}, {ymin}) to ({xmax}, {ymax})")
        self.ct_drone.ReadROSImageUpdateFindings(ack=self.ack)
        self.maneuver.set_bounds(
            {
                "xmin": -100,
                "xmax": 100,
                "ymin": -100,
                "ymax": 100,
                "zmin": -2,
                "zmax": 30,
                "yawmin": -10.0,
                "yawmax": 10.0,
            },
            ack=self.ack,
        )
        y = ymin
        while y < ymax:
            result = self.maneuver.goto(
                {
                    "x": xmin,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "duration": (xmax - xmin) / self.speed,
                }
            )
            result = self.maneuver.goto(
                {
                    "x": xmax,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "duration": (xmax - xmin) / self.speed,
                }
            )
            y += self._step_size
            yaw = math.pi
            result = self.maneuver.goto(
                {
                    "x": xmax,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "duration": 0,
                }
            )
            result = self.maneuver.goto(
                {
                    "x": xmin,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "duration": (xmax - xmin) / self.speed,
                }
            )
            y += self._step_size
            yaw = 0.0
            result = self.maneuver.goto(
                {
                    "x": xmin,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "duration": 0,
                }
            )
        return result

    def callback(self, data):
        print(data.status)


# FIXME: this action
class SurveyY:
    """Survey action along y-axis for the drone"""

    def __init__(self, components):
        self.maneuver = components["maneuver"].component
        self.ct_drone = components["CT_drone"].component
        self.ack = True
        self._step_size = 1.0
        self.speed = 1.0

    def __call__(self, **kwargs):
        xmin = kwargs.get("xmin", -5)
        ymin = kwargs.get("ymin", -5)
        xmax = kwargs.get("xmax", 5)
        ymax = kwargs.get("ymax", 5)
        z = kwargs.get("z", -5)
        yaw = kwargs.get("yaw", 0)
        logger.info(f"Surveying from ({xmin}, {ymin}) to ({xmax}, {ymax})")
        # self.ct_drone.ReadROSImageUpdateFindings(ack=self.ack)
        self.maneuver.set_bounds(
            {
                "xmin": -100,
                "xmax": 100,
                "ymin": -100,
                "ymax": 100,
                "zmin": -2,
                "zmax": 30,
                "yawmin": -10.0,
                "yawmax": 10.0,
            },
            ack=self.ack,
        )
        x = xmin
        yaw = math.pi / 2
        result = self.maneuver.goto(
            {
                "x": x,
                "y": ymin,
                "z": z,
                "yaw": yaw,
                "duration": (ymax - ymin) / self.speed,
            }
        )
        while x < xmax:
            result = self.maneuver.goto(
                {
                    "x": x,
                    "y": ymax,
                    "z": z,
                    "yaw": yaw,
                    "duration": (ymax - ymin) / self.speed,
                }
            )
            x += self._step_size
            result = self.maneuver.goto(
                {
                    "x": x,
                    "y": ymax,
                    "z": z,
                    "yaw": -yaw,
                    "duration": 0,
                }
            )
            result = self.maneuver.goto(
                {
                    "x": x,
                    "y": ymin,
                    "z": z,
                    "yaw": -yaw,
                    "duration": (ymax - ymin) / self.speed,
                }
            )
            x += self._step_size
            result = self.maneuver.goto(
                {
                    "x": x,
                    "y": ymin,
                    "z": z,
                    "yaw": yaw,
                    "duration": 0,
                }
            )
        return result

    def callback(self, data):
        print(data.status)
