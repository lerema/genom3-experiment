"""Actions for the drone."""
import logging
import math
import time

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Actions:
    def __init__(self, components):
        self.land = Land(components)
        self.takeoff = Takeoff(components)
        self.move = Move(components)
        self.stop = Stop(components)
        self.surveyx = SurveyX(components)
        self.surveyy = SurveyY(components)
        self.survey = self.surveyx
        self.send_info = SendInfo(components)
        self.capture_photo = CapturePhoto(components)


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
            {"height": 0.15 if self._is_robot else 0.05, "duration": 5}, ack=self.ack
        )
        result = self.maneuver.wait()

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

    def __call__(self, l_from: dict = None, l_to: dict = None):

        assert l_from is not None, "l_from is not defined"
        assert l_to is not None, "l_to is not defined"

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
                "duration": 5,
            },
            ack=self.ack,
        )
        result = self.maneuver.wait()

        return result


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
        z = area.get("z", -5)
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


class SendInfo:
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

        logger.info(f"Checking Info at ({self.x}, {self.y}, {self.z}, {self.yaw})")

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
            },
        )

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
                "duration": 5,
            },
            ack=self.ack,
        )
        result = self.maneuver.wait()

        return result


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
        result = self.maneuver.take_off(height=height, duration=2)
        result = self.maneuver.take_off(height=0.15, duration=2)
        time.sleep(2)
        # TODO: Capture photo
        result = self.maneuver.take_off(height=height, duration=2, ack=self.ack)
        result = self.maneuver.wait()
        return result
