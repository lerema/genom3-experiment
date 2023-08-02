"""Survey action."""
import logging
import math

from genomix import Status

from drone_api.core.data import JSONSerializer

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Survey:
    """Survey action along x-axis for the drone"""

    def __init__(self, components, robot_id=0):
        self._components = ["maneuver", "CT_drone"]
        self.maneuver = components["maneuver"].genomix
        self.ct_drone = components["CT_drone"].genomix
        self.ack = True
        self._x_step_size = 1
        self._y_step_size = 1
        self.speed = 1.0

        self._status = None
        self._id = robot_id

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        self._data = JSONSerializer()

        self.ct_drone.ReadROSImageUpdateFindings(ack=self.ack, callback=self.callback)

    @property
    def components(self):
        return self._components

    def __call__(self, area: dict = None, **kwargs):
        assert area is not None, "Area is not defined"

        if not isinstance(area, dict):
            area = area.__dict__()

        xmin = area.get("xmin", -5.0)
        ymin = area.get("ymin", -5.0)
        xmax = area.get("xmax", 5.0)
        ymax = area.get("ymax", 5.0)
        z = area.get("z", 1.0)
        yaw = area.get("yaw", 0.0)
        logger.info(f"Surveying from ({xmin}, {ymin}) to ({xmax}, {ymax})")
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

        # Add waypoints
        input_dict = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "wz": 0.0,
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "duration": 0.0,
        }

        x = xmin
        self._y_step_size = ymax - ymin
        while x <= xmax:
            y = ymin
            while y <= ymax:
                input_dict.update({"x": x, "y": y, "z": z, "yaw": yaw})
                self.maneuver.waypoint(input_dict)
                y += self._y_step_size
            x += self._x_step_size

        self.robot_x, self.robot_y, self.robot_z = xmax, ymax, z
        self._survey_coordinates = [xmin, ymin, xmax, ymax, z, yaw]

        return self.maneuver.wait(ack=self.ack, callback=self.callback)

    def callback(self, request):
        self._status = request.status
        if self._status == Status.done:
            self._data.update(
                f"ROBOTS.{self._id}.pose",
                [
                    self.robot_x,
                    self.robot_y,
                    self.robot_z,
                ],
            )
            self._data.update("ENV.SURVEY_AREA", self._survey_coordinates)


class SurveyX:
    """Survey action along x-axis for the drone"""

    def __init__(self, components):
        self._components = ["maneuver"]
        self.maneuver = components["maneuver"].component
        self.ack = True
        self._step_size = 1.0
        self.speed = 1.0

    @property
    def components(self):
        return self._components

    def __call__(self, area: dict = None, **kwargs):
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
                {"x": xmax, "y": y, "z": z, "yaw": yaw, "duration": 0}
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
                {"x": xmin, "y": y, "z": z, "yaw": yaw, "duration": 0}
            )
        return result

    def callback(self, data):
        print(data.status)
