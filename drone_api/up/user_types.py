import logging

from drone_api.actions import Actions


logger = logging.getLogger("[UP]")
logger.setLevel(logging.INFO)


class Location:
    """Location of the object understandable with the robot."""

    def __init__(self, name: str = "", x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.name = name
        self.x = x
        self.y = y
        self.z = z

        logger.info(f"Location {self.name} created")

    def __repr__(self):
        return f"{self.name}"


class Area:
    """Area of the object understandable with the robot."""

    def __init__(
        self, name: str = "", survey_size: float = 0.0, survey_height: float = 1.0
    ):
        self.name = name
        self.survey_size = survey_size
        self.survey_height = survey_height

        logger.info(f"Area {self.name} created")

    def __repr__(self):
        return f"{self.name}-{self.survey_size}"


class Robot:
    """Robot object."""

    def __init__(
        self,
        name: str,
        actions: Actions,
        robot_id: int = 0,
        pose: tuple = (0.0, 0.0, 0.0),
        battery_level: int = 100,
    ):
        self.name = name
        self.id = robot_id
        self.pose = pose
        self.battery_level = battery_level
        self.actions = actions

        logger.info(f"Robot {self.name} created")

    def __repr__(self):
        return f"{self.name}-{self.id}"
