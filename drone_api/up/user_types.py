from drone_api.actions import Actions


class Location:
    """Location of the object understandable with the robot."""

    def __init__(self, name: str = "", x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.name = name
        self.x = x
        self.y = y
        self.z = z

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

    def __repr__(self):
        return f"{self.name}-{self.id}"
