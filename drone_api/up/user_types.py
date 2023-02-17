from up_bridge.components import UserTypeDefinition


class Location(UserTypeDefinition):
    """Location of the object understandable with the robot."""

    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    parameters = {"x": x, "y": y, "z": z, "yaw": yaw}


class Area(UserTypeDefinition):
    """Area of the object understandable with the robot."""

    name: str = ""
    xmin: float = 0.0
    xmax: float = 0.0
    ymin: float = 0.0
    ymax: float = 0.0
    z: float = 0.0
    yaw: float = 0.0

    parameters = {
        "xmin": xmin,
        "xmax": xmax,
        "ymin": ymin,
        "ymax": ymax,
        "z": z,
        "yaw": yaw,
    }
