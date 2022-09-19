"""Functions to be called for estimating the drone and environment state."""


def get_battery_level(components):
    """Return the battery level of the drone."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"]


def get_robot_pose(components):
    """Return the robot pose."""

    pom = components["maneuver"].component
    data = pom.get_reference()

    return data["pos"]


def monitor_battery(components):
    """Return True if the battery is low."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"] > 1
