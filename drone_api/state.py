"""Loads the current robot and environment state from the components to JSON."""
import threading
from drone_api.core import JSONSerializer, Robot, Environment

data_lock = threading.RLock()  # Locks the file access to the state file.

# FIXME: possible uncertainty in receiving the data from the components.
class RobotState:
    """Pushes the robot state to the serializer."""

    def __init__(self, components, id: int):
        self.components = components
        self.id = id

    def robot_pose(self):
        """Return the robot pose."""
        pom = self.components["pom"].component
        data = pom.frame("robot")

        return [
            data["frame"]["pos"]["x"],
            data["frame"]["pos"]["y"],
            data["frame"]["pos"]["z"],
        ]

    def get_battery_level(self):
        """Return the battery level of the drone."""
        data = self.components["rotorcraft"].component.get_battery()

        return (
            float(data["battery"]["level"])
            if data["battery"]["level"] is not None
            else None
        )

    def run(self):
        """Push the robot state to the serializer."""
        while True:
            with data_lock:
                data = Robot(self.id, self.robot_pose(), self.get_battery_level())
                JSONSerializer().update(f"ROBOT.{self.id}", data.__dict__())


def get_aruco_pose(components, marker):
    """Return the aruco marker pose."""
    data = components["arucotag"].pose(str(marker))
    return data["pos"]
