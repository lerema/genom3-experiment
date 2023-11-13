"""Create a JSON file for storing robot and environment state."""

import os
import json

from drone_api import DATA_PATH


class Robot:
    """Robot Data Representation."""

    def __init__(
        self, robot_id: int = 1, pose: tuple = (0.0, 0.0, 0.0), battery_level: int = 100
    ):
        assert robot_id > 0, "Robot ID must be greater than 0."

        self.robot_id = robot_id
        self.pose = pose
        self.battery_level = battery_level

    def __dict__(self):
        try:
            return {
                "ID": self.robot_id,
                "pose": self.pose,
                "location_name": f"base_station_{self.robot_id}",
                "battery_level": self.battery_level,
                "is_available": True,
            }
        except AttributeError as e:
            raise ValueError("Robot object not initialized.") from e


class Environment:
    """Environment Data Representation."""

    def __init__(self, no_robots: int = 1, no_plates: int = 1, no_arucos: int = 1):
        self.no_robots = no_robots
        self.no_plates = no_plates
        self.no_arucos = no_arucos

        self.arucos = {}
        self.plates = {"ORDER_OPTIMIZED": False}

        self.home = (0.0, 0.0, 0.0)
        self.survey_area = []

    def __dict__(self):
        try:
            return {
                "NO_ROBOTS": self.no_robots,
                "NO_PLATES": self.no_plates,
                "NO_ARUCOS": self.no_arucos,
                "HOME": self.home,
                "SURVEY_AREA": self.survey_area,
                "ARUCOS": self.arucos,
                "PLATES": self.plates,
            }
        except AttributeError as e:
            raise ValueError("Environment object not initialized.") from e


class DataRepresentation:
    """Data representation class."""

    def __init__(
        self, no_robots: int = 5, no_plates: int = 0, no_arucos: int = 0
    ) -> None:
        self.robots = [Robot(robot_id=i + 1) for i in range(no_robots)]
        self.env = Environment()

        self.env.no_arucos = no_arucos
        self.env.no_plates = no_plates
        self.env.no_robots = no_robots
        self.env.ARUCO_POSES = []
        self.env.PLATE_POSES = []

    def __dict__(self):
        ROBOTS = {}
        for i in range(self.env.no_robots):
            ROBOTS[i + 1] = self.robots[i].__dict__()

        return {"ROBOTS": ROBOTS, "ENV": self.env.__dict__()}


class JSONSerializer:
    """JSON Serialization class.

    This class stores the data in a JSON file and reads from it during runtime.
    """

    filename = os.path.join(DATA_PATH, "genom3-experiment-data.json")

    def __init__(cls) -> None:
        cls.data = DataRepresentation()

        # Create the file if it doesn't exist
        cls._create_file()

    def _create_file(cls) -> None:
        """Create a JSON file for storing robot and environment state."""
        if os.path.exists(cls.filename):
            return

        if not os.path.exists(DATA_PATH):
            os.makedirs(DATA_PATH)

        with open(cls.filename, "w", encoding="utf-8") as f:
            json.dump(cls.data.__dict__(), f, indent=4)

    @classmethod
    def _update_key(cls, key: str, value) -> None:
        """Update a key in the JSON file."""
        with open(cls.filename, "r", encoding="utf-8") as f:
            data = json.load(f)

        # If key has a dot, then it is a nested key
        def update(data, key, value):
            if "." in key:
                key, rest = key.split(".", 1)
                data[key] = update(data.get(key, {}), rest, value)
            else:
                data[key] = value

            return data

        data = update(data, key, value)
        with open(cls.filename, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def update(cls, key: str, value) -> None:
        """Update a key in the JSON file."""
        cls._update_key(key, value)

    def get(self, key: str):
        """Get a key from the JSON file."""

        with open(self.filename, "r", encoding="utf-8") as f:
            data = json.load(f)

        # If key has a dot, then it is a nested key
        if "." in key:
            keys = key.split(".")
            data = self.get(keys[0])

            for k in keys[1:]:
                data = data[k]

            return data

        return data[key]

    def get_all(self):
        """Get all data from the JSON file."""
        with open(self.filename, "r", encoding="utf-8") as f:
            data = json.load(f)

        return data
