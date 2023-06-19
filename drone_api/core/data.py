"""Create a JSON file for storing robot and environment state."""

import os
import json

from drone_api import DATA_PATH


class Robot:
    def __init__(
        self, ID: int = 0, pose: tuple = (0.0, 0.0, 0.0), battery_level: int = 100
    ):
        self.ID = ID
        self.pose = pose
        self.battery_level = battery_level

    def __dict__(self):
        try:
            return {
                "ID": self.ID,
                "pose": self.pose,
                "location_name": "base_station",
                "battery_level": self.battery_level,
                "is_available": True,
            }
        except AttributeError:
            raise ValueError("Robot object not initialized.")


class Environment:
    def __init__(self, no_robots: int = 1, no_plates: int = 1, no_arucos: int = 1):
        self.NO_ROBOTS = no_robots
        self.NO_PLATES = no_plates
        self.NO_ARUCOS = no_arucos

        self.ARUCOS = {}
        self.PLATES = {"ORDER_OPTIMIZED": False}

        self.HOME = (0.0, 0.0, 0.0)
        self.SURVEY_AREA = []

    def __dict__(self):
        try:
            return {
                "NO_ROBOTS": self.NO_ROBOTS,
                "NO_PLATES": self.NO_PLATES,
                "NO_ARUCOS": self.NO_ARUCOS,
                "HOME": self.HOME,
                "SURVEY_AREA": self.SURVEY_AREA,
                "ARUCOS": self.ARUCOS,
                "PLATES": self.PLATES,
            }
        except AttributeError:
            raise ValueError("Environment object not initialized.")


class DataRepresentation:
    """Data representation class."""

    def __init__(
        self, no_robots: int = 1, no_plates: int = 0, no_arucos: int = 0
    ) -> None:
        self.ROBOTS = [Robot()] * no_robots
        self.ENV = Environment()

        self.ENV.NO_ARUCOS = no_arucos
        self.ENV.NO_PLATES = no_plates
        self.ENV.NO_ROBOTS = no_robots
        self.ENV.ARUCO_POSES = []
        self.ENV.PLATE_POSES = []

    def __dict__(self):
        ROBOTS = {}
        for i in range(self.ENV.NO_ROBOTS):
            ROBOTS[i] = self.ROBOTS[i].__dict__()

        return {"ROBOTS": ROBOTS, "ENV": self.ENV.__dict__()}


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

        with open(cls.filename, "w") as f:
            json.dump(cls.data.__dict__(), f, indent=4)

    @classmethod
    def _update_key(cls, key: str, value) -> None:
        """Update a key in the JSON file."""
        with open(cls.filename, "r") as f:
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
        with open(cls.filename, "w") as f:
            json.dump(data, f, indent=4)

    @classmethod
    def update(cls, key: str, value) -> None:
        """Update a key in the JSON file."""
        cls._update_key(key, value)

    def get(self, key: str):
        """Get a key from the JSON file."""

        with open(self.filename, "r") as f:
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
        with open(self.filename, "r") as f:
            data = json.load(f)

        return data
