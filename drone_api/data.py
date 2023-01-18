"""Create a JSON file for storing robot and environment state."""

import os
import json

from typing import Tuple


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
                "battery_level": self.battery_level,
            }
        except AttributeError:
            raise ValueError("Robot object not initialized.")


class Environment:
    def __init__(self, no_robots: int = 1, no_plates: int = 1, no_arucos: int = 1):
        self.NO_ROBOTS = no_robots
        self.NO_PLATES = no_plates
        self.NO_ARUCOS = no_arucos

        self.ARUCO_POSES = [None for _ in range(self.NO_ARUCOS)]
        self.PLATE_POSES = [None for _ in range(self.NO_PLATES)]

        self.HOME = (0.0, 0.0, 0.0)
        self.SURVEY_AREA = (-4.0, 4.0, -4.0, 4.0, 3.0, 0.0)

    def __dict__(self):
        try:
            return {
                "NO_ROBOTS": self.NO_ROBOTS,
                "NO_PLATES": self.NO_PLATES,
                "NO_ARUCOS": self.NO_ARUCOS,
                "HOME": self.HOME,
                "SURVEY_AREA": self.SURVEY_AREA,
                "ARUCO_POSES": self.ARUCO_POSES,
                "PLATE_POSES": self.PLATE_POSES,
            }
        except AttributeError:
            raise ValueError("Environment object not initialized.")


class DataRepresentation:
    """Data representation class."""

    def __init__(
        self, no_robots: int = 1, no_plates: int = 1, no_arucos: int = 1
    ) -> None:
        self.ROBOTS = [Robot()] * no_robots
        self.ENV = Environment()

        self.ENV.NO_ARUCOS = no_arucos
        self.ENV.NO_PLATES = no_plates
        self.ENV.NO_ROBOTS = no_robots
        self.ENV.ARUCO_POSES = [None for _ in range(no_arucos)]
        self.ENV.PLATE_POSES = [None for _ in range(no_plates)]

    def __dict__(self):
        ROBOTS = {}
        for i in range(self.ENV.NO_ROBOTS):
            ROBOTS[i] = self.ROBOTS[i].__dict__()

        return {
            "ROBOTS": ROBOTS,
            "ENV": self.ENV.__dict__(),
        }


class JSONSerializer:
    """JSON Serialization class.

    This class stores the data in a JSON file and reads from it during runtime.
    """

    def __init__(self, filename: str = "genom3-experiment-data.json") -> None:
        self.filename = filename
        self.data = DataRepresentation()

        # Create the file if it doesn't exist
        self._create_file()

    def _create_file(self) -> None:
        """Create a JSON file for storing robot and environment state."""
        if os.path.exists(self.filename):
            return

        with open(self.filename, "w") as f:
            json.dump(self.data.__dict__(), f, indent=4)

    def _update_key(self, key: str, value) -> None:
        """Update a key in the JSON file."""
        with open(self.filename, "r") as f:
            data = json.load(f)

        # If key has a dot, then it is a nested key
        data_copy = data
        if "." in key:
            keys = key.split(".")
            data_copy = self.get(keys[0])

            for k in keys[1:-1]:
                data_copy = data_copy[k]

            data_copy[keys[-1]] = value
            data[keys[0]] = data_copy
        else:
            data[key] = value

        with open(self.filename, "w") as f:
            json.dump(data, f, indent=4)

    def update(self, key: str, value) -> None:
        """Update a key in the JSON file."""
        self._update_key(key, value)

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

    def __del__(self) -> None:
        """Delete the JSON file."""
        os.remove(self.filename)
