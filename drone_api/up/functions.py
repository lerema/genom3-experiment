# Copyright 2022 Selvakumar H S, LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Functions to be called for estimating the drone and environment state."""
import logging

from drone_api.up.user_types import Location, Robot
from drone_api.core.data import JSONSerializer

logger = logging.getLogger("[UP]")
logger.setLevel(logging.INFO)


def is_surveyed():
    """Check if the survey is completed."""
    data = JSONSerializer().get("ENV.SURVEY_AREA")
    logger.info("Checking if the survey is completed")
    return bool(data)


def has_plates():
    """Check if the plate is collected."""
    data = JSONSerializer().get("ENV.NO_PLATES")
    logger.info("Checking if the plate is collected")
    return bool(data)


def robot_at(robot: Robot, location: Location = ""):
    """Check if the robot is at the location."""
    data = JSONSerializer().get(f"ROBOTS.{robot.id}.location_name")
    logger.info(f"Checking if the robot is at the location {location.name}")
    return bool(data == location.name)


def is_base_station(robot: Robot, location: Location = ""):
    """Check if the location is the base station."""
    data = JSONSerializer().get(f"ROBOTS.{robot.id}.location_name")
    logger.info(f"Checking if the location is the base station {location.name}")
    return bool(data == location.name)


def get_plates_no():
    """Get the number of plates collected."""
    data = JSONSerializer().get("ENV.NO_PLATES")
    logger.info("Getting the number of plates collected")
    return data


def get_arucos_no():
    """Get the number of arucos collected."""
    data = JSONSerializer().get("ENV.NO_ARUCOS")
    logger.info("Getting the number of arucos collected")
    return data


def get_plate_info(plate_id: int):
    """Get the plate information."""
    data = JSONSerializer().get(f"ENV.PLATES.{plate_id}")
    logger.info(f"Getting the plate information {plate_id}")
    return data


def get_aruco_info(aruco_id: int):
    """Get the aruco information."""
    data = JSONSerializer().get(f"ENV.ARUCOS.{aruco_id}")
    logger.info(f"Getting the aruco information {aruco_id}")
    return data


def is_location_inspected(location: Location):
    """Check if the location is inspected."""
    data = JSONSerializer().get(f"ENV.LOCATIONS.{location.name}.inspected")
    logger.info(f"Checking if the location is inspected {location.name}")
    return bool(data)


def is_plate_order_optimized():
    """Check if the plates order is optimized."""

    data = JSONSerializer().get("ENV.PLATES.ORDER_OPTIMIZED")
    logger.info(f"Checking if the plates order are optimized: {data}")
    return bool(data)


def all_plates_inspected():
    """Check if all plates are inspected."""

    if get_plates_no() < get_spawned_plates_no():
        return False

    for plate_id in range(get_plates_no()):
        data = get_plate_info(plate_id)

        if bool(data["INSPECTED"]) is not True:
            return False

    return True


def is_plate_inspected(location: Location):
    """Check if the plate at specific locatin is inspected."""

    # Split plate number with _
    plate_id = int(location.name.split("_")[-1])
    data = get_plate_info(plate_id)

    return bool(data["INSPECTED"])


def is_robot_available(robot: Robot):
    """Check for robot's availability"""

    data = JSONSerializer().get(f"ROBOTS.{robot.id}.is_available")

    return bool(data)


def get_spawned_plates_no():
    """Get spawned plates no."""

    import os

    env_path = os.getenv("DRONE_VV_PATH", None)

    if env_path is not None:
        data_path = os.path.join(env_path, "genom3-experiment/data/data")

        with open(data_path, "r") as f:
            line = f.readline()

            return int(line.split("=")[-1])
