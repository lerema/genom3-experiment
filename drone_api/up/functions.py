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
from drone_api.up.user_types import Area, Location, Robot
from drone_api.core.data import JSONSerializer


def is_surveyed():
    """Check if the survey is completed."""
    data = JSONSerializer().get("ENV.SURVEY_AREA")
    return bool(data)


def has_plates():
    """Check if the plate is collected."""
    data = JSONSerializer().get("ENV.NO_PLATES")
    return bool(data)


def robot_at(robot: Robot, location: Location = ""):
    """Check if the robot is at the location."""
    data = JSONSerializer().get(f"ROBOTS.{robot.id}.location_name")
    return bool(data == location.name)


def is_base_station(robot: Robot, location: Location = ""):
    """Check if the location is the base station."""
    data = JSONSerializer().get(f"ROBOTS.{robot.id}.location_name")
    return bool(data == location.name)
