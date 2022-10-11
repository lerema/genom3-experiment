# Copyright 2022 Selvakumar H S
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


def get_battery_level(components):
    """Return the battery level of the drone."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"]


def get_robot_pose(components):
    """Return the robot pose."""

    pom = components["maneuver"].component
    data = pom.get_reference()

    return data["reference"]["pos"]


def monitor_battery(components):
    """Return True if the battery is low."""

    data = components["rotorcraft"].component.get_battery()
    return data["level"] > 1
