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


def plate_found(components):
    """Return True if the plate is found."""

    try:
        _ = components["CT_drone"].component.TargetPose()
        return True
    except Exception:
        return False


def detect_aruco_marker(components, marker):
    """Return True if the aruco marker is found."""

    data = components["arucotag"].pose(str(marker))

    if (
        data["pos"]["x"] is not None
        and data["pos"]["y"] is not None
        and data["pos"]["z"] is not None
    ):
        return True
    else:
        return False


def get_aruco_pose(components, marker):
    """Return the aruco marker pose."""

    data = components["arucotag"].pose(str(marker))
    return data["pos"]
