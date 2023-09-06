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
import os


USE_ROBOT = bool(os.environ.get("USE_ROBOT", False) == "1")
IS_OUTDOOR = bool(os.environ.get("IS_OUTDOOR", False) == "1")
DATA_PATH = "./data"

EXPECTED_MODULES = [
    "pom",
    "maneuver",
    "rotorcraft",
    "nhfc",
    "ColorTracker",
    "camgazebo",
    "arucotag",
    "camviz",
    "FoxgloveStudio",
]

COMMON_MODULES = [
    "optitrack"
]

if USE_ROBOT:
    EXPECTED_MODULES.remove("camgazebo")
    EXPECTED_MODULES.remove("camviz")
    EXPECTED_MODULES.insert(0, "d435")

if IS_OUTDOOR:
    COMMON_MODULES.append("gps")
    COMMON_MODULES.remove("optitrack")

MODULES = {
    "expected": EXPECTED_MODULES,
    "dedicated": set(EXPECTED_MODULES) - set(COMMON_MODULES),
    "common": COMMON_MODULES,
}

from .actions import *  # pylint: disable=unused-wildcard-import, wrong-import-position
from .utils import *  # pylint: disable=unused-wildcard-import, wrong-import-position
