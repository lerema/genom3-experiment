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

EXPECTED_MODULES = [
    "optitrack",
    "maneuver",
    "pom",
    "rotorcraft",
    "nhfc",
    "CT_drone",
    "tf2",
    "d435",
    "arucotag",
    "camviz",
]

COMMON_MODULES = ["tf2", "optitrack"]

MODULES = {
    "expected": EXPECTED_MODULES,
    "dedicated": set(EXPECTED_MODULES) - set(COMMON_MODULES),
    "common": COMMON_MODULES,
}
DATA_PATH = "./data"


USE_ROBOT = True

if USE_ROBOT:
    try:
        EXPECTED_MODULES.remove("camgazebo")
        EXPECTED_MODULES.remove("camviz")
        EXPECTED_MODULES.append("d435")
    except ValueError:
        pass
from .actions import *
from .utils import *
