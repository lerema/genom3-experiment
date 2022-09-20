EXPECTED_MODULES = [
    "optitrack",
    "maneuver",
    "pom",
    "rotorcraft",
    "nhfc",
    "CT_drone",
    "tf2",
    "arucotag",
]

COMMON_MODULES = ["tf2", "optitrack"]

MODULES = {
    "expected": EXPECTED_MODULES,
    "dedicated": set(EXPECTED_MODULES) - set(COMMON_MODULES),
    "common": COMMON_MODULES,
}

from .actions import *
from .utils import *


USE_ROBOT = False
