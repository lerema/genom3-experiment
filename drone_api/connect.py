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

"""Connect to genomix server, load and setup all pocolib modules"""
import glob
import logging
import os
import subprocess
import time
from drone_api.params import DRONES

import genomix

from drone_api import MODULES
from drone_api.genom3 import (
    Optitrack,
    POM,
    Maneuver,
    RotorCraft,
    NHFC,
    CTDrone,
    TF2,
    ArucoTag,
    CamGazebo,
    CamViz,
)

USE_CAM = True

LIB_PATH = os.environ["DRONE_VV_PATH"] + "/lib"

logger = logging.getLogger("[Drone Connector]")
logger.setLevel(logging.DEBUG)


class Connector:
    def __init__(self, id: int = 0, host: str = "localhost", port: int = 8080) -> None:
        # Attempt to start the genomix server
        self.id = id
        self.params = DRONES[self.id]
        self.genomix_process = subprocess.Popen(
            ["genomixd", "-d", "-v", "-v"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        self.handle = genomix.connect(f"{host}:{port}")
        self.components = {}

        assert self.handle is not None
        assert self.genomix_process is not None

        self._load_modules()
        if not set(MODULES["expected"]).issubset({*self.components}):
            raise ModuleNotFoundError(
                f"Failed to find all expected modules. Missing modules: {set(MODULES['expected']) - {*self.components}}"
            )

        self.components = {
            "optitrack": self._connect_optitrack(),
            "pom": self._connect_pom(),
            "maneuver": self._connect_maneuver(),
            "rotorcraft": self._connect_rotorcraft(),
            "nhfc": self._connect_nhfc(),
            "CT_drone": self._connect_ctdrone(),
            "tf2": self._connect_tf2(),
            "camgazebo": self._connect_camgazebo(),
            "arucotag": self._connect_arucotag(),
            "camviz": self._connect_camviz(),
        }

    def start(self):
        # for component in self.components.values():
        #     component.start()
        self.components["rotorcraft"].start()
        time.sleep(1)
        self.components["nhfc"].start()
        time.sleep(1)
        self.components["maneuver"].start()
        time.sleep(1)
        self.components["CT_drone"].start()

    def stop(self):
        self.components["rotorcraft"].stop()

    def __del__(self):
        pass
        # self._unload_modules()
        # del self.components
        # self.genomix_process.kill()
        # subprocess.call(["pkill", "gzserver"])
        # subprocess.call(["pkill", "gzclient"])
        # subprocess.call(["h2", "end"])
        # subprocess.call(["pkill", "genomixd"])
        # subprocess.call(["pkill", "-f", "\-pocolibs"])

    def _load_modules(self) -> None:
        """Load all pocolib modules"""

        assert os.path.isdir(LIB_PATH)
        modules = glob.glob(f"{LIB_PATH}/genom/*/plugins/*.so")
        try:
            modules.extend(glob.glob("/opt/openrobots/lib/genom/*/plugins/*.so"))
        except Exception:
            pass

        if self.id == 0:
            id = ""
        else:
            id = str(self.id)

        for module in modules:
            logger.debug(f"Loading module {os.path.basename(module)}")
            tag = os.path.basename(module).split(".")[0]
            try:
                if tag in MODULES["dedicated"]:
                    self.components[tag] = self.handle.load(module, "-i", f"{tag}{id}")
                elif tag in MODULES["common"]:
                    self.components[tag] = self.handle.load(module)
            except Exception as e:
                logger.error(
                    f"Failed to load module {os.path.basename(module)}. Check if  the respective module is available in the genomix server. Throws {e}"
                )

    def _unload_modules(self) -> None:
        """Unload all pocolib modules"""

        assert os.path.isdir(LIB_PATH)
        modules = glob.glob(f"{LIB_PATH}/genom/*/plugins/*.so")

        for module in modules:
            logger.debug(f"Unloading module {os.path.basename(module)}")
            try:
                self.handle.unload(module)
            except Exception:
                pass  # Unloading is an execption

    def _connect_optitrack(self) -> Optitrack:
        """Connect to Optitrack and load all pocolib modules"""
        return Optitrack(self.components["optitrack"], params=self.params)()

    def _connect_pom(self) -> POM:
        """Connect to POM and load all pocolib modules"""
        return POM(self.components["pom"], params=self.params)()

    def _connect_maneuver(self) -> Maneuver:
        """Connect to maneuver and load all pocolib modules"""
        return Maneuver(self.components["maneuver"], params=self.params)()

    def _connect_rotorcraft(self) -> RotorCraft:
        """Connect to Rotorcraft and load all pocolib modules"""
        return RotorCraft(self.components["rotorcraft"], params=self.params)()

    def _connect_nhfc(self) -> NHFC:
        """Connect to NHFC and load all pocolib modules"""
        return NHFC(self.components["nhfc"], params=self.params)()

    def _connect_ctdrone(self) -> CTDrone:
        """Connect to CTDrone and load all pocolib modules"""
        return CTDrone(self.components["CT_drone"], params=self.params)()

    def _connect_tf2(self) -> TF2:
        """Connect to TF2 and load all pocolib modules"""
        return TF2(self.components["tf2"], params=self.params)()

    def _connect_arucotag(self) -> ArucoTag:
        """Connect to ArucoTag and load all pocolib modules"""
        return ArucoTag(self.components["arucotag"], params=self.params)()

    def _connect_camgazebo(self) -> CamGazebo:
        """Connect to CamGazebo and load all pocolib modules"""
        return CamGazebo(self.components["camgazebo"], params=self.params)()

    def _connect_camviz(self) -> CamViz:
        """Connect to CamViz and load all pocolib modules"""
        return CamViz(self.components["camviz"], params=self.params)()
