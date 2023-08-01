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
import sys
import time
from types import SimpleNamespace
from typing import Dict, NamedTuple, List

import genomix
from drone_api import MODULES, USE_ROBOT
from drone_api.genom3 import *  # noqa: F401, F403 # pylint: disable=unused-wildcard-import
from drone_api.params import DroneCommon
from drone_api.utils import setup_logging


logger = logging.getLogger("[Drone Connector]")


def logging_setup():
    """Setup logging"""
    setup_logging("drone_api", logging.DEBUG)
    logger.info("Logging setup complete")


# NamedTuple to store the component pairs (python_handle, genomix_handle)
GenomixComponent = NamedTuple(
    "GenomixComponent", [("python", object), ("genomix", object)]
)


class Connector:
    """Connect to genomix server, load and setup all pocolib modules"""

    def __init__(
        self, drone_id: int = 0, host: str = "localhost", port: int = 8080
    ) -> None:
        self.drone_id: int = drone_id
        self.params: Dict[str, any] = DroneCommon()(drone_id, is_robot=USE_ROBOT)
        self.components: Dict[str, GenomixComponent] = {}

        # Connect to genomix server
        self.handle = genomix.connect(f"{host}:{port}")
        assert self.handle is not None, "Failed to connect to genomix server"

        # Load all modules
        self._modules: List[str] = self._load_modules()
        if not set(MODULES["expected"]).issubset({*self.components}):
            raise ModuleNotFoundError(
                f"Failed to find/load all expected modules. \
                    Missing modules: {set(MODULES['expected']) - {*self.components}}"
            )

        # Connect Genom handles to python instances
        connectors = {
            "optitrack": self._connect_optitrack,
            "pom": self._connect_pom,
            "maneuver": self._connect_maneuver,
            "rotorcraft": self._connect_rotorcraft,
            "nhfc": self._connect_nhfc,
            "CT_drone": self._connect_ctdrone,
            "tf2": self._connect_tf2,
            "arucotag": self._connect_arucotag,
            "d435": self._connect_d435,
            "camgazebo": self._connect_camgazebo,
            "camviz": self._connect_camviz,
        }

        # Better to start with common modules first
        modules = list(MODULES["common"]) + list(MODULES["dedicated"])
        for module in modules:
            self.components[module] = connectors[module]()
            time.sleep(1)  # Hack to let modules start

    def setup(self):
        """Start the drone.

        Python version of TCL's `setup` command
        """
        self.components["rotorcraft"].python.start()
        time.sleep(1)
        self.components["nhfc"].python.start()
        time.sleep(1)
        self.components["maneuver"].python.start()
        time.sleep(1)

    def start(self):
        """Start the drone. Starts the rotorcraft component."""
        self.components["rotorcraft"].python.start()
        time.sleep(1)

    def stop(self):
        """Stop the drone. Stops the rotorcraft component."""
        self.components["rotorcraft"].python.stop()

    def kill(self):
        """Kill all genom3 modules"""
        for component in self.components.values():
            component.python.kill()

    def __del__(self):
        self.kill()

    def _load_modules(self) -> List[str]:
        """Load all pocolib modules"""
        lib_dir = os.environ["DRONE_VV_PATH"] + "/lib"

        assert os.path.isdir(lib_dir)
        modules = glob.glob(f"{lib_dir}/genom/*/plugins/*.so")
        modules.extend(glob.glob("/opt/openrobots/lib/genom/*/plugins/*.so"))

        if self.drone_id == 0:
            drone_id = ""
        else:
            drone_id = str(self.drone_id)

        for module in modules:
            logger.debug(f"Loading module {os.path.basename(module)}")
            tag = os.path.basename(module).split(".")[0]
            try:
                handle = None
                if tag in MODULES["dedicated"]:
                    handle = self.handle.load(module, "-i", f"{tag}{drone_id}")
                elif tag in MODULES["common"]:
                    handle = self.handle.load(module)
                self.components[tag] = GenomixComponent(None, handle)
            except Exception as exception:
                logger.error(
                    f"Failed to load module {os.path.basename(module)}. \
                        Check if  the respective module is available in the genomix server. \
                            Throws {exception}"
                )
                sys.exit(1)

        return modules

    def _unload_modules(self) -> None:
        """Unload all pocolib modules"""
        for module in self._modules:
            logger.debug(f"Unloading module {os.path.basename(module)}")
            try:
                self.handle.unload(module)
            except Exception:
                pass  # Unloading is an execption

    def _connect_optitrack(self) -> Optitrack:
        """Connect to Optitrack and load all pocolib modules"""
        python = Optitrack(self.components["optitrack"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["optitrack"].genomix)

    def _connect_pom(self) -> POM:
        """Connect to POM and load all pocolib modules"""
        python = POM(self.components["pom"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["pom"].genomix)

    def _connect_maneuver(self) -> Maneuver:
        """Connect to maneuver and load all pocolib modules"""
        python = Maneuver(self.components["maneuver"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["maneuver"].genomix)

    def _connect_rotorcraft(self) -> RotorCraft:
        """Connect to Rotorcraft and load all pocolib modules"""
        python = RotorCraft(self.components["rotorcraft"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["rotorcraft"].genomix)

    def _connect_nhfc(self) -> NHFC:
        """Connect to NHFC and load all pocolib modules"""
        python = NHFC(self.components["nhfc"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["nhfc"].genomix)

    def _connect_ctdrone(self) -> CTDrone:
        """Connect to CTDrone and load all pocolib modules"""
        python = CTDrone(self.components["CT_drone"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["CT_drone"].genomix)

    def _connect_tf2(self) -> TF2:
        """Connect to TF2 and load all pocolib modules"""
        python = TF2(self.components["tf2"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["tf2"].genomix)

    def _connect_arucotag(self) -> ArucoTag:
        """Connect to ArucoTag and load all pocolib modules"""
        python = ArucoTag(self.components["arucotag"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["arucotag"].genomix)

    def _connect_camgazebo(self) -> CamGazebo:
        """Connect to CamGazebo and load all pocolib modules"""
        python = CamGazebo(self.components["camgazebo"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["camgazebo"].genomix)

    def _connect_camviz(self) -> CamViz:
        """Connect to CamViz and load all pocolib modules"""
        python = CamViz(self.components["camviz"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["camviz"].genomix)

    def _connect_d435(self) -> D435Camera:
        """Connect to D435 and load all pocolib modules"""
        python = D435Camera(self.components["d435"].genomix, params=self.params)()
        return GenomixComponent(python, self.components["d435"].genomix)

    def get_python_components(self):
        """Get all python components"""
        python_components = {}
        for module, component in self.components.items():
            python_components[module] = component.python
        return SimpleNamespace(**python_components)

    def get_genomix_components(self):
        """Get all genomix components"""
        genomix_components = {}
        for module, component in self.components.items():
            genomix_components[module] = component.genomix

        return SimpleNamespace(**genomix_components)

    def calibrate_drone(self):
        """Calibrate drone if called"""

        if not USE_ROBOT:
            logger.info("Calibrating drone")
            return

        rotorcraft = self.components["rotorcraft"].component
        rotorcraft.set_calibration_param(30)
        rotorcraft.calibrate_imu()

    def save_calibration(self):
        """Save the calibration data"""
        import datetime

        import scipy.io as sio

        date = datetime.datetime.now().strftime("%Y_%m_%d")

        # Save as Matlab file
        file_name = os.path.join(
            os.environ["DRONE_VV_PATH"],
            "genom3-experiment/calibrations" f"{date}_lerema.mat",
        )
        sio.savemat(
            file_name, self.components["rotorcraft"].component.get_imu_calibration()
        )

        logger.info(f"Saved calibration data to {file_name}")

    def set_drone_zero(self):
        """Set drone zero"""

        if not USE_ROBOT:
            logger.info("Setting drone zero")
            return

        rotorcraft = self.components["rotorcraft"].component
        rotorcraft.set_zero()

    @property
    def rotorcraft(self):
        """Return rotorcraft handle"""
        return self.components["rotorcraft"].component

    @property
    def nhfc(self):
        """Return nhfc handle"""
        return self.components["nhfc"].component

    @property
    def maneuver(self):
        """Return maneuver handle"""
        return self.components["maneuver"].component

    @property
    def optitrack(self):
        """Return optitrack handle"""
        return self.components["optitrack"].component

    @property
    def pom(self):
        """Return pom handle"""
        return self.components["pom"].component

    @property
    def tf2(self):
        """Return tf2 handle"""
        return self.components["tf2"].component

    @property
    def arucotag(self):
        """Return arucotag handle"""
        return self.components["arucotag"].component

    @property
    def d435(self):
        """Return d435 handle"""
        return self.components["d435"].component

    @property
    def camgazebo(self):
        """Return camgazebo handle"""
        return self.components["camgazebo"].component

    @property
    def camviz(self):
        """Return camviz handle"""
        return self.components["camviz"].component

    @property
    def ctdrone(self):
        """Return ctdrone handle"""
        return self.components["CT_drone"].component
