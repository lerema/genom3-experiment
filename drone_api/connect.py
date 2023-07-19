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
import time
from types import SimpleNamespace

import genomix

from drone_api import MODULES, USE_ROBOT
from drone_api.genom3 import (
    NHFC,
    POM,
    TF2,
    ArucoTag,
    CamGazebo,
    CamViz,
    CTDrone,
    D435Camera,
    Maneuver,
    Optitrack,
    RotorCraft,
)
from drone_api.params import DroneCommon

USE_CAM = True

LIB_PATH = os.environ["DRONE_VV_PATH"] + "/lib"

logger = logging.getLogger("[Drone Connector]")
logger.setLevel(logging.DEBUG)

from drone_api.utils import setup_logging

def logging_setup():
    """Setup logging"""
    setup_logging("drone_api")
    logger.info("Logging setup complete")

class Connector:
    """Connect to genomix server, load and setup all pocolib modules"""

    def __init__(
        self, drone_id: int = 0, host: str = "localhost", port: int = 8080
    ) -> None:
        self.id = drone_id
        self.params = DroneCommon()(drone_id, is_robot=USE_ROBOT)
        self.components = {}

        # Connect to genomix server
        try:
            self.handle = genomix.connect(f"{host}:{port}")
            assert self.handle is not None
        except genomix.event.GenoMError as exception:
            raise ConnectionError(f"Failed to connect to genomix server. Check if `genomixd` is running. Throws {exception}")

        # Load all modules
        self._load_modules()
        if not set(MODULES["expected"]).issubset({*self.components}):
            raise ModuleNotFoundError(
                f"Failed to find all expected modules. Missing modules: {set(MODULES['expected']) - {*self.components}}"
            )
        
        # Connect Genom instances to python modules
        self.connectors = {
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
        for component in MODULES["common"]:
            self.components[component] = self.connectors[component]()
            time.sleep(1) # Time to let modules start

        for component in MODULES["expected"]:
            self.components[component] = self.connectors[component]()
            time.sleep(1) # Time to let modules start

    def setup(self):
        """Start the drone. 
        
        Python version of TCL's `setup` command
        """
        self.components["rotorcraft"].start()
        time.sleep(1)
        self.components["nhfc"].start()
        time.sleep(1)
        self.components["maneuver"].start()
        time.sleep(1)
    
    def start(self):
        """Start the drone. Starts the rotorcraft component."""
        self.components["rotorcraft"].start()
        time.sleep(1)

    def stop(self):
        """Stop the drone. Stops the rotorcraft component."""
        self.components["rotorcraft"].stop()
    
    def kill(self):
        """Kill all genom3 modules"""
        for component in self.components.values():
            try:
                component.kill()
            except AttributeError as exception:
                logger.error(f"Failed to kill component {component}. Throws {exception}")
    
    def __del__(self):
        self.kill()

    def _load_modules(self) -> None:
        """Load all pocolib modules"""

        assert os.path.isdir(LIB_PATH)
        modules = glob.glob(f"{LIB_PATH}/genom/*/plugins/*.so")
        try:
            modules.extend(glob.glob("/opt/openrobots/lib/genom/*/plugins/*.so"))
        except Exception as e:
            raise ModuleNotFoundError(e)

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

    def _connect_d435(self) -> D435Camera:
        """Connect to D435 and load all pocolib modules"""
        return D435Camera(self.components["d435"], params=self.params)()

    def get_components(self):
        # Make the components as simplenamespace for attribute access
        return SimpleNamespace(**self.components)
    
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
        import scipy.io as sio
        import datetime
        
        date = datetime.datetime.now().strftime("%Y_%m_%d")
        
        # Save as Matlab file
        file_name = os.path.join(os.environ["DRONE_VV_PATH"], "genom3-experiment/calibrations" f"{date}_lerema.mat")
        sio.savemat(file_name, self.components["rotorcraft"].component.get_imu_calibration())
        
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