"""Connect to genomix server, load and setup all pocolib modules"""
import glob
import logging
import os
import subprocess
import time
from drone_api.params import DRONES

import genomix

from drone_api import MODULES
from drone_api.utils import Singleton

USE_ROBOT = False
USE_CAM = True

LIB_PATH = os.environ["DRONE_VV_PATH"] + "/lib"

logger = logging.getLogger("[Drone Connector]")
logger.setLevel(logging.DEBUG)


class Optitrack:
    def __init__(self, component, params) -> None:
        """
        Connect to Optitrack and load all pocolib modules
        """
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """
        Connect to Optitrack and load all pocolib modules
        """
        try:
            logger.info("Connecting to Optitrack")
            self.component.connect(
                {
                    "host": self.params["host"],
                    "host_port": self.params["host_port"],
                    "mcast": self.params["mcast"],
                    "mcast_port": self.params["mcast_port"],
                }
            )
        except Exception as e:
            logger.error(f"Failed to connect to Optitrack. Throws {e}")
            raise e
        finally:
            logger.info("Connected to Optitrack")

        return self

    def start(self):
        """Start the Optitrack component"""
        return

    def stop(self):
        """Stop the Optitrack component"""
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "optitrack"


class POM:
    def __init__(self, component, params) -> None:
        """
        Connect to POM and load all pocolib modules
        """
        self.component = component
        self.params = params[str(self)]
        self.ack = True

    def __call__(self):
        """Connect to component port"""

        try:
            logger.info("Connecting to POM")
            for port in self.params["ports"]:
                self._connect_port(port[0], port[1])

            for measurement in self.params["add_measurements"].keys():
                self._add_measurement(measurement)

            self._set_mag_field(
                x=self.params["set_mag_field"][0],
                y=self.params["set_mag_field"][1],
                z=self.params["set_mag_field"][2],
            )
            self.component.set_history_length(
                {"history_length": self.params["history_length"]}
            )
        except Exception as e:
            logger.error(f"Failed to connect to POM. Throws {e}")
            raise e
        finally:
            logger.info("Connected to POM")

        return self

    def start(self):
        """Start the POM component"""
        return

    def stop(self):
        """Stop the POM component"""
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "pom"

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})

    def _add_measurement(self, port, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        return self.component.add_measurement(
            {
                "port": port,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        )

    def _set_mag_field(self, x=0.0, y=0.0, z=0.0):
        return self.component.set_mag_field({"magdir": {"x": x, "y": y, "z": z}})


class Maneuver:
    def __init__(self, component, params) -> None:
        """
        Connect to POM and load all pocolib modules
        """
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """Connect to component port"""

        try:
            logger.info("Connecting to maneuver")
            self._connect_port(self.params["ports"][0], self.params["ports"][1])
            if not USE_ROBOT:
                self.component.set_velocity_limit(
                    {
                        "v": self.params["set_velocity_limit"][0],
                        "w": self.params["set_velocity_limit"][1],
                    }
                )
        except Exception as e:
            logger.error(f"Failed to connect to maneuver. Throws {e}")
            raise e
        finally:
            logger.info("Connected to maneuver")

        return self

    def start(self):
        """Start the Maneuver component"""
        logger.info("Starting maneuver")
        self.component.set_bounds(
            {
                "xmin": -100,
                "xmax": 100,
                "ymin": -100,
                "ymax": 100,
                "zmin": -2,
                "zmax": 30,
                "yawmin": -10,
                "yawmax": 10,
            }
        )
        self.component.set_current_state()
        self.component.take_off({"height": 0.15, "duration": 0})

    def stop(self):
        """Stop the Maneuver component"""
        self.component.stop()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "maneuver"

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})


class RotorCraft:
    def __init__(self, component, params):
        """Connect to Rotorcraft and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):

        try:
            self._connect(self.params["connect"][0], self.params["connect"][1])
            self._set_sensor_rate(
                self.params["set_sensor_rate"][0],
                self.params["set_sensor_rate"][1],
                self.params["set_sensor_rate"][2],
                self.params["set_sensor_rate"][3],
            )
            if USE_ROBOT:
                self._load_imu_calibration()

            self.component.connect_port(
                {"local": self.params["ports"][0], "remote": self.params["ports"][1]}
            )
        except Exception as e:
            logger.error(f"Failed to connect to Rotorcraft. Throws {e}")
            raise e
        finally:
            logger.info("Connected to Rotorcraft")

        return self

    def start(self):
        """Start the Rotorcraft component"""
        logger.info("Starting Rotorcraft")
        self.component.start()
        self.component.servo(ack=self.ack)

    def stop(self):
        """Stop the Rotorcraft component"""
        self.component.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        )
        self.component.stop()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "rotorcraft"

    def _load_imu_calibration(self):
        # TODO: fix this
        with open(
            os.path.join(os.path.dirname(__file__), "../imu_calib.txt"),
            "r",
            encoding="utf-8",
        ) as f:
            lines = f.readlines()
            for line in lines:
                self.component.set_imu_calibration({"calib": line})

    def _connect(self, serial, baudrate):
        """Connect to Rotorcraft and load all pocolib modules"""
        return self.component.connect({"serial": serial, "baud": baudrate})

    def _set_sensor_rate(self, imu_rate, mag_rate, motor_rate, battery_rate):
        return self.component.set_sensor_rate(
            {
                "rate": {
                    "imu": imu_rate,
                    "mag": mag_rate,
                    "motor": motor_rate,
                    "battery": battery_rate,
                }
            }
        )


class NHFC:
    def __init__(self, component, params):
        """Connect to NHFC and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):

        try:
            self.component.set_gtmrp_geom({})
            self._servo_gain(
                self.params["servo_gain"][0],
                self.params["servo_gain"][1],
                self.params["servo_gain"][2],
                self.params["servo_gain"][3],
                self.params["servo_gain"][4],
                self.params["servo_gain"][5],
                self.params["servo_gain"][6],
                self.params["servo_gain"][7],
                self.params["servo_gain"][8],
                self.params["servo_gain"][9],
            )
            self.component.set_wlimit(
                {"wmin": self.params["wlimit"][0], "wmax": self.params["wlimit"][1]}
            )
            self.component.set_emerg(
                {
                    "emerg": {
                        "descent": self.params["set_emerg"][0],
                        "dx": self.params["set_emerg"][1],
                        "dq": self.params["set_emerg"][2],
                        "dv": self.params["set_emerg"][3],
                        "dw": self.params["set_emerg"][4],
                    }
                }
            )
            for port in self.params["ports"]:
                self.component.connect_port({"local": port[0], "remote": port[1]})
        except Exception as e:
            logger.error(f"Failed to connect to NHFC. Throws {e}")
            raise e
        finally:
            logger.info("Connected to NHFC")

        return self

    def start(self):
        """Start the NHFC component"""
        logger.info("Starting NHFC")
        self.component.set_current_position()
        self.component.servo(ack=self.ack)

    def stop(self):
        """Stop the NHFC component"""
        self.component.stop()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "nhfc"

    def _servo_gain(self, Kpxy, Kpz, Kqxy, Kqz, Kvxy, Kvz, Kwxy, Kwz, Kixy, Kiz):
        return self.component.set_servo_gain(
            {
                "gain": {
                    "Kpxy": Kpxy,
                    "Kpz": Kpz,
                    "Kqxy": Kqxy,
                    "Kqz": Kqz,
                    "Kvxy": Kvxy,
                    "Kvz": Kvz,
                    "Kwxy": Kwxy,
                    "Kwz": Kwz,
                    "Kixy": Kixy,
                    "Kiz": Kiz,
                }
            }
        )

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})


class CTDrone:
    def __init__(self, component, params):
        """Connect to CTDrone and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """CTDrone component"""

        try:
            self.component.Set_my_r(self.params["rgb"][0])
            self.component.Set_my_g(self.params["rgb"][1])
            self.component.Set_my_b(self.params["rgb"][2])
            self.component.Set_my_seuil(self.params["threshold"])
            self.component.connect_port(
                {"local": self.params["ports"][0], "remote": self.params["ports"][1]}
            )
            self.component.SetCameraImageTopicName(self.params["image_topic"])
            self.component.SetCameraInfoTopicName(self.params["image_info_topic"])
        except Exception as e:
            logging.error(f"Failed to connect to CTDrone. Throws {e}")
            raise e
        finally:
            logging.info("Connected to CTDrone")

        return self

    def start(self):
        """Start the CTDrone component"""
        logger.info("Starting CT_Drone")
        self.component.PublishOccupancyGrid(ack=self.ack)

    def stop(self):
        """Stop the CTDrone component"""
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "CT_drone"


class TF2:
    def __init__(self, component, params):
        """Connect to TF2 and load all pocolib modules"""
        self.component = component
        self.params = params[str(self)]
        self.ack = True

    def __call__(self):
        """TF2 component"""

        try:
            for port in self.params["ports"]:
                self.component.connect_port({"local": port[0], "remote": port[1]})
            self.component.Init()
            self._add_dynamic_tf(**self.params["dynamic_tf"])
            self._add_dynamic_pos_tf(**self.params["dynamic_tf_pos"])
            self._add_odometry(self.params["odometry"])

            self.component.AddTwistFromPose(self.params["twist_from_pose"])
            self.component.AddWrenchFromPose(self.params["wrench_from_pose"])
            for tf in self.params["static_transform"]:
                self._publish_static_tf(**tf)
            self.component.AddOccupancyGrid(self.params["occupancy_grid"])
        except Exception as e:
            logging.error(f"Failed to connect to TF2. Throws {e}")
            raise e
        finally:
            logging.info("Connected to TF2")

        return self

    def start(self):
        return

    def stop(self):
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "tf2"

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})

    def _add_odometry(self, name):
        return self.component.AddOdometry({"name": name})

    def _publish_static_tf(self, **kwargs):
        return self.component.PublishStaticTF(kwargs, ack=self.ack)

    def _add_dynamic_tf(self, **kwargs):
        return self.component.AddDynamicTF(kwargs)

    def _add_dynamic_pos_tf(self, **kwargs):
        return self.component.AddDynamicPosTF(kwargs)


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
            raise ModuleNotFoundError(f"Failed to load all expected modules")

        self.components = {
            "optitrack": self._connect_optitrack(),
            "pom": self._connect_pom(),
            "maneuver": self._connect_maneuver(),
            "rotorcraft": self._connect_rotorcraft(),
            "nhfc": self._connect_nhfc(),
            "CT_drone": self._connect_ctdrone(),
            "tf2": self._connect_tf2(),
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
        print("Stopped")

    def __del__(self):
        # self._unload_modules()
        del self.components
        self.genomix_process.kill()
        subprocess.call(["pkill", "gzserver"])
        subprocess.call(["pkill", "gzclient"])
        subprocess.call(["h2", "end"])
        subprocess.call(["pkill", "genomixd"])
        subprocess.call(["pkill", "-f", "\-pocolibs"])

    def _load_modules(self) -> None:
        """Load all pocolib modules"""

        assert os.path.isdir(LIB_PATH)
        modules = glob.glob(f"{LIB_PATH}/genom/*/plugins/*.so")
        if self.id == 0:
            id = ""
        else:
            id = str(self.id)

        for module in modules:
            logger.info(f"Loading module {os.path.basename(module)}")
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
            logger.info(f"Unloading module {os.path.basename(module)}")
            try:
                self.handle.unload(module)
            except Exception as e:
                logger.error(
                    f"Failed to unload module {os.path.basename(module)}. Throws {e}"
                )

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
