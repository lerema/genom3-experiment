"""Connect to genomix server, load and setup all pocolib modules"""
import glob
import logging
import multiprocessing
import os
import subprocess

import genomix

from drone_api import EXPECTED_MODULES
from drone_api.utils import Singleton

USE_ROBOT = False
USE_CAM = True

LIB_PATH = os.environ["DRONE_VV_PATH"] + "/lib"

logger = logging.getLogger("[Drone Connector]")
logger.setLevel(logging.DEBUG)


class Optitrack:
    def __init__(self, component) -> None:
        """
        Connect to Optitrack and load all pocolib modules
        """
        self.component = component
        self.ack = True

    def __call__(self):
        """
        Connect to Optitrack and load all pocolib modules
        """
        try:
            logger.info("Connecting to Optitrack")
            if USE_ROBOT:
                self.component.connect(
                    {
                        "host": "marey",
                        "host_port": "1510",
                        "mcast": "239.192.168.30",
                        "mcast_port": "1511",
                    },
                    ack=self.ack,
                ).wait()
            else:
                self.component.connect(
                    {
                        "host": "localhost",
                        "host_port": "1509",
                        "mcast": "",
                        "mcast_port": "",
                    },
                    ack=self.ack,
                ).wait()
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
        self.component.kill()

    def __str__(self) -> str:
        return "optitrack"


class POM:
    def __init__(self, component) -> None:
        """
        Connect to POM and load all pocolib modules
        """
        self.component = component
        self.ack = True

    def __call__(self):
        """Connect to component port"""

        try:
            logger.info("Connecting to POM")
            self._connect_port("measure/imu", "rotorcraft/imu")
            if USE_ROBOT:
                self._connect_port("measure/mocap", "optitrack/bodies/QR_1")
            else:
                self._connect_port("measure/mag", "rotorcraft/mag")
                self._connect_port("measure/mocap", "optitrack/bodies/QR")

            self._add_measurement(port="imu")
            self._add_measurement(port="mocap")
            if not USE_ROBOT:
                self._add_measurement(port="mag")
                self._set_mag_field(x=23.816e-6, y=-0.41e-6, z=-39.829e-6)
            self.component.set_history_length(
                {"history_length": 0.5}, ack=self.ack
            ).wait()
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
        self.component.kill()

    def __str__(self) -> str:
        return "pom"

    def _connect_port(self, local, remote):
        return self.component.connect_port(
            {
                "local": local,
                "remote": remote,
            },
            ack=self.ack,
        ).wait()

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
            },
            ack=self.ack,
        ).wait()

    def _set_mag_field(self, x=0.0, y=0.0, z=0.0):
        return self.component.set_mag_field(
            {
                "magdir": {
                    "x": x,
                    "y": y,
                    "z": z,
                }
            },
            ack=self.ack,
        ).wait()


class Maneuver:
    def __init__(self, component) -> None:
        """
        Connect to POM and load all pocolib modules
        """
        self.component = component
        self.ack = True

    def __call__(self):
        """Connect to component port"""

        try:
            logger.info("Connecting to maneuver")
            self._connect_port("state", "pom/frame/robot")
            if not USE_ROBOT:
                self.component.set_velocity_limit({"v": 2, "w": 1}, ack=self.ack).wait()
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
            },
            ack=self.ack,
        ).wait()
        self.component.set_current_state(ack=self.ack).wait()
        self.component.take_off(
            {
                "height": 0.15,
                "duration": 0,
            },
            ack=self.ack,
        ).wait()

    def stop(self):
        """Stop the Maneuver component"""
        self.component.stop()

    def __del__(self):
        self.component.kill()

    def __str__(self) -> str:
        return "maneuver"

    def _connect_port(self, local, remote):
        return self.component.connect_port(
            {
                "local": local,
                "remote": remote,
            },
            ack=self.ack,
        ).wait()


class RotorCraft:
    def __init__(self, component):
        """Connect to Rotorcraft and load all pocolib modules"""
        self.component = component
        self.ack = True

    def __call__(self):

        try:
            if USE_ROBOT:
                self._connect("/dev/ttyUSB0", "500000")
                self._set_sensor_rate(1000, 0, 16, 1)
                self._load_imu_calibration()
            else:
                if USE_CAM:
                    self._connect("/tmp/pty-mrsim-quadrotor-cam", "500000")
                else:
                    self._connect("/tmp/pty-mrsim-quadrotor", "500000")

                self._set_sensor_rate(1000, 50, 16, 1)

            self.component.connect_port(
                {"local": "rotor_input", "remote": "nhfc/rotor_input"}, ack=self.ack
            ).wait()
        except Exception as e:
            logger.error(f"Failed to connect to Rotorcraft. Throws {e}")
            raise e
        finally:
            logger.info("Connected to Rotorcraft")

        return self

    def start(self):
        """Start the Rotorcraft component"""
        logger.info("Starting Rotorcraft")
        self.component.start(ack=self.ack).wait()
        self._thread = multiprocessing.Process(target=self.component.servo)
        self._thread.start()

    def stop(self):
        """Stop the Rotorcraft component"""
        self.component.set_velocity(
            {
                "desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
            ack=self.ack,
        ).wait()
        self.component.stop(ack=self.ack).wait()

    def __del__(self):
        self._thread.terminate()
        self.component.kill()

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
        return self.component.connect(
            {
                "serial": serial,
                "baud": baudrate,
            },
            ack=self.ack,
        ).wait()

    def _set_sensor_rate(self, imu_rate, mag_rate, motor_rate, battery_rate):
        return self.component.set_sensor_rate(
            {
                "rate": {
                    "imu": imu_rate,
                    "mag": mag_rate,
                    "motor": motor_rate,
                    "battery": battery_rate,
                }
            },
            ack=self.ack,
        ).wait()


class NHFC:
    def __init__(self, component):
        """Connect to NHFC and load all pocolib modules"""
        self.component = component
        self.ack = True

    def __call__(self):

        try:
            self.component.set_gtmrp_geom({})
            if USE_ROBOT:
                self._servo_gain(20, 25, 3, 0.3, 15, 20, 0.3, 0.03, 0.5, 3)
            else:
                self._servo_gain(20, 20, 3, 0.3, 8, 8, 0.3, 0.03, 0, 0)
                self.component.set_wlimit(
                    {"wmin": 16, "wmax": 100}, ack=self.ack
                ).wait()
            self.component.set_emerg(
                {
                    "emerg": {
                        "descent": 1.2,
                        "dx": 0.1,
                        "dq": 1,
                        "dv": 0.3,
                        "dw": 1,
                    }
                },
                ack=self.ack,
            ).wait()
            self._connect_port("state", "pom/frame/robot")
            self._connect_port("reference", "maneuver/desired")
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
        self._thread = multiprocessing.Process(target=self.component.servo)
        self._thread.start()

    def stop(self):
        """Stop the NHFC component"""
        self.component.stop()

    def __del__(self):
        self._thread.terminate()
        self.component.kill()

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
            },
            ack=self.ack,
        ).wait()

    def _connect_port(self, local, remote):
        return self.component.connect_port(
            {
                "local": local,
                "remote": remote,
            },
            ack=self.ack,
        ).wait()


class CTDrone:
    def __init__(self, component):
        """Connect to CTDrone and load all pocolib modules"""
        self.component = component
        self.ack = True

    def __call__(self):
        """CTDrone component"""

        try:
            self.component.connect_port(
                {
                    "local": "Pose",
                    "remote": "pom/frame/robot",
                },
                ack=self.ack,
            ).wait()
            self.component.SetCameraImageTopicName(
                "/quad1/down_camera_link/down_raw_image", ack=self.ack
            ).wait()
            self.component.SetCameraInfoTopicName(
                "/quad1/down_camera_link/down_camera_info", ack=self.ack
            ).wait()
        except Exception as e:
            logging.error(f"Failed to connect to CTDrone. Throws {e}")
            raise e
        finally:
            logging.info("Connected to CTDrone")

        return self

    def start(self):
        """Start the CTDrone component"""
        logger.info("Starting CT_Drone")
        self._thread = multiprocessing.Process(
            target=self.component.PublishOccupancyGrid
        )
        self._thread.start()

    def stop(self):
        """Stop the CTDrone component"""
        return

    def __del__(self):
        self._thread.terminate()
        self.component.kill()

    def __str__(self) -> str:
        return "CT_drone"


class TF2:
    def __init__(self, component):
        """Connect to TF2 and load all pocolib modules"""
        self.component = component
        self.ack = True

    def __call__(self):
        """TF2 component"""

        try:
            self._connect_port("Poses/drone", "pom/frame/robot")
            self._connect_port("Poses/drone_pos", "pom/frame/robot")
            self._connect_port("OccupancyGrids/og", "CT_drone/OccupancyGrid")
            self.component.Init(ack=self.ack).wait()
            self._publish_static_tf("base", "drone", 0, 0, 0, 0, 0, 0)
            self._add_dynamic_tf("drone", "world", 10, True)
            self._add_dynamic_pos_tf("drone_pos", "world", 10, True)
            self._add_odometry("drone")
            self.component.AddTwistFromPose(
                {
                    "name": "drone",
                    "frame": "drone_pos",
                    "topic": "drone_twist",
                    "ms_period": 10,
                },
                ack=self.ack,
            ).wait()
            self.component.AddWrenchFromPose(
                {
                    "name": "drone",
                    "frame": "drone_pos",
                    "topic": "drone_wrench",
                    "ms_period": 10,
                },
                ack=self.ack,
            ).wait()
            self._publish_static_tf("og", "world", 0, 0, 0, 0, 0, 0)
            self.component.AddOccupancyGrid(
                {
                    "name": "og",
                    "frame": "og",
                    "topic": "og",
                    "ms_period": 50,
                },
                ack=self.ack,
            ).wait()
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
        self.component.kill()

    def __str__(self) -> str:
        return "tf2"

    def _connect_port(self, local, remote):
        return self.component.connect_port(
            {
                "local": local,
                "remote": remote,
            },
            ack=self.ack,
        ).wait()

    def _add_odometry(self, name):
        return self.component.AddOdometry({"name": name}, ack=self.ack).wait()

    def _publish_static_tf(self, name, parent_frame, x, y, z, roll, pitch, yaw):
        return self.component.PublishStaticTF(
            {
                "name": name,
                "parent_frame": parent_frame,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            },
            ack=self.ack,
        ).wait()

    def _add_dynamic_tf(self, name, parent_frame, ms_period, undef_in_orig):
        return self.component.AddDynamicTF(
            {
                "name": name,
                "parent_frame": parent_frame,
                "ms_period": ms_period,
                "undef_in_orig": undef_in_orig,
            },
            ack=self.ack,
        ).wait()

    def _add_dynamic_pos_tf(self, name, parent_frame, ms_period, undef_in_orig):
        return self.component.AddDynamicPosTF(
            {
                "name": name,
                "parent_frame": parent_frame,
                "ms_period": ms_period,
                "undef_in_orig": undef_in_orig,
            },
            ack=self.ack,
        ).wait()


class Connector:
    def __init__(self, host: str = "localhost", port: int = 8080) -> None:
        # Attempt to start the genomix server
        self.genomix_process = subprocess.Popen(
            ["genomixd", "-d"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        self.handle = genomix.connect(f"{host}:{port}")
        self.components = {}

        assert self.handle is not None
        assert self.genomix_process is not None

        self._load_modules()
        if not set(EXPECTED_MODULES).issubset({*self.components}):
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
        for component in self.components.values():
            component.start()

    def stop(self):
        self._thread1.terminate()
        self._thread2.terminate()
        self._thread3.terminate()
        print("Stopped")

    def __del__(self):
        # self._unload_modules()
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

        for module in modules:
            logger.info(f"Loading module {os.path.basename(module)}")
            tag = os.path.basename(module).split(".")[0]
            try:
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
        return Optitrack(self.components["optitrack"])()

    def _connect_pom(self) -> POM:
        """Connect to POM and load all pocolib modules"""
        return POM(self.components["pom"])()

    def _connect_maneuver(self) -> Maneuver:
        """Connect to maneuver and load all pocolib modules"""
        return Maneuver(self.components["maneuver"])()

    def _connect_rotorcraft(self) -> RotorCraft:
        """Connect to Rotorcraft and load all pocolib modules"""
        return RotorCraft(self.components["rotorcraft"])()

    def _connect_nhfc(self) -> NHFC:
        """Connect to NHFC and load all pocolib modules"""
        return NHFC(self.components["nhfc"])()

    def _connect_ctdrone(self) -> CTDrone:
        """Connect to CTDrone and load all pocolib modules"""
        return CTDrone(self.components["CT_drone"])()

    def _connect_tf2(self) -> TF2:
        """Connect to TF2 and load all pocolib modules"""
        return TF2(self.components["tf2"])()
