import os
import logging

from drone_api import USE_ROBOT

logger = logging.getLogger("[Rotorcraft]")


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
