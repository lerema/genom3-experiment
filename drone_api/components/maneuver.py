import logging
from drone_api import USE_ROBOT

logger = logging.getLogger("[Maneuver]")


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
