import logging

logger = logging.getLogger("[CTDrone]")


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
