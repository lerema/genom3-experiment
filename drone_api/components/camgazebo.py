import logging

logger = logging.getLogger("[CamGazebo]")


class CamGazebo:
    def __init__(self, component, params):
        """Connect to Gazebo through Camgazebo pocolibs and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """Camgazebo component"""

        try:

            self.component.set_hfov(self.params["hfov"])
            self.component.set_format(
                self.params["x_resolution"], self.params["y_resolution"]
            )
            self.component.connect(self.params["port"])
            self.component.set_extrinsics(self.params["extrinsics"])
        except Exception as e:
            logger.error(f"Failed to connect to CamGazebo. Throws {e}")
            raise e
        finally:
            logger.info("Connected to CamGazebo")

    def stop(self):
        """Stop the CamGazebo component"""
        self.component.stop()

    def __str__(self):
        return "camgazebo"
