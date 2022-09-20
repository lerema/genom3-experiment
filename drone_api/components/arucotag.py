import logging

logger = logging.getLogger("[ArucoTag]")


class ArucoTag:
    def __init__(self, component, params):
        """Connect to ArucoTag and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """Aruco Tag component"""

        try:

            self.component
        except Exception as e:
            logger.error(f"Failed to connect to ArucoTag. Throws {e}")
            raise e
        finally:
            logger.info("Connected to ArucoTag")

    def stop(self):
        """Stop the ArucoTag component"""
        self.component.stop()
