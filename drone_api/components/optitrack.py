import logging

logger = logging.getLogger("[Optittrack]")


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
