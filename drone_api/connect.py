"""Connect to genomix server and load all pocolib modules"""
import subprocess
import genomix
import os
import logging
import glob


LIB_PATH = os.environ["DRONE_VV_PATH"] + "/lib"

logger = logging.getLogger("Drone Connector")


class Connector:
    def __init__(self, host: str = "localhost", port: int = 8080) -> None:
        # Attempt to start the genomix server
        self.genomix_process = subprocess.Popen(
            ["genomixd", "-d"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )

        self.handle = genomix.connect(f"{host}:{port}")

        assert self.handle is not None
        assert self.genomix_process is not None

        self._load_modules()

    def __del__(self):
        self.genomix_process.kill()

    def _load_modules(self) -> None:
        """Load all pocolib modules"""

        assert os.path.isdir(LIB_PATH)
        modules = glob.glob(f"{LIB_PATH}/genom/*/plugins/*.so")

        for module in modules:
            logger.info(f"Loading module {module}")
            self.handle.load(module)


def main():
    connector = Connector()


if __name__ == "__main__":
    main()
    print("Done")
    exit(0)
