from drone_api import Connector
from drone_api.actions import *


def main():
    """Main function"""

    try:
        c = Connector()
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    # c.start()
    # FIXME: This is a hack to get the drone to take off.
    input("Press Enter to continue...")
    # Currently the takeoff and startup scripts are still handled by TCL.

    # Start actions
    m = Move(c.components)
    l = Land(c.components)
    t = Takeoff(c.components)

    # Demo actions
    t(height=0.5, duration=0)
    m(x=0.5, y=0.5, z=2, yaw=0.0)
    m(x=1.0, y=0.5, z=1, yaw=0.0)
    m(x=1.0, y=1.0, z=1, yaw=0.0)
    m(x=0.0, y=0.0, z=1, yaw=0.0)
    l()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
