from drone_api import Connector
from drone_api.actions import *


def main():
    """Main function"""

    try:
        c = Connector()
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    c.start()
    # FIXME: This is a hack to get the drone to take off.
    # input("Press Enter to continue...")
    # Currently the takeoff and startup scripts are still handled by TCL.

    # Start actions
    t = Takeoff(c.components)(height=0.5)
    m = Move(c.components)(x=0.5, y=0.5, z=0.5, yaw=0.0)
    m = Move(c.components)(x=-0.5, y=-0.5, z=0.5, yaw=0.0)
    m = Move(c.components)(x=0.0, y=0.0, z=0.5, yaw=0.0)
    l = Land(c.components)()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
