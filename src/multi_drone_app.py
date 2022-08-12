from drone_api import Connector
from drone_api.actions import *


def main():
    """Main function"""

    try:
        c1 = Connector(id=1)
        c2 = Connector(id=2)
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    c1.start()
    c2.start()

    # Start actions
    t = Takeoff(c1.components)(height=0.5)
    t = Takeoff(c2.components)(height=0.5)

    m = Move(c1.components)(x=0.5, y=0.5, z=0.5, yaw=0.0)
    m = Move(c2.components)(x=1.5, y=1.5, z=0.5, yaw=0.0)

    m = Move(c1.components)(x=-0.5, y=-0.5, z=0.5, yaw=0.0)
    m = Move(c2.components)(x=0.5, y=0.5, z=0.5, yaw=0.0)

    m = Move(c1.components)(x=0.0, y=0.0, z=0.5, yaw=0.0)
    m = Move(c2.components)(x=1.0, y=1.0, z=0.5, yaw=0.0)

    l = Land(c1.components)()
    l = Land(c2.components)()

    c1.stop()
    c2.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
