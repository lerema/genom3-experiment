from drone_api.connect import Connector
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
    a1 = Actions(c1.components)
    a2 = Actions(c2.components)

    # Start actions
    t = a1.takeoff(height=0.5)
    t = a2.takeoff(height=0.5)

    m = a1.move(area={}, l_from={}, l_to={"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})
    m = a2.move(area={}, l_from={}, l_to={"x": 1.5, "y": 1.5, "z": 0.5, "yaw": 0.0})

    m = a1.move(area={}, l_from={}, l_to={"x": -0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})
    m = a2.move(area={}, l_from={}, l_to={"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})

    m = a1.move(area={}, l_from={}, l_to={"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0})
    m = a2.move(area={}, l_from={}, l_to={"x": 1.0, "y": 1.0, "z": 0.5, "yaw": 0.0})

    l = a1.land()
    l = a2.land()

    c1.stop()
    c2.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
