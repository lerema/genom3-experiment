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
    action = Actions(c.components)

    # Start actions
    t = action.takeoff(height=0.5)
    m = action.move(l_from=None, l_to={"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})
    m = action.move(l_from=None, l_to={"x": 0.5, "y": -0.5, "z": 0.5, "yaw": 0.0})
    # s = action.surveyy(xmin=-5.0, xmax=5.0, ymin=-2.0, ymax=2.0, z=3.0, yaw=0.5)
    s = action.surveyx(
        area={
            "xmin": -5.0,
            "xmax": 5.0,
            "ymin": -2.0,
            "ymax": 2.0,
            "z": 3.0,
            "yaw": 0.5,
        }
    )
    m = action.move(l_from=None, l_to={"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0})
    l = action.land()
    c.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
