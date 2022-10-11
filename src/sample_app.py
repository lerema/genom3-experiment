# Copyright 2022 Selvakumar H S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from drone_api.connect import Connector
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
    m = action.move(area={}, l_from={}, l_to={"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})
    m = action.move(
        area={}, l_from={}, l_to={"x": 0.5, "y": -0.5, "z": 0.5, "yaw": 0.0}
    )
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
    m = action.move(area={}, l_from={}, l_to={"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0})
    l = action.land()
    c.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
