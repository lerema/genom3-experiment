# Copyright 2022 Selvakumar H S, LAAS-CNRS
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

    print("Current version is not compatible with multiple drones.")
    return

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
