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

import multiprocessing
from drone_api.connect import Connector
from drone_api.actions import *


class Drone:
    def __init__(self, id):
        self.id = id

    def __call__(self):
        try:
            self.c = Connector(id=self.id)
        except Exception as e:
            raise Exception("Failed to connect to the drone") from e

        self.c.start()
        self.action = Actions(self.c.components)

        t = Takeoff(self.c.components)(height=0.5)
        m = Move(self.c.components)(
            area={},
            l_from={},
            l_to={
                "x": self.id + 0.5,
                "y": self.id + 0.5,
                "z": self.id + 0.5,
                "yaw": 0.0,
            },
        )
        m = Move(self.c.components)(
            area={},
            l_from={},
            l_to={
                "x": -self.id + 0.5,
                "y": -self.id + 0.5,
                "z": self.id + 0.5,
                "yaw": 0.0,
            },
        )
        m = Move(self.c.components)(
            area={}, l_from={}, l_to={"x": self.id, "y": self.id, "z": 0.5, "yaw": 0.0}
        )
        l = Land(self.c.components)()

        self.c.stop()


def main():
    """Main function"""

    d1 = Drone(1)
    d2 = Drone(2)

    thread_1 = multiprocessing.Process(target=d1)
    thread_2 = multiprocessing.Process(target=d2)
    thread_1.start()
    thread_2.start()

    # wait until keypress
    input("Press Enter to exit...")
    thread_1.terminate()
    thread_2.terminate()


if __name__ == "__main__":
    main()
