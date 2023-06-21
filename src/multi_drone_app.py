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

import genomix
from drone_api.actions import Actions
from drone_api.connect import Connector
from drone_api.utils import setup_logging

# Setup logging to file
setup_logging(__file__)


class Drone:
    def __init__(self, drone_id):
        self._id = drone_id

    def __call__(self):
        """Test run of the available actions."""
        drone = Connector(drone_id=self._id)
        drone.start()
        action = Actions(drone.components)

        functions = self.action_functions(action)

        for function, parameters in functions:
            result = function(**parameters)
            while True:
                if genomix.update() or str(result.status) == "done":
                    print(f"Action {function.__name__} completed")
                    break

    def action_functions(self, action):
        """Test run of the available actions."""
        # Start actions
        function_map = []
        function_map.append((action.takeoff, {"height": 0.5}))
        function_map.append(
            (
                action.move,
                {
                    "l_from": {},
                    "l_to": {"x": self._id + 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0},
                },
            )
        )
        function_map.append(
            (
                action.survey,
                {
                    "area": {
                        "xmin": self._id + -2.0,
                        "xmax": self._id + 2.0,
                        "ymin": -2.0,
                        "ymax": 2.0,
                        "z": 1.5,
                        "yaw": 0.0,
                    }
                },
            )
        )
        function_map.append(
            (
                action.move,
                {"l_from": {}, "l_to": {"x": self._id, "y": 0.0, "z": 0.5, "yaw": 0.0}},
            )
        )
        function_map.append((action.land, {}))

        return function_map


def main():
    """Main function"""

    drone_1 = Drone(drone_id=1)
    drone_2 = Drone(drone_id=2)

    thread_1 = multiprocessing.Process(target=drone_1)
    thread_2 = multiprocessing.Process(target=drone_2)
    thread_1.start()
    thread_2.start()

    # wait until keypress
    input("Press Enter to exit...")
    thread_1.terminate()
    thread_2.terminate()


if __name__ == "__main__":
    main()
