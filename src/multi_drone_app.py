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
import genomix
import multiprocessing

from drone_api.actions import Actions
from drone_api.connect import Connector
from drone_api.utils import setup_logging

# Setup logging to file
setup_logging(__file__)


def drone_1_actions(action: Actions):
    """Test run of the available actions."""
    # Start actions
    function_map = []
    function_map.append((action.takeoff, {"height": 0.5}))
    function_map.append(
        (
            action.move,
            {"l_from": {}, "l_to": {"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0}},
        )
    )
    function_map.append(
        (
            action.survey,
            {
                "area": {
                    "xmin": -2.0,
                    "xmax": 2.0,
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
            {"l_from": {}, "l_to": {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0}},
        )
    )
    function_map.append((action.land, {}))

    return function_map


def drone_2_actions(action: Actions):
    """Test run of the available actions."""
    # Start actions
    function_map = []
    function_map.append((action.takeoff, {"height": 0.5}))
    function_map.append(
        (
            action.move,
            {"l_from": {}, "l_to": {"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0}},
        )
    )
    function_map.append(
        (
            action.survey,
            {
                "area": {
                    "xmin": -2.0,
                    "xmax": 2.0,
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
            {"l_from": {}, "l_to": {"x": 0.0, "y": 0.0, "z": 0.5, "yaw": 0.0}},
        )
    )
    function_map.append((action.land, {}))

    return function_map


def main():
    """Main function"""

    try:
        drone_1_action_handler = Connector(id=1)
    except Exception as e:
        raise Exception("Failed to connect to the drone 1") from e
    try:
        drone_2_action_handler = Connector(id=2)
    except Exception as e:
        raise Exception("Failed to connect to the drone 2") from e

    def drone_1():
        # Start the connection and take off
        drone_1_action_handler.start()
        action = Actions(drone_1_action_handler.components)

        functions = drone_1_actions(action)

        for function, kwargs in functions:
            result = function(**kwargs)
            while True:
                if genomix.update() or str(result.status) == "done":
                    print(f"Action {function.__name__} completed")
                    break

    def drone_2():
        # Start the connection and take off
        drone_2_action_handler.start()
        action = Actions(drone_2_action_handler.components)

        functions = drone_2_actions(action)

        for function, kwargs in functions:
            result = function(**kwargs)
            while True:
                if genomix.update() or str(result.status) == "done":
                    print(f"Action {function.__name__} completed")
                    break

    thread_1 = multiprocessing.Process(target=drone_1)
    thread_2 = multiprocessing.Process(target=drone_2)
    thread_1.start()
    thread_2.start()

    # wait until keypress
    drone_1_action_handler.stop()
    drone_2_action_handler.stop()
    input("Press Enter to exit...")

    thread_1.terminate()
    thread_2.terminate()


if __name__ == "__main__":
    main()
