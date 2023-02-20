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

from drone_api.actions import Actions
from drone_api.connect import Connector
from drone_api.utils import setup_logging

# Setup logging to file
setup_logging(__file__)


def sample_actions(action: Actions):
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
        action_handler = Connector()
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    action_handler.start()
    action = Actions(action_handler.components)

    functions = sample_actions(action)

    for function, kwargs in functions:
        result = function(**kwargs)
        while True:
            if genomix.update() or str(result.status) == "done":
                print(f"Action {function.__name__} completed")
                break

    action_handler.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
