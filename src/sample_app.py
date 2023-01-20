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
import threading
from drone_api.connect import Connector
from drone_api.actions import *
from drone_api.state import RobotState


def sample_actions(action):
    """Test run of the available actions."""
    # Start actions
    t = action.takeoff(height=0.5)
    m = action.move(area={}, l_from={}, l_to={"x": 0.5, "y": 0.5, "z": 0.5, "yaw": 0.0})
    m = action.move(
        area={}, l_from={}, l_to={"x": 0.5, "y": -0.5, "z": 0.5, "yaw": 0.0}
    )
    s = action.survey(
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


def main():
    """Main function"""

    try:
        action_handler = Connector()
        data_handler = Connector()
    except Exception as e:
        raise Exception("Failed to connect to the drone") from e

    # Start the connection and take off
    action_handler.start()
    action = Actions(action_handler.components)
    robot_state = RobotState(data_handler.components, data_handler.id)

    thread_actions = threading.Thread(target=sample_actions, args=(action,))
    thread_state = threading.Thread(target=robot_state.run)

    thread_actions.start()
    thread_state.start()

    thread_state.join()
    thread_state.join()

    action_handler.stop()

    # wait until keypress
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
