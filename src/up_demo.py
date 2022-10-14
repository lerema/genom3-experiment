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

"""UP demo for the drone project"""
from drone_api.actions import *
from drone_api.connect import Connector
from unified_planning.shortcuts import *
from up_bridge import Bridge, VerifyStationProblem


def main():
    """Main function"""

    connector = Connector()
    bridge = Bridge()
    demo = VerifyStationProblem(bridge)
    problem = demo.get_problem()
    plan = None
    actions = []

    connector.start()

    print("*** Planning ***")
    with OneshotPlanner(name="aries") as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action_instance in result.plan.timed_actions:
            print(action_instance)
            actions.append(action_instance[1])
        print("*** End of result ***")
        plan = result.plan

    print("*** Executing plan ***")
    demo.start_execution(actions, components=connector.components)

    print("*** End of Execution ***")

    input("Press enter to exit...")
    connector.stop()


if __name__ == "__main__":
    main()
