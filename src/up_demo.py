"""UP demo for the drone project"""
import sys
import time

sys.path.append("up")

from drone_api import Connector
from drone_api.actions import *
from unified_planning.shortcuts import *

from up.problem import VerifyStationProblem


class UPBridge:
    """Connect UP components to the drone API components"""

    fluents = {}
    objects = {
        "l1": {"x": 3.5, "y": 3.5, "z": 1.0, "yaw": 0.0},
        "l2": {"x": -2.5, "y": 1.5, "z": 1.0, "yaw": 0.0},
        "l3": {"x": 1.5, "y": -2.5, "z": 1.0, "yaw": 0.0},
        "l4": {"x": -1.5, "y": -3.5, "z": 1.0, "yaw": 0.0},
        "home": {"x": 0.0, "y": 0.0, "z": 0.15, "yaw": 0.0},
    }
    actions = {
        "move": Move,
        "land": Land,
        "stop": Stop,
        "takeoff": Takeoff,
        "capture_photo": None,  # TODO: implement this action
    }


def main():
    """Main function"""

    connector = Connector()
    problem = VerifyStationProblem().demo_problem()
    plan = None
    bridge = UPBridge()

    with OneshotPlanner(name="tamer") as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action_instance in result.plan.actions:
            print(action_instance)
        print("*** End of result ***")
        plan = result.plan

    # Execute plan
    connector.start()

    print("*** Executing plan ***")
    for action_instance in plan.actions:
        action_name = action_instance.action.name
        drone_action = bridge.actions[str(action_name)]
        parameter = action_instance.actual_parameters
        # TODO: Implement in a better way
        if str(action_name) == "move":
            result = drone_action(connector.components)(
                **bridge.objects[str(parameter[-1])]
            )
        elif str(action_name) == "capture_photo":
            result = Move(connector.components)(**bridge.objects[str(parameter[-1])])
            result = Land(connector.components)()
            result = Takeoff(connector.components)()
            print("*** Capturing photo ***")
        time.sleep(1)
    print("*** End of Execution ***")

    input("Press enter to exit...")
    connector.stop()


if __name__ == "__main__":
    main()
