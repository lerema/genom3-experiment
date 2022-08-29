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
        "l1": {"x": 0.0, "y": 0.0, "z": 0.15, "yaw": 0.0},
        "l2": {"x": -2.5, "y": 1.5, "z": 1.0, "yaw": 0.0},
        "l3": {"x": 1.5, "y": -2.5, "z": 1.0, "yaw": 0.0},
        "l4": {"x": -1.5, "y": -3.5, "z": 1.0, "yaw": 0.0},
        "l5": {"x": 3.5, "y": 3.5, "z": 1.0, "yaw": 0.0},
        "area": {
            "xmin": -5.0,
            "xmax": 5.0,
            "ymin": -5.0,
            "ymax": 5.0,
            "z": 3.0,
            "yaw": 0.0,
        },
    }
    actions = {
        "move": Move,
        "land": Land,
        "stop": Stop,
        "takeoff": Takeoff,
        "capture_photo": None,  # TODO: implement this action
        "survey": SurveyX,
        "send_info": None,  # TODO: implement this action
    }


def main():
    """Main function"""

    connector = Connector()
    problem = VerifyStationProblem().demo_problem()
    plan = None
    bridge = UPBridge()
    action = Actions(connector.components)

    connector.start()

    print("*** Planning ***")
    with OneshotPlanner(name="aries") as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action_instance in result.plan.timed_actions:
            print(action_instance)
        print("*** End of result ***")
        plan = result.plan

    print("*** Executing plan ***")
    result = action.move(l_from=None, l_to=bridge.objects["l1"])
    for timed_action in plan.timed_actions:
        action_instance = timed_action[1]
        print(action_instance)
        action_name = action_instance.action.name
        drone_action = bridge.actions[str(action_name)]
        parameter = action_instance.actual_parameters
        # TODO: Implement in a better way
        if str(action_name) == "move":
            result = drone_action(connector.components)(
                l_from=None, l_to=bridge.objects[str(parameter[0])]
            )
        elif str(action_name) == "capture_photo":
            parameter = bridge.objects[str(parameter[0])]
            parameter["z"] = 0.15
            result = action.move(l_from=None, l_to=parameter)
            time.sleep(3)
            result = action.takeoff(height=1.0)
        elif str(action_name) == "send_info":
            parameter = bridge.objects[str(parameter[-1])]
            print(f"Sending info to {parameter}")
        elif str(action_name) == "survey":
            parameter = bridge.objects[str(parameter[0])]
            result = action.surveyx(**bridge.objects[str(parameter[0])])
        time.sleep(1)
    result = action.move(l_from=None, l_to=bridge.objects["l1"])
    print("*** End of Execution ***")

    input("Press enter to exit...")
    connector.stop()


if __name__ == "__main__":
    main()
