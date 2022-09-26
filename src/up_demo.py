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
    demo.start_execution()

    print("*** End of Execution ***")

    input("Press enter to exit...")
    connector.stop()


if __name__ == "__main__":
    main()
