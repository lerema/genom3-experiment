"""UP demo for the drone project"""
import sys

sys.path.append("up")

from up.problem import BatteryChargingProblem
import unified_planning as up
from unified_planning.shortcuts import *


def main():
    """Main function"""

    problem = BatteryChargingProblem().demo_problem()

    with OneshotPlanner(name="tamer") as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action in result.plan.actions:
            print(action)


if __name__ == "__main__":
    main()
