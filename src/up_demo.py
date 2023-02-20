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
import matplotlib.pyplot as plt
import networkx as nx
import unified_planning as up
from unified_planning.shortcuts import (EndTiming, Not, OneshotPlanner,
                                        Problem, StartTiming)
from up_bridge import Bridge
from up_bridge.plexmo import PlanDispatcher

from drone_api.actions import Actions
from drone_api.connect import Connector
from drone_api.up import *


class ProblemDefinition:
    """Problem Setup"""

    def __init__(self) -> None:
        self._bridge = Bridge()
        self._drone_1 = Connector(id=0)
        self._action_1 = Actions(self._drone_1.components, robot_id=0)

    @property
    def bridge(self):
        return self._bridge

    def setup_problem(self):
        problem = Problem()

        self._bridge.create_types([Location, Area, Robot])

        # Fluent definitions
        f_robot_at = self._bridge.create_fluent_from_function(robot_at)
        f_is_surveyed = self._bridge.create_fluent_from_function(is_surveyed)
        f_has_plates = self._bridge.create_fluent_from_function(has_plates)
        f_is_base_station = self._bridge.create_fluent_from_function(is_base_station)

        # Default objects
        base = Location("base_station", z=1.0)
        base_station = self._bridge.create_object("base_station", base)
        r = Robot("drone_1", actions=self._action_1)
        robot = self._bridge.create_object("robot_1", r)
        charge = Location("charging_station", x=1.0, z=1.0)
        charging_station = self._bridge.create_object("charging_station", charge)
        first_area = Area("area_1", survey_size=4.0)
        area = self._bridge.create_object("area", first_area)

        # Action definitions
        survey, [r, _, l] = self._bridge.create_action(
            "survey",
            _callable=Survey,
            robot=Robot,
            area=Area,
            l_from=Location,
            duration=10,
        )
        survey.add_condition(StartTiming(), Not(f_is_surveyed()))
        survey.add_condition(StartTiming(), f_is_base_station(r, l))
        survey.add_condition(StartTiming(), Not(f_has_plates()))
        survey.add_effect(EndTiming(), f_is_surveyed(), True)

        gather_info, _ = self._bridge.create_action(
            "gather_info", _callable=GatherInfo, robot=Robot, duration=3
        )
        gather_info.add_condition(StartTiming(), f_is_surveyed())
        gather_info.add_condition(StartTiming(), Not(f_has_plates()))
        gather_info.add_effect(EndTiming(), f_has_plates(), True)

        move, [r, l_from, l_to] = self._bridge.create_action(
            "move",
            _callable=Move,
            robot=Robot,
            l_from=Location,
            l_to=Location,
            duration=10,
        )
        move.add_condition(StartTiming(), f_robot_at(r, l_from))
        move.add_condition(StartTiming(), Not(f_robot_at(r, l_to)))
        move.add_condition(StartTiming(), f_has_plates())
        move.add_effect(EndTiming(), f_robot_at(r, l_to), True)
        move.add_effect(StartTiming(), f_robot_at(r, l_from), False)

        # Problem definition
        problem.add_fluent(f_robot_at, default_initial_value=False)
        problem.add_fluent(f_is_surveyed, default_initial_value=False)
        problem.add_fluent(f_has_plates, default_initial_value=False)
        problem.add_fluent(f_is_base_station, default_initial_value=False)

        problem.add_objects([robot, base_station, charging_station, area])

        problem.add_actions([survey, gather_info, move])

        problem.set_initial_value(f_robot_at(robot, base_station), True)
        problem.set_initial_value(f_is_base_station(robot, base_station), True)

        problem.add_goal(f_is_surveyed())
        problem.add_goal(f_has_plates())
        problem.add_goal(f_robot_at(robot, charging_station))

        return problem


def main():
    """Main function"""

    # Get problem definition
    problem_def = ProblemDefinition()
    problem = problem_def.setup_problem()
    bridge = problem_def.bridge

    print("*** Planning ***")
    with OneshotPlanner(
        name="aries",
        optimality_guarantee=up.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY,
    ) as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action_instance in result.plan.timed_actions:
            print(action_instance)
        print("*** End of result ***")
        plan = result.plan

    executable_graph = bridge.get_executable_graph(plan)
    # draw graph
    plt.figure(figsize=(10, 10))

    pos = nx.nx_pydot.pydot_layout(executable_graph, prog="dot")
    nx.draw(
        executable_graph,
        pos,
        with_labels=True,
        node_size=1000,
        node_color="skyblue",
        font_size=20,
    )
    plt.show()

    # Execute plan
    dispatcher = PlanDispatcher()
    dispatcher.execute_plan(executable_graph)

    input("Press enter to exit...")


if __name__ == "__main__":
    main()
