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

"""Create a set of problems for the unified planning domain."""
import unified_planning as up
from unified_planning.shortcuts import *


def demo_problem():
    """Create a simple station verification application"""
    Location = UserType("Location")
    Area = UserType("Area")

    # Fluent definitions
    robot_at = Fluent("robot_at", BoolType(), position=Location)
    is_surveyed = Fluent("is_surveyed", BoolType())
    has_plates = Fluent("has_plates", BoolType())
    is_distance_optimized = Fluent("is_distance_optimized", BoolType())
    is_base_station = Fluent("is_base_station", BoolType(), position=Location)

    # Default objects
    base_station = Object("base_station", Location)
    charging_station = Object("charging_station", Location)
    area = Object("area", Area)

    survey = DurativeAction("survey", area=Area, From=Location)
    l_from = survey.parameter("From")
    survey.set_fixed_duration(10)  # TODO: make this a variable
    survey.add_condition(StartTiming(), Not(is_surveyed()))
    survey.add_condition(StartTiming(), is_base_station(l_from))
    survey.add_condition(StartTiming(), Not(has_plates()))
    survey.add_effect(EndTiming(), is_surveyed(), True)

    gather_info = InstantaneousAction("gather_info")
    gather_info.add_precondition(is_surveyed())
    gather_info.add_precondition(Not(has_plates()))
    gather_info.add_effect(has_plates(), True)

    move = DurativeAction("move", l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    move.set_fixed_duration(1)  # TODO: make this a variable
    move.add_condition(StartTiming(), robot_at(l_from))
    move.add_condition(StartTiming(), Not(robot_at(l_to)))
    move.add_condition(StartTiming(), has_plates())
    move.add_condition(StartTiming(), is_distance_optimized())
    move.add_effect(StartTiming(), robot_at(l_from), False)
    move.add_effect(EndTiming(), robot_at(l_to), True)

    acquire_plan_order = InstantaneousAction("acquire_plan_order", location=Location)
    location = acquire_plan_order.parameter("location")
    acquire_plan_order.add_precondition(is_base_station(location))
    acquire_plan_order.add_precondition(has_plates())
    acquire_plan_order.add_effect(is_distance_optimized(), True)

    problem = Problem()

    problem.add_fluent(is_surveyed, default_initial_value=False)
    problem.add_fluent(has_plates, default_initial_value=False)
    problem.add_fluent(is_distance_optimized, default_initial_value=False)
    problem.add_fluent(robot_at, default_initial_value=False)
    problem.add_fluent(is_base_station, default_initial_value=False)

    problem.add_objects([base_station, charging_station, area])

    problem.add_actions([survey, gather_info, move, acquire_plan_order])
    problem.set_initial_value(robot_at(base_station), True)
    problem.set_initial_value(is_base_station(base_station), True)

    problem.add_goal(is_surveyed())
    problem.add_goal(has_plates())
    problem.add_goal(is_distance_optimized())
    problem.add_goal(robot_at(charging_station))

    return problem


problem = demo_problem()
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
