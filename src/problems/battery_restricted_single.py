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
from unified_planning.shortcuts import *


def demo_problem():
    """Create a simple station verification application"""
    Location = UserType("Location")
    Area = UserType("Area")
    Robot = UserType("Robot")

    # Fluent definitions
    robot_at = Fluent("robot_at", BoolType(), robot=Robot, position=Location)
    is_surveyed = Fluent("is_surveyed", BoolType())
    has_plates = Fluent("has_plates", BoolType())
    is_distance_optimized = Fluent("is_distance_optimized", BoolType())
    is_base_station = Fluent(
        "is_base_station", BoolType(), robot=Robot, position=Location
    )
    is_location_inspected = Fluent(
        "is_location_inspected", BoolType(), position=Location
    )
    is_plate_inspected = Fluent("is_plate_inspected", BoolType(), location=Location)
    # Default objects
    robot = Object("robot", Robot)
    base_station = Object("base_station", Location)
    charging_station = Object("charging_station", Location)
    area = Object("area", Area)
    l1 = Object("l1", Location)
    l2 = Object("l2", Location)
    l3 = Object("l3", Location)
    l4 = Object("l4", Location)

    survey = InstantaneousAction("survey", robot=Robot, area=Area, From=Location)
    l_from = survey.parameter("From")
    r = survey.parameter("robot")
    survey.add_precondition(Not(is_surveyed()))
    survey.add_precondition(is_base_station(r, l_from))
    survey.add_precondition(Not(has_plates()))
    survey.add_effect(is_surveyed(), True)

    send_info = InstantaneousAction("send_info", robot=Robot)
    send_info.add_precondition(is_surveyed())
    send_info.add_precondition(Not(has_plates()))
    send_info.add_effect(has_plates(), True)

    move = InstantaneousAction("move", robot=Robot, l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    r = move.parameter("robot")
    move.add_precondition(robot_at(r, l_from))
    move.add_precondition(Not(robot_at(r, l_to)))
    move.add_precondition(has_plates())
    move.add_precondition(is_distance_optimized())
    move.add_effect(robot_at(r, l_from), False)
    move.add_effect(robot_at(r, l_to), True)

    acquire_plates_order = InstantaneousAction("acquire_plates_order", robot=Robot)
    acquire_plates_order.add_precondition(has_plates())
    acquire_plates_order.add_precondition(Not(is_distance_optimized()))
    acquire_plates_order.add_effect(is_distance_optimized(), True)

    inspect_plate = InstantaneousAction("inspect_plate", robot=Robot, location=Location)
    r = inspect_plate.parameter("robot")
    l = inspect_plate.parameter("location")
    inspect_plate.add_precondition(robot_at(r, l))
    inspect_plate.add_precondition(has_plates())
    inspect_plate.add_precondition(is_distance_optimized())
    inspect_plate.add_effect(is_location_inspected(l), True)
    inspect_plate.add_effect(is_plate_inspected(l), True)

    problem = Problem()

    problem.add_fluent(is_surveyed, default_initial_value=False)
    problem.add_fluent(has_plates, default_initial_value=False)
    problem.add_fluent(is_distance_optimized, default_initial_value=False)
    problem.add_fluent(robot_at, default_initial_value=False)
    problem.add_fluent(is_base_station, default_initial_value=False)
    problem.add_fluent(is_location_inspected, default_initial_value=False)
    problem.add_fluent(is_plate_inspected, default_initial_value=False)

    problem.add_objects([robot, base_station, charging_station, area, l1, l2, l3, l4])

    problem.add_actions([survey, send_info, move, acquire_plates_order, inspect_plate])
    problem.set_initial_value(robot_at(robot, base_station), True)
    problem.set_initial_value(is_base_station(robot, base_station), True)

    problem.add_goal(is_surveyed())
    problem.add_goal(has_plates())
    problem.add_goal(is_distance_optimized())
    problem.add_goal(is_location_inspected(l1))
    problem.add_goal(is_location_inspected(l2))
    problem.add_goal(is_location_inspected(l3))
    problem.add_goal(is_location_inspected(l4))
    problem.add_goal(is_plate_inspected(l1))
    problem.add_goal(is_plate_inspected(l2))
    problem.add_goal(is_plate_inspected(l3))
    problem.add_goal(is_plate_inspected(l4))
    problem.add_goal(robot_at(robot, charging_station))

    return problem


problem = demo_problem()
print("*** Problem ***")
print(problem.kind)
print("*** Planning ***")
with OneshotPlanner(problem_kind=problem.kind) as planner:
    result = planner.solve(problem)
    print("*** Result ***")
    for action_instance in result.plan.actions:
        print(action_instance)
    print("*** End of result ***")
    plan = result.plan
