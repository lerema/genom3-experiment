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
from unified_planning.model.htn import *


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
    is_robot_available = Fluent("is_robot_available", BoolType(), robot=Robot)
    is_plate_inspected = Fluent("is_plate_inspected", BoolType(), location=Location)

    # Default objects
    r1 = Object("r1", Robot)
    r2 = Object("r2", Robot)
    base_station_1 = Object("base_station_1", Location)
    base_station_2 = Object("base_station_2", Location)
    charging_station = Object("charging_station", Location)
    area = Object("area", Area)
    l1 = Object("l1", Location)
    l2 = Object("l2", Location)
    l3 = Object("l3", Location)
    l4 = Object("l4", Location)

    send_info = InstantaneousAction("send_info", robot=Robot)
    r = send_info.parameter("robot")
    send_info.add_precondition(is_robot_available(r))
    send_info.add_precondition(is_surveyed())
    send_info.add_precondition(Not(has_plates()))
    send_info.add_effect(has_plates(), True)

    move = DurativeAction("move", robot=Robot, l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    r = move.parameter("robot")
    move.set_fixed_duration(1)  # TODO: make this a variable
    move.add_condition(StartTiming(), is_robot_available(r))
    move.add_condition(StartTiming(), robot_at(r, l_from))
    move.add_condition(StartTiming(), Not(robot_at(r, l_to)))
    move.add_condition(StartTiming(), has_plates())
    move.add_condition(StartTiming(), is_distance_optimized())
    move.add_effect(StartTiming(), robot_at(r, l_from), False)
    move.add_effect(StartTiming(), is_robot_available(r), False)
    move.add_effect(EndTiming(), robot_at(r, l_to), True)
    move.add_effect(EndTiming(), is_robot_available(r), True)

    acquire_plates_order = InstantaneousAction("acquire_plates_order", robot=Robot)
    r = acquire_plates_order.parameter("robot")
    acquire_plates_order.add_precondition(is_robot_available(r))
    acquire_plates_order.add_precondition(has_plates())
    acquire_plates_order.add_precondition(Not(is_distance_optimized()))
    acquire_plates_order.add_effect(is_distance_optimized(), True)

    inspect_plate = DurativeAction("inspect_plate", robot=Robot, location=Location)
    r = inspect_plate.parameter("robot")
    l = inspect_plate.parameter("location")
    inspect_plate.set_fixed_duration(2)
    inspect_plate.add_condition(StartTiming(), is_robot_available(r))
    inspect_plate.add_condition(StartTiming(), robot_at(r, l))
    inspect_plate.add_condition(StartTiming(), has_plates())
    inspect_plate.add_condition(StartTiming(), is_distance_optimized())
    inspect_plate.add_effect(StartTiming(), is_robot_available(r), False)
    inspect_plate.add_effect(EndTiming(), is_location_inspected(l), True)
    inspect_plate.add_effect(EndTiming(), is_plate_inspected(l), True)
    inspect_plate.add_effect(EndTiming(), is_robot_available(r), True)

    # Tasks
    survey_task = Task(
        "survey_task", robot=Robot, area=Area, From=Location, To=Location
    )

    survey_method = Method("survey", robot=Robot, area=Area, From=Location, To=Location)
    l_from = survey_method.parameter("From")
    l_to = survey_method.parameter("To")
    r = survey_method.parameter("robot")
    a = survey_method.parameter("area")
    survey_method.set_task(survey_task, r, a, l_from, l_to)
    survey_method.add_precondition(is_robot_available(r))
    survey_method.add_precondition(Not(is_surveyed()))
    survey_method.add_precondition(is_base_station(r, l_from))
    # survey_method.add_precondition(Not(has_plates()))
    survey_method.add_subtask(move, r, l_from, l_to)

    problem = HierarchicalProblem()

    problem.add_fluent(is_surveyed, default_initial_value=False)
    problem.add_fluent(has_plates, default_initial_value=False)
    problem.add_fluent(is_distance_optimized, default_initial_value=False)
    problem.add_fluent(robot_at, default_initial_value=False)
    problem.add_fluent(is_base_station, default_initial_value=False)
    problem.add_fluent(is_location_inspected, default_initial_value=False)
    problem.add_fluent(is_plate_inspected, default_initial_value=False)
    problem.add_fluent(is_robot_available, default_initial_value=True)

    problem.add_objects(
        [r1, r2, base_station_1, base_station_2, charging_station, area, l1, l2, l3, l4]
    )

    problem.add_actions([send_info, move, acquire_plates_order, inspect_plate])

    problem.add_task(survey_task)
    problem.add_method(survey_method)
    
    problem.task_network.add_subtask(survey_task, r1, area, base_station_1, base_station_1)

    problem.set_initial_value(robot_at(r1, base_station_1), True)
    problem.set_initial_value(is_base_station(r1, base_station_1), True)
    problem.set_initial_value(robot_at(r2, base_station_2), True)
    problem.set_initial_value(is_base_station(r2, base_station_2), True)

    problem.add_goal(is_surveyed())
    # problem.add_goal(has_plates())
    # problem.add_goal(is_distance_optimized())
    # problem.add_goal(is_location_inspected(l1))
    # problem.add_goal(is_location_inspected(l2))
    # problem.add_goal(is_location_inspected(l3))
    # problem.add_goal(is_location_inspected(l4))
    # problem.add_goal(is_plate_inspected(l1))
    # problem.add_goal(is_plate_inspected(l2))
    # problem.add_goal(is_plate_inspected(l3))
    # problem.add_goal(is_plate_inspected(l4))
    # problem.add_goal(robot_at(r1, base_station_1))
    # problem.add_goal(robot_at(r2, base_station_2))
    print(problem)

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
