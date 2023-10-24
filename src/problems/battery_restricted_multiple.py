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
import argparse
import math

from unified_planning.shortcuts import *

TIME_EPSILON = 5
SURVEY_START, SURVEY_END = (-5, -5), (5, 5)
INSPECTION_HEIGHT = 1, 3
DRONE_VELOCITY = 0.2


def estimate_time(x1, y1, x2, y2, speed=1):
    """Estimate the time taken to travel from (x1, y1) to (x2, y2) at the given speed in seconds"""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) / speed


def estimate_inspection_time(height, speed=1):
    """Estimate the time taken to inspect a plate at the given height at the given speed in seconds"""
    return height / speed


def simulate_battery_usage(distance):
    """Simulate battery usage for the given distance"""
    return 0.5 * distance


def demo_time_triggered_problem():
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
    battery_level = Fluent("battery_level", RealType(0, 100), robot=Robot)

    # Default objects
    r1 = Object("robot1", Robot)
    r2 = Object("robot2", Robot)
    base_station_1 = Object("base_station_1", Location)
    base_station_2 = Object("base_station_2", Location)
    charging_station_1 = Object("charging_station_1", Location)
    charging_station_2 = Object("charging_station_2", Location)
    area = Object("area", Area)
    l1 = Object("l1", Location)
    l2 = Object("l2", Location)
    l3 = Object("l3", Location)
    l4 = Object("l4", Location)

    survey = DurativeAction("survey", robot=Robot, area=Area, From=Location)
    l_from = survey.parameter("From")
    r = survey.parameter("robot")
    survey.set_fixed_duration(10)
    survey.add_condition(StartTiming(), GE(battery_level(r), 95))
    survey.add_condition(StartTiming(), Not(is_surveyed()))
    survey.add_condition(StartTiming(), is_base_station(r, l_from))
    survey.add_condition(StartTiming(), Not(has_plates()))
    survey.add_effect(EndTiming(), is_surveyed(), True)
    survey.add_effect(EndTiming(), battery_level(r), Minus(battery_level(r), 10))

    send_info = InstantaneousAction("send_info", robot=Robot)
    send_info.add_precondition(is_surveyed())
    send_info.add_precondition(Not(has_plates()))
    send_info.add_effect(has_plates(), True)

    move = DurativeAction("move", robot=Robot, l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    r = move.parameter("robot")
    move.set_fixed_duration(5)
    move.add_condition(StartTiming(), GE(battery_level(r), 40))
    move.add_condition(StartTiming(), robot_at(r, l_from))
    move.add_condition(StartTiming(), Not(robot_at(r, l_to)))
    move.add_condition(StartTiming(), has_plates())
    move.add_condition(StartTiming(), is_distance_optimized())
    move.add_effect(EndTiming(), robot_at(r, l_from), False)
    move.add_effect(EndTiming(), robot_at(r, l_to), True)
    move.add_effect(EndTiming(), battery_level(r), Minus(battery_level(r), 5))

    acquire_plates_order = InstantaneousAction("acquire_plates_order", robot=Robot)
    acquire_plates_order.add_precondition(has_plates())
    acquire_plates_order.add_precondition(Not(is_distance_optimized()))
    acquire_plates_order.add_effect(is_distance_optimized(), True)

    inspect_plate = DurativeAction("inspect_plate", robot=Robot, location=Location)
    r = inspect_plate.parameter("robot")
    l = inspect_plate.parameter("location")
    inspect_plate.set_fixed_duration(1)
    inspect_plate.add_condition(StartTiming(), robot_at(r, l))
    inspect_plate.add_condition(StartTiming(), has_plates())
    inspect_plate.add_condition(StartTiming(), is_distance_optimized())
    inspect_plate.add_condition(StartTiming(), GE(battery_level(r), 40))
    inspect_plate.add_effect(EndTiming(), is_location_inspected(l), True)
    inspect_plate.add_effect(EndTiming(), is_plate_inspected(l), True)
    inspect_plate.add_effect(EndTiming(), battery_level(r), Minus(battery_level(r), 1))

    recharge_robot = InstantaneousAction(
        "recharge_drone", robot=Robot, charging_station=Location
    )
    r = recharge_robot.parameter("robot")
    charging_station = recharge_robot.parameter("charging_station")
    recharge_robot.add_precondition(robot_at(r, charging_station))
    recharge_robot.add_effect(battery_level(r), 100.0)

    problem = Problem()

    problem.add_fluent(is_surveyed, default_initial_value=False)
    problem.add_fluent(has_plates, default_initial_value=False)
    problem.add_fluent(is_distance_optimized, default_initial_value=False)
    problem.add_fluent(robot_at, default_initial_value=False)
    problem.add_fluent(is_base_station, default_initial_value=False)
    problem.add_fluent(is_location_inspected, default_initial_value=False)
    problem.add_fluent(is_plate_inspected, default_initial_value=False)
    problem.add_fluent(battery_level, default_initial_value=100)

    problem.add_objects(
        [
            r1,
            r2,
            base_station_1,
            base_station_2,
            charging_station_1,
            charging_station_2,
            area,
            l1,
            l2,
            l3,
            l4,
        ]
    )

    problem.add_actions(
        [survey, send_info, move, acquire_plates_order, inspect_plate, recharge_robot]
    )
    problem.set_initial_value(robot_at(r1, base_station_1), True)
    problem.set_initial_value(is_base_station(r1, base_station_1), True)
    problem.set_initial_value(robot_at(r2, base_station_2), True)
    problem.set_initial_value(is_base_station(r2, base_station_2), True)
    problem.set_initial_value(battery_level(r1), 100.0)

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
    problem.add_goal(robot_at(r1, charging_station_1))
    problem.add_goal(robot_at(r2, charging_station_2))

    return problem


def demo_sequential_problem():
    """Create a simple station verification application"""

    # Prequesties
    survey_time = estimate_time(*SURVEY_START, *SURVEY_END, DRONE_VELOCITY)
    inspect_time = estimate_inspection_time(
        2 * (INSPECTION_HEIGHT[1] - INSPECTION_HEIGHT[0]), DRONE_VELOCITY
    )

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
    battery_level = Fluent("battery_level", RealType(0, 100), robot=Robot)

    # Default objects
    r1 = Object("robot_1", Robot)
    r2 = Object("robot_2", Robot)
    base_station_1 = Object("base_station_1", Location)
    base_station_2 = Object("base_station_2", Location)
    charging_station_1 = Object("charging_station_1", Location)
    charging_station_2 = Object("charging_station_2", Location)
    area = Object("area", Area)
    l1 = Object("l1", Location)
    l2 = Object("l2", Location)
    l3 = Object("l3", Location)
    l4 = Object("l4", Location)

    survey = InstantaneousAction("survey", robot=Robot, area=Area, From=Location)
    l_from = survey.parameter("From")
    r = survey.parameter("robot")
    survey.add_precondition(GE(battery_level(r), 95))
    survey.add_precondition(Not(is_surveyed()))
    survey.add_precondition(is_base_station(r, l_from))
    survey.add_precondition(Not(has_plates()))
    survey.add_effect(is_surveyed(), True)
    survey.add_effect(
        battery_level(r),
        Minus(battery_level(r), int(simulate_battery_usage(survey_time))),
    )

    send_info = InstantaneousAction("send_info", robot=Robot)
    send_info.add_precondition(is_surveyed())
    send_info.add_precondition(Not(has_plates()))
    send_info.add_effect(has_plates(), True)

    move = InstantaneousAction("move", robot=Robot, l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    r = move.parameter("robot")
    move.add_precondition(GE(battery_level(r), 40))
    move.add_precondition(robot_at(r, l_from))
    move.add_precondition(Not(robot_at(r, l_to)))
    move.add_precondition(has_plates())
    move.add_precondition(is_distance_optimized())
    move.add_effect(robot_at(r, l_from), False)
    move.add_effect(robot_at(r, l_to), True)
    move.add_effect(battery_level(r), Minus(battery_level(r), 5))

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
    inspect_plate.add_precondition(GE(battery_level(r), 40))
    inspect_plate.add_effect(is_location_inspected(l), True)
    inspect_plate.add_effect(is_plate_inspected(l), True)
    inspect_plate.add_effect(
        battery_level(r), Minus(battery_level(r), simulate_battery_usage(inspect_time))
    )

    recharge_robot = InstantaneousAction(
        "recharge_drone", robot=Robot, charging_station=Location
    )
    r = recharge_robot.parameter("robot")
    charging_station = recharge_robot.parameter("charging_station")
    recharge_robot.add_precondition(robot_at(r, charging_station))
    recharge_robot.add_effect(battery_level(r), 100.0)

    problem = Problem()

    problem.add_fluent(is_surveyed, default_initial_value=False)
    problem.add_fluent(has_plates, default_initial_value=False)
    problem.add_fluent(is_distance_optimized, default_initial_value=False)
    problem.add_fluent(robot_at, default_initial_value=False)
    problem.add_fluent(is_base_station, default_initial_value=False)
    problem.add_fluent(is_location_inspected, default_initial_value=False)
    problem.add_fluent(is_plate_inspected, default_initial_value=False)
    problem.add_fluent(battery_level, default_initial_value=100)

    problem.add_objects(
        [
            r1,
            r2,
            base_station_1,
            base_station_2,
            charging_station_1,
            charging_station_2,
            area,
            l1,
            l2,
            l3,
            l4,
        ]
    )

    problem.add_actions(
        [survey, send_info, move, acquire_plates_order, inspect_plate, recharge_robot]
    )
    problem.set_initial_value(robot_at(r1, base_station_1), True)
    problem.set_initial_value(is_base_station(r1, base_station_1), True)
    problem.set_initial_value(robot_at(r2, base_station_2), True)
    problem.set_initial_value(is_base_station(r2, base_station_2), True)
    problem.set_initial_value(battery_level(r1), 100.0)

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
    problem.add_goal(robot_at(r1, charging_station_1))
    problem.add_goal(robot_at(r2, charging_station_2))

    return problem


if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument(
        "--sequential", help="Use sequential actions", action="store_true"
    )
    args = argparse.parse_args()
    if args.sequential:
        problem = demo_sequential_problem()
        print("*** Planning ***")
        with OneshotPlanner(
            name="tamer",
            optimality_guarantee=up.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY,
        ) as planner:
            result = planner.solve(problem)
            print("*** Result ***")
            for action_instance in result.plan.actions:
                print(action_instance)
            print("*** End of result ***")
            plan = result.plan
    else:
        problem = demo_time_triggered_problem()

        print("*** Planning ***")
        with OneshotPlanner(
            problem_kind=problem.kind,
            optimality_guarantee=up.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY,
        ) as planner:
            result = planner.solve(problem)
            print("*** Result ***")
            for action_instance in result.plan.timed_actions:
                print(action_instance)
            print("*** End of result ***")
            plan = result.plan
