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


class VerifyStationProblem(object):
    def demo_facts(self):
        """Create a list of facts for the robot to check"""
        pass

    def demo_execution(self):
        """Execute the problem"""
        pass

    def demo_problem(self):
        """Create a simple station verification application"""
        Location = UserType("Location")
        Area = UserType("Area")

        robot_at = Fluent("robot_at", BoolType(), position=Location)
        verify_station_at = Fluent("verify_station_at", BoolType(), position=Location)
        is_surveyed = Fluent("is_surveyed", BoolType(), area=Area)
        is_location_surveyed = Fluent(
            "is_location_surveyed", BoolType(), area=Area, position=Location
        )
        is_within_area = Fluent(
            "is_within_area", BoolType(), area=Area, position=Location
        )

        move = InstantaneousAction("move", area=Area, l_from=Location, l_to=Location)
        l_from = move.parameter("l_from")
        l_to = move.parameter("l_to")
        area = move.parameter("area")
        move.add_precondition(is_surveyed(area))
        move.add_precondition(is_location_surveyed(area, l_to))
        move.add_precondition(Not(Equals(l_from, l_to)))
        move.add_precondition(robot_at(l_from))
        move.add_precondition(Not(robot_at(l_to)))
        move.add_effect(robot_at(l_from), False)
        move.add_effect(robot_at(l_to), True)

        capture_photo = InstantaneousAction("capture_photo", area=Area, l=Location)
        l = capture_photo.parameter("l")
        area = capture_photo.parameter("area")
        capture_photo.add_precondition(is_surveyed(area))
        capture_photo.add_precondition(is_location_surveyed(area, l))
        capture_photo.add_precondition(robot_at(l))
        capture_photo.add_effect(verify_station_at(l), True)
        capture_photo.add_effect(robot_at(l), True)  # TODO: why is this needed?

        survey = InstantaneousAction("survey", area=Area)
        area = survey.parameter("area")
        survey.add_precondition(Not(is_surveyed(area)))
        survey.add_effect(is_surveyed(area), True)

        send_info = InstantaneousAction("send_info", area=Area, position=Location)
        area = send_info.parameter("area")
        position = send_info.parameter("position")
        send_info.add_precondition(is_surveyed(area))
        send_info.add_precondition(is_within_area(area, position))
        send_info.add_effect(is_location_surveyed(area, position), True)

        l1 = Object("l1", Location)
        l2 = Object("l2", Location)
        l3 = Object("l3", Location)
        l4 = Object("l4", Location)
        area = Object("area", Area)
        home = Object("home", Location)

        problem = Problem("robot")
        problem.add_fluent(robot_at)
        problem.add_fluent(verify_station_at)
        problem.add_fluent(is_surveyed)
        problem.add_fluent(is_location_surveyed)
        problem.add_fluent(is_within_area)

        problem.add_action(move)
        problem.add_action(capture_photo)
        problem.add_action(survey)
        problem.add_action(send_info)

        problem.add_object(l1)
        problem.add_object(l2)
        problem.add_object(l3)
        problem.add_object(l4)
        problem.add_object(area)
        problem.add_object(home)

        problem.set_initial_value(robot_at(home), True)
        problem.set_initial_value(robot_at(l1), False)
        problem.set_initial_value(robot_at(l2), False)
        problem.set_initial_value(robot_at(l3), False)
        problem.set_initial_value(robot_at(l4), False)

        problem.set_initial_value(verify_station_at(home), True)
        problem.set_initial_value(verify_station_at(l1), False)
        problem.set_initial_value(verify_station_at(l2), False)
        problem.set_initial_value(verify_station_at(l3), False)
        problem.set_initial_value(verify_station_at(l4), False)

        # Without actual preconditions, this is not a valid problem. It is only a
        # placeholder for the demo.
        problem.set_initial_value(is_surveyed(area), False)
        problem.set_initial_value(is_within_area(area, home), True)
        problem.set_initial_value(is_within_area(area, l1), True)
        problem.set_initial_value(is_within_area(area, l2), True)
        problem.set_initial_value(is_within_area(area, l3), True)
        problem.set_initial_value(is_within_area(area, l4), True)
        problem.set_initial_value(is_location_surveyed(area, home), True)
        problem.set_initial_value(is_location_surveyed(area, l1), False)
        problem.set_initial_value(is_location_surveyed(area, l2), False)
        problem.set_initial_value(is_location_surveyed(area, l3), False)
        problem.set_initial_value(is_location_surveyed(area, l4), False)

        problem.add_goal(is_surveyed(area))
        problem.add_goal(is_location_surveyed(area, l1))
        problem.add_goal(is_location_surveyed(area, l2))
        problem.add_goal(is_location_surveyed(area, l3))
        problem.add_goal(is_location_surveyed(area, l4))
        problem.add_goal(verify_station_at(l1))
        problem.add_goal(verify_station_at(l2))
        problem.add_goal(verify_station_at(l3))
        problem.add_goal(verify_station_at(l4))
        problem.add_goal(robot_at(home))

        return problem


problem = VerifyStationProblem().demo_problem()
print("*** Planning ***")
with OneshotPlanner(name="tamer", optimality_guarantee=up.engines.PlanGenerationResultStatus.SOLVED_OPTIMALLY) as planner:
    result = planner.solve(problem)
    print("*** Result ***")
    for action_instance in result.plan.actions:
        print(action_instance)
    print("*** End of result ***")
    plan = result.plan