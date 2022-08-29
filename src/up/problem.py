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

        move = InstantaneousAction("move", l_from=Location, l_to=Location, area=Area)
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

        capture_photo = InstantaneousAction("capture_photo", l=Location, area=Area)
        l = capture_photo.parameter("l")
        area = capture_photo.parameter("area")
        capture_photo.add_precondition(is_surveyed(area))
        capture_photo.add_precondition(is_location_surveyed(area, l))
        capture_photo.add_precondition(robot_at(l))
        capture_photo.add_effect(verify_station_at(l), True)

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

        problem.set_initial_value(robot_at(l1), True)
        problem.set_initial_value(robot_at(l2), False)
        problem.set_initial_value(robot_at(l3), False)
        problem.set_initial_value(robot_at(l4), False)

        problem.set_initial_value(verify_station_at(l1), False)
        problem.set_initial_value(verify_station_at(l2), False)
        problem.set_initial_value(verify_station_at(l3), False)
        problem.set_initial_value(verify_station_at(l4), False)

        # Without actual preconditions, this is not a valid problem. It is only a
        # placeholder for the demo.
        problem.set_initial_value(is_surveyed(area), False)
        problem.set_initial_value(is_within_area(area, l1), True)
        problem.set_initial_value(is_within_area(area, l2), True)
        problem.set_initial_value(is_within_area(area, l3), True)
        problem.set_initial_value(is_within_area(area, l4), True)
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
        problem.add_goal(robot_at(l4))

        return problem
