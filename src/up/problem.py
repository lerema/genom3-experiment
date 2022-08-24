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
        """Create a simple battery charging application"""
        Location = UserType("Location")
        robot_at = Fluent("robot_at", BoolType(), position=Location)
        verify_station_at = Fluent("verify_station_at", BoolType(), position=Location)

        move = InstantaneousAction("move", l_from=Location, l_to=Location)
        l_from = move.parameter("l_from")
        l_to = move.parameter("l_to")
        move.add_precondition(Not(Equals(l_from, l_to)))
        move.add_precondition(robot_at(l_from))
        move.add_precondition(Not(robot_at(l_to)))
        move.add_effect(robot_at(l_from), False)
        move.add_effect(robot_at(l_to), True)

        capture_photo = InstantaneousAction("capture_photo", l=Location)
        l = capture_photo.parameter("l")
        capture_photo.add_precondition(robot_at(l))
        capture_photo.add_effect(verify_station_at(l), True)

        l1 = Object("l1", Location)
        l2 = Object("l2", Location)
        l3 = Object("l3", Location)
        l4 = Object("l4", Location)
        home = Object("home", Location)

        problem = Problem("robot")
        problem.add_fluent(robot_at)
        problem.add_fluent(verify_station_at)

        problem.add_action(move)
        problem.add_action(capture_photo)

        problem.add_object(l1)
        problem.add_object(l2)
        problem.add_object(l3)
        problem.add_object(l4)
        problem.add_object(home)

        problem.set_initial_value(robot_at(home), True)
        problem.set_initial_value(robot_at(l1), False)
        problem.set_initial_value(robot_at(l2), False)
        problem.set_initial_value(robot_at(l3), False)
        problem.set_initial_value(robot_at(l4), False)

        problem.set_initial_value(verify_station_at(home), False)
        problem.set_initial_value(verify_station_at(l1), False)
        problem.set_initial_value(verify_station_at(l2), False)
        problem.set_initial_value(verify_station_at(l3), False)
        problem.set_initial_value(verify_station_at(l4), False)

        problem.add_goal(verify_station_at(l2))
        # problem.add_goal(verify_station_at(l3))
        problem.add_goal(verify_station_at(l4))
        problem.add_goal(robot_at(home))

        return problem
