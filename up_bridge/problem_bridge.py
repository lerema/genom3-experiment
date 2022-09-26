from unified_planning.shortcuts import *

from up_bridge.components import *
from up_bridge.planning_bridge import Bridge


class Application:
    # Fluents
    robot_at = Fluents.robot_at
    verify_station_at = Fluents.verify_station_at
    is_surveyed = Fluents.is_surveyed
    is_location_surveyed = Fluents.is_location_surveyed
    is_within_area = Fluents.is_within_area

    # Objects
    l1 = Location("l1", 3.5, 3.5, 1.0, 0.0)
    l2 = Location("l2", -2.5, 1.5, 1.0, 0.0)
    l3 = Location("l3", 1.5, -2.5, 1.0, 0.0)
    l4 = Location("l4", -1.5, -3.5, 1.0, 0.0)
    home = Location("home", 0.0, 0.0, 0.15, 0.0)
    area = Area("area", -5.0, 5.0, -5.0, 5.0, 3.0, 0.0)

    # Actions
    move = Move
    capture_photo = CapturePhoto
    survey = Survey
    send_info = SendInfo

    def __init__(self) -> None:
        self.instance = self


class VerifyStationProblem(Application):
    def __init__(self, bridge: Bridge) -> None:
        assert isinstance(bridge, Bridge), "bridge must be a Generic Bridge instance"
        self.bridge = bridge

    def start_execution(self, action_instances: list, **kwargs):
        self.app = Application()

        for action in action_instances:
            (executor, parameters) = self.bridge.get_executable_action(action)
            execute_action = executor(**kwargs)
            return execute_action(*parameters)

    def get_problem(self):

        self.bridge.create_types([Location, Area])

        f_robot_at = self.bridge.create_fluent(self.robot_at)
        f_verified_station_at = self.bridge.create_fluent(self.verify_station_at)
        f_is_surveyed = self.bridge.create_fluent(self.is_surveyed)
        f_is_location_surveyed = self.bridge.create_fluent(self.is_location_surveyed)
        f_is_within_area = self.bridge.create_fluent(self.is_within_area)

        o_l1 = self.bridge.create_object(str(self.l1), self.l1)
        o_l2 = self.bridge.create_object(str(self.l2), self.l2)
        o_l3 = self.bridge.create_object(str(self.l3), self.l3)
        o_l4 = self.bridge.create_object(str(self.l4), self.l4)
        o_home = self.bridge.create_object(str(self.home), self.home)
        o_area = self.bridge.create_object(str(self.area), self.area)

        move, (a, l_from, l_to) = self.bridge.create_action(self.move)
        move.add_precondition(f_is_surveyed(a))
        move.add_precondition(f_is_location_surveyed(a, l_to))
        move.add_precondition(Not(Equals(l_from, l_to)))
        move.add_precondition(f_robot_at(l_from))
        move.add_precondition(Not(f_robot_at(l_to)))
        move.add_effect(f_robot_at(l_from), False)
        move.add_effect(f_robot_at(l_to), True)

        capture_photo, (a, l) = self.bridge.create_action(self.capture_photo)
        capture_photo.add_precondition(f_is_surveyed(a))
        capture_photo.add_precondition(f_is_location_surveyed(a, l))
        capture_photo.add_precondition(f_robot_at(l))
        capture_photo.add_effect(f_verified_station_at(l), True)
        capture_photo.add_effect(
            f_robot_at(l), True
        )  # Since using instantaneous actions

        survey, a = self.bridge.create_action(self.survey)
        survey.add_precondition(Not(f_is_surveyed(a)))
        survey.add_effect(f_is_surveyed(a), True)

        send_info, (a, l) = self.bridge.create_action(self.send_info)
        send_info.add_precondition(f_is_surveyed(a))
        send_info.add_precondition(f_is_within_area(a, l))
        send_info.add_effect(f_is_location_surveyed(a, l), True)

        problem = self.bridge.define_problem()
        problem.set_initial_value(f_robot_at(o_home), True)
        problem.set_initial_value(f_robot_at(o_l1), False)
        problem.set_initial_value(f_robot_at(o_l2), False)
        problem.set_initial_value(f_robot_at(o_l3), False)
        problem.set_initial_value(f_robot_at(o_l4), False)

        problem.set_initial_value(f_verified_station_at(o_home), True)
        problem.set_initial_value(f_verified_station_at(o_l1), False)
        problem.set_initial_value(f_verified_station_at(o_l2), False)
        problem.set_initial_value(f_verified_station_at(o_l3), False)
        problem.set_initial_value(f_verified_station_at(o_l4), False)

        problem.set_initial_value(f_is_surveyed(o_area), False)
        problem.set_initial_value(f_is_within_area(o_area, o_home), True)
        problem.set_initial_value(f_is_within_area(o_area, o_l1), True)
        problem.set_initial_value(f_is_within_area(o_area, o_l2), True)
        problem.set_initial_value(f_is_within_area(o_area, o_l3), True)
        problem.set_initial_value(f_is_within_area(o_area, o_l4), True)
        problem.set_initial_value(f_is_location_surveyed(o_area, o_home), True)
        problem.set_initial_value(f_is_location_surveyed(o_area, o_l1), False)
        problem.set_initial_value(f_is_location_surveyed(o_area, o_l2), False)
        problem.set_initial_value(f_is_location_surveyed(o_area, o_l3), False)
        problem.set_initial_value(f_is_location_surveyed(o_area, o_l4), False)

        problem.add_goal(f_is_surveyed(o_area))
        problem.add_goal(f_is_location_surveyed(o_area, o_l1))
        problem.add_goal(f_is_location_surveyed(o_area, o_l2))
        problem.add_goal(f_is_location_surveyed(o_area, o_l3))
        problem.add_goal(f_is_location_surveyed(o_area, o_l4))
        problem.add_goal(f_verified_station_at(o_l1))
        problem.add_goal(f_verified_station_at(o_l2))
        problem.add_goal(f_verified_station_at(o_l3))
        problem.add_goal(f_verified_station_at(o_l4))
        problem.add_goal(f_robot_at(o_home))

        return problem
