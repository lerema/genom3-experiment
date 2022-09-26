from unified_planning.shortcuts import *

from components import *
from planning_bridge import Bridge

bridge = Bridge()

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

bridge.create_types([Location, Area])

f_robot_at = bridge.create_fluent(robot_at)
f_verified_station_at = bridge.create_fluent(verify_station_at)
f_is_surveyed = bridge.create_fluent(is_surveyed)
f_is_location_surveyed = bridge.create_fluent(is_location_surveyed)
f_is_within_area = bridge.create_fluent(is_within_area)

o_l1 = bridge.create_object(str(l1), l1)
o_l2 = bridge.create_object(str(l2), l2)
o_l3 = bridge.create_object(str(l3), l3)
o_l4 = bridge.create_object(str(l4), l4)
o_home = bridge.create_object(str(home), home)
o_area = bridge.create_object(str(area), area)

move, (a, l_from, l_to) = bridge.create_action(move)
move.add_precondition(f_is_surveyed(a))
move.add_precondition(f_is_location_surveyed(a, l_to))
move.add_precondition(Not(Equals(l_from, l_to)))
move.add_precondition(f_robot_at(l_from))
move.add_precondition(Not(f_robot_at(l_to)))
move.add_effect(f_robot_at(l_from), False)
move.add_effect(f_robot_at(l_to), True)


capture_photo, (a, l) = bridge.create_action(capture_photo)
capture_photo.add_precondition(f_is_surveyed(a))
capture_photo.add_precondition(f_is_location_surveyed(a, l))
capture_photo.add_precondition(f_robot_at(l))
capture_photo.add_effect(f_verified_station_at(l), True)
capture_photo.add_effect(f_robot_at(l), True)  # Since using instantaneous actions

survey, a = bridge.create_action(survey)
survey.add_precondition(Not(f_is_surveyed(a)))
survey.add_effect(f_is_surveyed(a), True)

send_info, (a, l) = bridge.create_action(send_info)
send_info.add_precondition(f_is_surveyed(a))
send_info.add_precondition(f_is_within_area(a, l))
send_info.add_effect(f_is_location_surveyed(a, l), True)

problem = bridge.define_problem()
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

with OneshotPlanner(name="aries") as planner:
    result = planner.solve(problem)
    print("*** Result ***")
    for action_instance in result.plan.timed_actions:
        print(action_instance)
    print("*** End Result ***")
