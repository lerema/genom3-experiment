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

from unified_planning.shortcuts import Not, Equals, OneshotPlanner
from up_bridge.bridge import Bridge

from up_components import (
    Fluents,
    Move,
    CapturePhoto,
    Survey,
    GatherInfo,
    Location,
    Area,
)


class Objects:
    # Objects
    l1 = Location("l1", x=3.5, y=3.5, z=1.0, yaw=0.0)
    l2 = Location("l2", x=-2.5, y=1.5, z=1.0, yaw=0.0)
    l3 = Location("l3", x=1.5, y=-2.5, z=1.0, yaw=0.0)
    l4 = Location("l4", x=-1.5, y=-3.5, z=1.0, yaw=0.0)
    home = Location("home", x=0.0, y=0.0, z=1.0, yaw=0.0)
    area = Area("area", xmin=-4.0, xmax=4.0, ymin=-4.0, ymax=4.0, z=3.0, yaw=0.0)


class VerifyStationProblem:
    def __init__(self, bridge: Bridge) -> None:
        assert isinstance(bridge, Bridge), "bridge must be a Generic Bridge instance"
        self.bridge = bridge
        self.objects = Objects()

    def get_problem(self, no_drones: int = 1):
        if no_drones == 1:
            return self._get_single_drone_problem()

    def _get_single_drone_problem(self):
        self.bridge.create_types([Location, Area])

        f_robot_at = self.bridge.create_fluent_from_function(Fluents.robot_at)
        f_verified_station_at = self.bridge.create_fluent_from_function(
            Fluents.verify_station_at
        )
        f_is_surveyed = self.bridge.create_fluent_from_function(Fluents.is_surveyed)
        f_is_location_surveyed = self.bridge.create_fluent_from_function(
            Fluents.is_location_surveyed
        )
        f_is_within_area = self.bridge.create_fluent_from_function(
            Fluents.is_within_area
        )

        o_l1 = self.bridge.create_object(str(self.objects.l1), self.objects.l1)
        o_l2 = self.bridge.create_object(str(self.objects.l2), self.objects.l2)
        o_l3 = self.bridge.create_object(str(self.objects.l3), self.objects.l3)
        o_l4 = self.bridge.create_object(str(self.objects.l4), self.objects.l4)
        o_home = self.bridge.create_object(str(self.objects.home), self.objects.home)
        o_area = self.bridge.create_object(str(self.objects.area), self.objects.area)

        move, (a, l_from, l_to) = self.bridge.create_action(
            "Move", _callable=Move, area=Area, l_from=Location, l_to=Location
        )
        move.add_precondition(f_is_surveyed(a))
        move.add_precondition(f_is_location_surveyed(a, l_to))
        move.add_precondition(Not(Equals(l_from, l_to)))
        move.add_precondition(f_robot_at(l_from))
        move.add_precondition(Not(f_robot_at(l_to)))
        move.add_effect(f_robot_at(l_from), False)
        move.add_effect(f_robot_at(l_to), True)

        capture_photo, (a, l) = self.bridge.create_action(
            "CapturePhoto", _callable=CapturePhoto, area=Area, l=Location
        )
        capture_photo.add_precondition(f_is_surveyed(a))
        capture_photo.add_precondition(f_is_location_surveyed(a, l))
        capture_photo.add_precondition(f_robot_at(l))
        capture_photo.add_effect(f_verified_station_at(l), True)
        capture_photo.add_effect(
            f_robot_at(l), True
        )  # Since using instantaneous actions

        survey, [a] = self.bridge.create_action("Survey", _callable=Survey, area=Area)
        survey.add_precondition(Not(f_is_surveyed(a)))
        survey.add_effect(f_is_surveyed(a), True)

        gather_info, (a, l) = self.bridge.create_action(
            "GatherInfo", _callable=GatherInfo, area=Area, l=Location
        )
        gather_info.add_precondition(f_is_surveyed(a))
        gather_info.add_precondition(f_is_within_area(a, l))
        gather_info.add_effect(f_is_location_surveyed(a, l), True)

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


def main():
    """Main function"""
    actions = []
    bridge = Bridge()
    demo = VerifyStationProblem(bridge)
    problem = demo.get_problem()
    with OneshotPlanner(name="aries") as planner:
        result = planner.solve(problem)
        print("*** Result ***")
        for action_instance in result.plan.timed_actions:
            print(action_instance)
            actions.append(action_instance)
        print("*** End of result ***")


if __name__ == "__main__":
    main()
