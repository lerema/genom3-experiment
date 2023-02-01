import unified_planning as up
from unified_planning.shortcuts import *
from drone_api.up.executor import plan_to_dependency_graph
import networkx as nx
from matplotlib import pyplot as plt
from networkx.drawing.nx_agraph import graphviz_layout


def test_dependency_graph():
    Match = UserType("Match")
    Fuse = UserType("Fuse")
    handfree = Fluent("handfree")
    light = Fluent("light")
    match_used = Fluent("match_used", BoolType(), match=Match)
    fuse_mended = Fluent("fuse_mended", BoolType(), fuse=Fuse)
    light_match = DurativeAction("light_match", m=Match)
    m = light_match.parameter("m")
    light_match.set_fixed_duration(6)
    light_match.add_condition(StartTiming(), Not(match_used(m)))
    light_match.add_effect(StartTiming(), match_used(m), True)
    light_match.add_effect(StartTiming(), light, True)
    light_match.add_effect(EndTiming(), light, False)
    mend_fuse = DurativeAction("mend_fuse", f=Fuse)
    f = mend_fuse.parameter("f")
    mend_fuse.set_fixed_duration(5)
    mend_fuse.add_condition(StartTiming(), handfree)
    mend_fuse.add_condition(ClosedTimeInterval(StartTiming(), EndTiming()), light)
    mend_fuse.add_effect(StartTiming(), handfree, False)
    mend_fuse.add_effect(EndTiming(), fuse_mended(f), True)
    mend_fuse.add_effect(EndTiming(), handfree, True)
    f1 = Object("f1", Fuse)
    f2 = Object("f2", Fuse)
    f3 = Object("f3", Fuse)
    m1 = Object("m1", Match)
    m2 = Object("m2", Match)
    m3 = Object("m3", Match)
    problem = Problem("MatchCellar")
    problem.add_fluent(handfree)
    problem.add_fluent(light)
    problem.add_fluent(match_used, default_initial_value=False)
    problem.add_fluent(fuse_mended, default_initial_value=False)
    problem.add_action(light_match)
    problem.add_action(mend_fuse)
    problem.add_object(f1)
    problem.add_object(f2)
    problem.add_object(f3)
    problem.add_object(m1)
    problem.add_object(m2)
    problem.add_object(m3)
    problem.set_initial_value(light, False)
    problem.set_initial_value(handfree, True)
    problem.add_goal(fuse_mended(f1))
    problem.add_goal(fuse_mended(f2))
    problem.add_goal(fuse_mended(f3))
    t_plan = up.plans.TimeTriggeredPlan(
        [
            (
                Fraction(0, 1),
                up.plans.ActionInstance(light_match, (ObjectExp(m1),)),
                Fraction(6, 1),
            ),
            (
                Fraction(1, 100),
                up.plans.ActionInstance(mend_fuse, (ObjectExp(f1),)),
                Fraction(5, 1),
            ),
            (
                Fraction(601, 100),
                up.plans.ActionInstance(light_match, (ObjectExp(m2),)),
                Fraction(6, 1),
            ),
            (
                Fraction(602, 100),
                up.plans.ActionInstance(mend_fuse, (ObjectExp(f2),)),
                Fraction(5, 1),
            ),
            (
                Fraction(1202, 100),
                up.plans.ActionInstance(light_match, (ObjectExp(m3),)),
                Fraction(6, 1),
            ),
            (
                Fraction(1203, 100),
                up.plans.ActionInstance(mend_fuse, (ObjectExp(f3),)),
                Fraction(5, 1),
            ),
        ]
    )

    graph = plan_to_dependency_graph(t_plan)

    # draw graph
    print(t_plan)
    plt.figure(figsize=(10, 10))

    pos = graphviz_layout(graph, prog="dot")
    nx.draw(
        graph, pos, with_labels=True, node_size=2000, node_color="skyblue", font_size=20
    )
    plt.show()


def test_dependency_graph2():
    Location = UserType("Location")
    Robot = UserType("Robot")
    is_at = Fluent("is_at", BoolType(), position=Location, robot=Robot)
    battery_charge = Fluent("battery_charge", RealType(0, 100), robot=Robot)
    is_connected = Fluent(
        "is_connected", BoolType(), location_1=Location, location_2=Location
    )
    visited = Fluent("visited", BoolType(), target=Location)
    move = InstantaneousAction("move", robot=Robot, l_from=Location, l_to=Location)
    robot = move.parameter("robot")
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    move.add_precondition(GE(battery_charge(robot), 10))
    move.add_precondition(Not(Equals(l_from, l_to)))
    move.add_precondition(is_at(l_from, robot))
    move.add_precondition(Not(is_at(l_to, robot)))
    move.add_precondition(Or(is_connected(l_from, l_to), is_connected(l_to, l_from)))
    move.add_effect(is_at(l_from, robot), False)
    move.add_effect(is_at(l_to, robot), True)
    move.add_effect(visited(l_to), True)
    move.add_decrease_effect(battery_charge(robot), 10)
    move_2 = InstantaneousAction("move_2", robot=Robot, l_from=Location, l_to=Location)
    robot = move_2.parameter("robot")
    l_from = move_2.parameter("l_from")
    l_to = move_2.parameter("l_to")
    move_2.add_precondition(GE(battery_charge(robot), 15))
    move_2.add_precondition(Not(Equals(l_from, l_to)))
    move_2.add_precondition(is_at(l_from, robot))
    move_2.add_precondition(Not(is_at(l_to, robot)))
    mid_location = Variable("mid_loc", Location)
    # (E (location mid_location)
    # !((mid_location == l_from) || (mid_location == l_to)) && (is_connected(l_from, mid_location) || is_connected(mid_location, l_from)) &&
    # && (is_connected(l_to, mid_location) || is_connected(mid_location, l_to)))
    move_2.add_precondition(
        Exists(
            And(
                Not(Or(Equals(mid_location, l_from), Equals(mid_location, l_to))),
                Or(
                    is_connected(l_from, mid_location),
                    is_connected(mid_location, l_from),
                ),
                Or(is_connected(l_to, mid_location), is_connected(mid_location, l_to)),
            ),
            mid_location,
        )
    )
    move_2.add_effect(is_at(l_from, robot), False)
    move_2.add_effect(is_at(l_to, robot), True)
    move_2.add_effect(visited(l_to), True)
    move_2.add_decrease_effect(battery_charge(robot), 15)
    l1 = Object("l1", Location)
    l2 = Object("l2", Location)
    l3 = Object("l3", Location)
    l4 = Object("l4", Location)
    l5 = Object("l5", Location)
    r1 = Object("r1", Robot)
    problem = Problem("robot_locations_visited")
    problem.add_fluent(is_at, default_initial_value=False)
    problem.add_fluent(battery_charge)
    problem.add_fluent(is_connected, default_initial_value=False)
    problem.add_fluent(visited, default_initial_value=False)
    problem.add_action(move)
    problem.add_action(move_2)
    problem.add_object(r1)
    problem.add_object(l1)
    problem.add_object(l2)
    problem.add_object(l3)
    problem.add_object(l4)
    problem.add_object(l5)
    problem.set_initial_value(is_at(l1, r1), True)
    problem.set_initial_value(visited(l1), True)
    problem.set_initial_value(is_connected(l1, l2), True)
    problem.set_initial_value(is_connected(l2, l3), True)
    problem.set_initial_value(is_connected(l3, l4), True)
    problem.set_initial_value(is_connected(l4, l5), True)
    problem.set_initial_value(battery_charge(r1), 50)
    problem.add_goal(is_at(l5, r1))
    visited_location = Variable("visited_loc", Location)
    problem.add_goal(Forall(visited(visited_location), visited_location))
    plan = up.plans.SequentialPlan(
        [
            up.plans.ActionInstance(
                move, (ObjectExp(r1), ObjectExp(l1), ObjectExp(l2))
            ),
            up.plans.ActionInstance(
                move, (ObjectExp(r1), ObjectExp(l2), ObjectExp(l3))
            ),
            up.plans.ActionInstance(
                move, (ObjectExp(r1), ObjectExp(l3), ObjectExp(l4))
            ),
            up.plans.ActionInstance(
                move, (ObjectExp(r1), ObjectExp(l4), ObjectExp(l5))
            ),
        ]
    )

    graph = plan_to_dependency_graph(plan)

    # draw graph
    print(plan)
    plt.figure(figsize=(10, 10))

    pos = graphviz_layout(graph, prog="dot")
    nx.draw(
        graph, pos, with_labels=True, node_size=2000, node_color="skyblue", font_size=20
    )
    plt.show()


test_dependency_graph()
