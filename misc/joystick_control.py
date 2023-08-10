#!/usr/bin/env python3
"""Joystick control for the drone"""
import os

import genomix
from drone_api.connect import Connector

LIB_PATH = os.environ.get("DRONE_VV_PATH")
handle = genomix.connect("localhost:8080")

joystick = handle.load(f"{LIB_PATH}/lib/genom/pocolibs/plugins/joystick.so")
c = Connector(1)

c.maneuver.set_velocity_limit(
    v=1.0,  # m/s
    w=1.0,  # rad/s
)


def get_keymap(key_map, reset=False) -> dict:
    """Convert the keymap from the joystick to a more readable format"""

    # Default mode key mapping
    # {'device': {'ts': {'sec': 1691657401, 'nsec': 387744000}, 'buttons': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'axes': [0, 0, 0, 0, 0, 0, 0, 0]}}
    # {'device': {'ts': {'sec': 1691657401, 'nsec': 387744000}, 'buttons': [A, B, X, Y, LB, RB, back, start, 0, 0, 0], 'axes': [LLT/LRT, LUP/DOWN, LT, RGY, RGX, RT, LGX, LGY]}}

    # TODO: Check if there is a way to check which mode is active
    if reset:
        new_key_map = {
            "A": 0,
            "B": 0,
            "X": 0,
            "Y": 0,
            "LB": 0,
            "RB": 0,
            "back": 0,
            "start": 0,
            "LT": 0,
            "RT": 0,
            "LUP": 0,
            "LDOWN": 0,
            "LLEFT": 0,
            "LRIGHT": 0,
            "RTOGG_X": 0,
            "RTOGG_Y": 0,
            "LTOGG_X": 0,
            "LTOGG_Y": 0,
        }

        return new_key_map

    new_key_map = {}
    new_key_map["A"] = key_map["device"]["buttons"][0]
    new_key_map["B"] = key_map["device"]["buttons"][1]
    new_key_map["X"] = key_map["device"]["buttons"][2]
    new_key_map["Y"] = key_map["device"]["buttons"][3]
    new_key_map["LB"] = key_map["device"]["buttons"][4]
    new_key_map["RB"] = key_map["device"]["buttons"][5]
    new_key_map["back"] = key_map["device"]["buttons"][6]
    new_key_map["start"] = key_map["device"]["buttons"][7]
    new_key_map["LT"] = key_map["device"]["axes"][2]
    new_key_map["RT"] = key_map["device"]["axes"][5]
    new_key_map["LUP"] = key_map["device"]["axes"][1]
    new_key_map["LDOWN"] = key_map["device"]["axes"][1]
    new_key_map["LLEFT"] = key_map["device"]["axes"][0]
    new_key_map["LRIGHT"] = key_map["device"]["axes"][0]
    new_key_map["RTOGG_X"] = key_map["device"]["axes"][4]
    new_key_map["RTOGG_Y"] = key_map["device"]["axes"][3]
    new_key_map["LTOGG_X"] = key_map["device"]["axes"][6]
    new_key_map["LTOGG_Y"] = key_map["device"]["axes"][7]

    return new_key_map


help_text = """\033[1m
This script will help you control the drone using a joystick.

The controls are as follows:

    RT/LT: Always press to accept any commands
    B: Kill switch for drone. (rotorcraft.stop)
    A: Start the drone
    LUP: Move the drone along z axis
    LDOWN: Move the drone along z axis
    LTOGG_X: Move the drone along x axis
    LTOGG_Y: Move the drone along y axis
    RTOGG_X: Rotate the drone along x axis
    RTOGG_Y: Rotate the drone along y axis

\033[0m
"""
print(help_text)
print("Ready to receive joystick commands", end="\r")
try:
    while True:
        key_map = joystick.device("Logitech")
        key_map = get_keymap(key_map)
        if key_map["B"]:
            # Stop the drone
            c.rotorcraft.stop()

        if (key_map["RT"] > 0) or (key_map["LT"] > 0):
            # Process command
            print("Processing command" + " " * 20, end="\r")
            if key_map["A"]:
                c.setup()
            elif key_map["LUP"] < -10000:
                # Move the drone along z axis
                c.maneuver.velocity(
                    vx=0, vy=0, vz=0.5, wz=0, ax=0, ay=0, az=0.1, duration=0
                )
            elif key_map["LDOWN"] > 10000:
                c.maneuver.velocity(
                    vx=0, vy=0, vz=-0.5, wz=0, ax=0, ay=0, az=-0.1, duration=0
                )
            elif key_map["LTOGG_X"] > 10000:
                c.maneuver.velocity(
                    vx=0.5, vy=0, vz=0, wz=0, ax=0.1, ay=0, az=0, duration=0
                )
            elif key_map["LTOGG_X"] < -10000:
                c.maneuver.velocity(
                    vx=-0.5, vy=0, vz=0, wz=0, ax=-0.1, ay=0, az=0, duration=0
                )
            elif key_map["LTOGG_Y"] > 10000:
                c.maneuver.velocity(
                    vx=0, vy=0.5, vz=0, wz=0, ax=0, ay=0.1, az=0, duration=0
                )
            elif key_map["LTOGG_Y"] < -10000:
                c.maneuver.velocity(
                    vx=0, vy=-0.5, vz=0, wz=0, ax=0, ay=-0.1, az=0, duration=0
                )
            elif key_map["RTOGG_Y"] > 10000:
                c.maneuver.velocity(
                    vx=0, vy=0, vz=0, wz=-0.5, ax=0, ay=0, az=0.1, duration=0
                )
            elif key_map["RTOGG_Y"] < -10000:
                c.maneuver.velocity(
                    vx=0, vy=0, vz=0, wz=0.5, ax=0, ay=0, az=-0.1, duration=0
                )
            else:
                c.maneuver.velocity(
                    vx=0, vy=0, vz=0, wz=0, ax=0, ay=0, az=0, duration=0
                )

        else:
            print("Press RT or LT to accept commands", end="\r")
            c.maneuver.velocity(vx=0, vy=0, vz=0, wz=0, ax=0, ay=0, az=0, duration=0)
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
