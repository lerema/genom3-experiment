# Joystick control for the drone
package require genomix

# Connect to the drone
set LIB_PATH $::env(DRONE_VV_PATH)
set handle [genomix::connect localhost:8080]


# Load the components
$handle rpath $LIB_PATH/lib/genom/pocolibs/plugins
$handle rpath /opt/openrobots/lib/genom/pocolibs/plugins

$handle load joystick
$handle load maneuver -i maneuver1
$handle load rotorcraft -i rotorcraft1

maneuver1::set_velocity_limit {v 1.0 w 1.0}

proc get_keymap {key_map {reset false}} {
    # Convert the keymap from the joystick to a more readable format
    # Default mode key mapping
    # {'device': {'ts': {'sec': 1691657401, 'nsec': 387744000}, 'buttons': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'axes': [0, 0, 0, 0, 0, 0, 0, 0]}}
    # {'device': {'ts': {'sec': 1691657401, 'nsec': 387744000}, 'buttons': [A, B, X, Y, LB, RB, back, start, 0, 0, 0], 'axes': [LLT/LRT, LUP/DOWN, LT, RGY, RGX, RT, LGX, LGY]}}

    # TODO: Check if there is a way to check which mode is active
    set new_key_map [dict create]

    if {$reset} {
        dict set new_key_map "A" 0
        dict set new_key_map "B" 0
        dict set new_key_map "X" 0
        dict set new_key_map "Y" 0
        dict set new_key_map "LB" 0
        dict set new_key_map "RB" 0
        dict set new_key_map "back" 0
        dict set new_key_map "start" 0
        dict set new_key_map "LT" 0
        dict set new_key_map "RT" 0
        dict set new_key_map "LUP" 0
        dict set new_key_map "LDOWN" 0
        dict set new_key_map "LLEFT" 0
        dict set new_key_map "LRIGHT" 0
        dict set new_key_map "RTOGG_X" 0
        dict set new_key_map "RTOGG_Y" 0
        dict set new_key_map "LTOGG_X" 0
        dict set new_key_map "LTOGG_Y" 0

        return $new_key_map
    }

    proc retrieve_value {key_map key index} {
        set value [dict get $key_map $key]
        set value [lindex $value $index]
        return $value
    }

    set key_map [dict get $key_map device]

    dict set new_key_map "A" [lindex [retrieve_value $key_map buttons 0] 0]
    dict set new_key_map "B" [lindex [retrieve_value $key_map buttons 1] 0]
    dict set new_key_map "X" [lindex [retrieve_value $key_map buttons 2] 0]
    dict set new_key_map "Y" [lindex [retrieve_value $key_map buttons 3] 0]
    dict set new_key_map "LB" [lindex [retrieve_value $key_map buttons 4] 0]
    dict set new_key_map "RB" [lindex [retrieve_value $key_map buttons 5] 0]
    dict set new_key_map "back" [lindex [retrieve_value $key_map buttons 6] 0]
    dict set new_key_map "start" [lindex [retrieve_value $key_map buttons 7] 0]
    dict set new_key_map "LT" [lindex [retrieve_value $key_map axes 2] 0]
    dict set new_key_map "RT" [lindex [retrieve_value $key_map axes 5] 0]
    dict set new_key_map "LUP" [lindex [retrieve_value $key_map axes 1] 0]
    dict set new_key_map "LDOWN" [lindex [retrieve_value $key_map axes 1] 0]
    dict set new_key_map "LLEFT" [lindex [retrieve_value $key_map axes 0] 0]
    dict set new_key_map "LRIGHT" [lindex [retrieve_value $key_map axes 0] 0]
    dict set new_key_map "RTOGG_X" [lindex [retrieve_value $key_map axes 4] 0]
    dict set new_key_map "RTOGG_Y" [lindex [retrieve_value $key_map axes 3] 0]
    dict set new_key_map "LTOGG_X" [lindex [retrieve_value $key_map axes 6] 0]
    dict set new_key_map "LTOGG_Y" [lindex [retrieve_value $key_map axes 7] 0]

    return $new_key_map
}

set help_text {
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


}

proc accept_command {} {
    puts "Press RT or LT to accept commands"

    while {1} {
        set raw_key_map [joystick::device "Logitech"]
        set result [get_keymap $raw_key_map]
        array set key_map $result

        if {[expr $key_map(B) > 0]} {
            # Stop the drone
            rotorcraft1::stop
        }

        if {$key_map(RT) > 0 || $key_map(LT) > 0} {
            # Process command
            if {$key_map(LUP) < -10000} {
                # Move the drone along z axis
                maneuver1::velocity {vx 0 vy 0 vz 0.5 wz 0 ax 0 ay 0 az 0.1 duration 0}
            } elseif {$key_map(LDOWN) > 10000} {
                maneuver1::velocity {vx 0 vy 0 vz -0.5 wz 0 ax 0 ay 0 az -0.1 duration 0}
            } elseif {$key_map(LTOGG_X) > 10000} {
                maneuver1::velocity {vx -0.5 vy 0 vz 0 wz 0 ax 0.1 ay 0 az 0 duration 0}
            } elseif {$key_map(LTOGG_X) < -10000} {
                maneuver1::velocity {vx 0.5 vy 0 vz 0 wz 0 ax -0.1 ay 0 az 0 duration 0}
            } elseif {$key_map(LTOGG_Y) > 10000} {
                maneuver1::velocity {vx 0 vy -0.5 vz 0 wz 0 ax 0 ay 0.1 az 0 duration 0}
            } elseif {$key_map(LTOGG_Y) < -10000} {
                maneuver1::velocity {vx 0 vy 0.5 vz 0 wz 0 ax 0 ay -0.1 az 0 duration 0}
            } elseif {$key_map(RTOGG_Y) > 10000} {
                maneuver1::velocity {vx 0 vy 0 vz 0 wz -0.5 ax 0 ay 0 az 0.1 duration 0}
            } elseif {$key_map(RTOGG_Y) < -10000} {
                maneuver1::velocity {vx 0 vy 0 vz 0 wz 0.5 ax 0 ay 0 az -0.1 duration 0}
            } else {
                maneuver1::velocity {vx 0 vy 0 vz 0 wz 0 ax 0 ay 0 az 0 duration 0}
            }
        } else {
            maneuver1::velocity {vx 0 vy 0 vz 0 wz 0 ax 0 ay 0 az 0 duration 0}
        }
    }
}

# proc handle_interrupt {} {
#     puts "Keyboard interrupt detected. Exiting..."
#     exit
# }

# signal trap SIGINT handle_interrupt

puts $help_text
