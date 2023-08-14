#
# Mostly copy paste from Martin Jacquet's script
#

set use_hippo 0
set simu -1
set cam 0
set robot -1
set use_tf 0
set mw pocolibs

if { $::argc > 0 } {
    set i 1
    foreach arg $::argv {
        #puts "argument $i is $arg"
        #incr i
        switch -exact $arg {
            -ros {
                set mw ros
            }
            -tf {
                set use_tf 1
            }
            -simu {
                set simu 1
            }
            -cam {
                set cam 1
            }
            -robot {
                set robot 1
            }
            -hippo {
                set use_hippo 1
            }
            default {
                puts "argument $arg is not recognized"
                break
            }
        }
    }
}

if { $simu == -1 && $robot == -1 } {
    puts "One of simu or robot arg must be specified."
    exit 2
}

if { $simu == 1 && $robot == 1 } {
    puts "Cannot have both simu or robot at the same time."
    exit 2
}

if {$simu == 1} {
    set robot 0
} else {
    set simu 0
}

puts "simu: $simu, cam: $cam, use_hippo: $use_hippo, use_tf: $use_tf, mw: $mw"

# UAV PARAMETERS

set m 1.28
set d 0.23
if {$robot} {
    set cf 5.9e-4
} else {
    set cf 6.5e-4
}
set ct 1e-5
set mct [expr {-$ct}]
set dc [expr {$d*$cf}]
set mdc [expr {-$dc}]

set G "0     0    0    0    0  0  0  0 \
    0     0    0    0    0  0  0  0 \
    $cf   $cf  $cf  $cf  0  0  0  0 \
    0     $dc   0   $mdc 0  0  0  0 \
    $mdc  0    $dc  0    0  0  0  0 \
    $ct   $mct $ct  $mct 0  0  0  0"
set J "0.015 0 0  0  0.015  0 0  0 0.007"

# GENOMIX & RPATH
set host localhost

package require genomix
set g [genomix::connect $host:8080]

if {$::env(USER) eq "felix"} {
    $g rpath  $::env(HOME)/work/lib/genom/$mw/plugins/
} elseif {$::env(USER) eq "docker"} {
    $g rpath  /opt/openrobots/lib/genom/$mw/plugins/
} elseif {$robot} {
    $g rpath  /opt/openrobots/lib/genom/$mw/plugins/
} else {
    $g rpath  $::env(DRONE_VV_PATH)/lib/genom/$mw/plugins/
}

$g load optitrack
$g load pom -i pom1
$g load maneuver -i maneuver1
$g load rotorcraft -i rotorcraft1
$g load nhfc -i nhfc1
if {$use_hippo} {
    $g load hippo
}

if {$use_tf} {
    $g load tf2
}

if {$cam} {
    $g load CT_drone -i CT_drone1
}

proc init {} {
    global m
    global G
    global J
    global use_tf
    global simu
    global cam
    global robot

    # OPTITRACK
    if {$robot} {
        optitrack::connect { host muybridge host_port 1510 mcast 239.192.168.30 mcast_port 1511 }
    } else {
        optitrack::connect { host localhost host_port 1509 mcast "" mcast_port "" }
    }
    after 1000


    # POM
    pom1::connect_port { local measure/imu remote rotorcraft1/imu }
    if {$robot} {
        pom1::connect_port { local measure/mocap remote optitrack/bodies/Lerema }
    } else {
        pom1::connect_port { local measure/mag remote rotorcraft1/mag }
        pom1::connect_port { local measure/mocap remote optitrack/bodies/QR1 }
    }

    pom1::add_measurement { port imu x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
    pom1::add_measurement { port mocap x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
    if {$simu} {
        pom1::add_measurement { port mag x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
        pom1::set_mag_field { magdir { x 23.816e-6 y -0.41e-6 z -39.829e-6 } }
    }

    pom1::set_history_length { history_length 0.5 }


    # MANEUVER
    maneuver1::connect_port { local state remote pom1/frame/robot }
    if {$simu} {
        maneuver1::set_velocity_limit {v 2 w 1}
    }


    # ROTORCRAFT
    if {$robot} {
        rotorcraft1::connect { serial /dev/ttyUSB0 baud 500000 }
    } else {
        if {$cam} {
            rotorcraft1::connect { serial /tmp/pty-quad1 baud 500000 }
        } else {
            rotorcraft1::connect { serial /tmp/pty-mrsim-quadrotor baud 500000 }
        }
    }
    rotorcraft1::connect_port { local rotor_input remote nhfc1/rotor_input }

    if {$robot} {
        rotorcraft1::set_sensor_rate { rate { imu 1000 mag 0 motor 16 battery 1 } }

        set imu_calib [read [open calibrations/imu_calib.txt r]]
        rotorcraft1::set_imu_calibration $imu_calib
    } else {
        rotorcraft1::set_sensor_rate { rate { imu 1000 mag 50 motor 16 battery 1 } }
    }

    # NHFC
    # nhfc1::set_mass $m
    # nhfc1::set_geom $G $J
    nhfc1::set_gtmrp_geom {}

    if {$robot} {
        nhfc1::set_servo_gain { gain {Kpxy 20 Kpz 25 Kqxy 3 Kqz 0.3 Kvxy 15
            Kvz 20  Kwxy 0.3 Kwz 0.03 Kixy 0.5  Kiz 3 } }
        } else {
        nhfc1::set_servo_gain { gain {Kpxy 20 Kpz 20 Kqxy 3  Kqz 0.3
            Kvxy 8 Kvz 8  Kwxy 0.3 Kwz 0.03 Kixy 0  Kiz 0 } }
        }
        nhfc1::set_emerg { emerg { descent 1.2 dx 0.1 dq 1 dv 0.3 dw 1 } }
        nhfc1::connect_port { local state remote pom1/frame/robot }
        nhfc1::connect_port { local reference remote maneuver1/desired }

        if {$simu} {
        nhfc1::set_wlimit { wmin 16 wmax 100}
    }

    #CT_drone
    if {$cam} {
        CT_drone1::Set_my_r 5
        CT_drone1::Set_my_g 5
        CT_drone1::Set_my_b 255
        CT_drone1::Set_my_seuil 40

        CT_drone1::connect_port { local Pose remote pom1/frame/robot }

        CT_drone1::SetCameraImageTopicName /quad1/down_camera_link/down_raw_image
        CT_drone1::SetCameraInfoTopicName /quad1/down_camera_link/down_info_image
    }

    if {$use_tf} {
        #tf2
        tf2::connect_port Poses/drone pom1/frame/robot
        tf2::connect_port Poses/drone_pos pom1/frame/robot
        tf2::connect_port OccupancyGrids/og CT_drone1/OccupancyGrid

        # tf2::Init

        tf2::PublishStaticTF -- base drone 0 0 0 0 0 0
        # f2::PublishStaticTF -- down_camera_link drone 0 0 0 0 1.5708 0

        tf2::AddDynamicTF drone world 10 1
        tf2::AddDynamicPosTF drone_pos world 10 1

        tf2::AddOdometry {name drone}
        tf2::AddTwistFromPose {name drone frame drone_pos topic drone_twist ms_period 10}
        tf2::AddWrenchFromPose {name drone frame drone_pos topic drone_wrench ms_period 10}

        tf2::PublishStaticTF -- og world 0 0 0 0 0 0
        tf2::AddOccupancyGrid og og og 50

    }
}

# LOGS
proc start_log {} {
    global simu
    global robot

    set log_rate_d 1
    set log_date [string trim [exec date +"%Y%m%d-%H%M%S"] "\""]

    if {$robot} {
        set dir_logs /log/log-quad-${log_date}
    } else {
        set dir_logs /tmp/log-quad-${log_date}
    }

    file mkdir $dir_logs

    pom1::log_state ${dir_logs}/pom.log $log_rate_d
    pom1::log_measurements ${dir_logs}/pom-measurements.log
    rotorcraft1::log ${dir_logs}/rotorcraft.log $log_rate_d
    nhfc1::log ${dir_logs}/nhfc.log $log_rate_d
    maneuver1::log ${dir_logs}/maneuver.log $log_rate_d
    optitrack::set_logfile ${dir_logs}/optitrack.log
}

# SETUP
proc setup {} {
    global cam
    #    start_log
    rotorcraft1::start
    maneuver1::set_bounds -- -100 100 -100 100 -2 30 -10 10
    maneuver1::set_current_state
    nhfc1::set_current_position
    rotorcraft1::servo &
    maneuver1::take_off { height 0.15 duration 0 }
    nhfc1::servo &

    if {$cam} {
        CT_drone1::PublishOccupancyGrid &
    }

}

proc restart {} {
    rotorcraft1::start
    maneuver1::set_current_state
    nhfc1::set_current_position
    rotorcraft1::servo &
    maneuver1::take_off { height 0.15 duration 0 }
    nhfc1::servo &
}

proc land {} {
    global robot

    if {$robot} {
        maneuver1::take_off {height 0.15 duration 0 }
    } else {
        maneuver1::take_off {height 0.05 duration 0 }
    }
    rotorcraft1::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
}

proc stop {} {
    rotorcraft1::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
    rotorcraft1::stop
}

proc stop_all {} {
    rotorcraft1::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
    rotorcraft1::stop
    nhfc1::stop
    maneuver1::stop
}

# STOP LOGS
proc stop_log {} {
    pom1::log_stop
    rotorcraft1::log_stop
    nhfc1::log_stop
    maneuver1::log_stop
}

proc kill_all {} {
    global use_tf
    global cam

    optitrack::kill
    pom1::kill
    maneuver1::kill
    rotorcraft1::kill
    nhfc1::kill
    if {$use_tf} {
        tf2::kill
    }
    if {$cam} {
        CT_drone1::kill
    }
}

if {$simu} {
    proc move {} {
        ::maneuver1::set_bounds -- -20 20 -20 20 -2 10 -3.14 3.14
        ::maneuver1::take_off 3 0
        after 1000
        ::maneuver1::waypoint -- 1.5 0 4 0     0 0 0 0   0 0 0  0
        ::maneuver1::waypoint -- 1 -0.5 1 3.14   0 0 0 0   0 0 0  0
        ::maneuver1::waypoint -- 1 -0.5 4 0    0 0 0 0   0 0 0  0
        ::maneuver1::waypoint --  1.5 0 4 0    0 0 0 0   0 0 0  0
        ::maneuver1::wait
        after 1000
        ::maneuver1::take_off 2 0
    }


    proc move2 {} {
        ::maneuver1::set_current_state

        ::maneuver1::set_bounds -- -100 100 -100 100 -2 30 -10 10
        ::maneuver1::take_off 3.0 0
        after 1000
        ::maneuver1::waypoint -- 10.0 0 5 0     0 0 0 0   0 0 0  0
        ::maneuver1::waypoint -- 1 -8.5 8 3.14   0 0 0 0   0 0 0  0
        ::maneuver1::waypoint -- 20 -25 7 0    0 0 0 0   0 0 0  0
        ::maneuver1::waypoint --  30 0 8 0    0 0 0 0   0 0 0  0
        ::maneuver1::waypoint --  0 0 6 0    0 0 0 0   0 0 0  0
        ::maneuver1::wait
        after 1000
        ::maneuver1::take_off 2.0 0
    }
}

proc carre {} {
    global robot

    if {$robot} {
        ::maneuver1::goto {x 0.5 y 0.5 z 0.7 yaw 2 duration 0}
        ::maneuver1::goto {x 0.5 y -0.5 z 0.5 yaw -2 duration 0}
        ::maneuver1::goto {x -0.5 y -0.5 z 0.7 yaw 2 duration 0}
        ::maneuver1::goto {x -0.5 y 0.5 z 0.3 yaw -2 duration 0}
    } else {
        ::maneuver1::set_bounds -- -10 10 -10 10 -1 10 -4 4
        ::maneuver1::goto {x 6 y 6 z 1.7 yaw 2 duration 0}
        ::maneuver1::goto {x 6 y -6 z 2.7 yaw -2 duration 0}
        ::maneuver1::goto {x -6 y -6 z 0.7 yaw 2 duration 0}
        ::maneuver1::goto {x -6 y 6 z 3.7 yaw -2 duration 0}
    }
}

proc draw_H {x y z width height} {
    ::tf2::SetOdometryDisplay drone 0
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 1
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height/2.0}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height/2.0}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
}

proc draw_P {x y z width height} {
    ::tf2::SetOdometryDisplay drone 0
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 1
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height*2.0/3.0}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*2.0/3.0}] yaw [expr 0] duration 0]
}

proc draw_O {x y z width height} {
    ::tf2::SetOdometryDisplay drone 0
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 1
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
}

proc draw_I {x y z width height} {
    ::tf2::SetOdometryDisplay drone 0
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 1
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 0
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*1.2}] yaw [expr 0] duration 0]
    ::tf2::SetOdometryDisplay drone 1
    ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*1.2}] yaw [expr -3.14] duration 0]
}

proc draw_hippo {x y z lw lh} {
    draw_H $x $y $z $lw $lh
    draw_I [expr {$x + $lw*1.4}] $y $z $lw $lh
    draw_P [expr {$x + $lw*1.8}] $y $z $lw $lh
    draw_P [expr {$x + $lw*3.2}] $y $z $lw $lh
    draw_O [expr {$x + $lw*4.6}] $y $z $lw $lh
}

if {$simu} {
    proc fly_around {} {
        ::maneuver1::set_bounds -- -10 10 -10 10 -1 10 -4 4
        while 1 {
            if { [catch {maneuver1::goto [dict create x [expr {rand()*20 - 10}] y [expr {rand()*20 - 10}] z [expr {rand()*10 + 0.2}] yaw [expr {rand()*6.26 -3.14 }] duration 0]} result]} {
                puts $result
                return
            }
        }
    }
}


proc fly_survey_x {xmin ymin z xmax ymax stepy speed} {

    #    CT_drone1::ReadROSImageUpdateFindings &

    ::maneuver1::set_bounds -- -100 100 -100 100 -2 30 -10 10
    ::maneuver1::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z}] yaw [expr 0] duration 0]
    set y $ymin
    while {$y < $ymax} {
        ::maneuver1::goto  [dict create x [expr {$xmax}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration [expr { ($xmax - $xmin)  / $speed}]]
        set y [expr {$y + $stepy}]
        ::maneuver1::goto  [dict create x [expr {$xmax}] y [expr {$y}] z [expr {$z}] yaw [expr 3.1416] duration 0]
        ::maneuver1::goto  [dict create x [expr {$xmin}] y [expr {$y}] z [expr {$z}] yaw [expr 3.1416] duration [expr { ($xmax - $xmin) / $speed}]]
        set y [expr {$y + $stepy}]
        ::maneuver1::goto  [dict create x [expr {$xmin}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    }
}

proc fly_survey_y {xmin ymin z xmax ymax stepx speed} {

    #    CT_drone1::ReadROSImageUpdateFindings &

    ::maneuver1::set_bounds -- -100 100 -100 100 -2 30 -10 10
    ::maneuver1::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z}] yaw [expr 1.57079] duration 0]
    set x $xmin
    while {$x < $xmax} {
        ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$ymax}] z [expr {$z}] yaw [expr 1.57079] duration [expr { ($ymax - $ymin)  / $speed}]]
        set x [expr {$x + $stepx}]
        ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$ymax}] z [expr {$z}] yaw [expr -1.57079] duration 0]
        ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$ymin}] z [expr {$z}] yaw [expr -1.57079] duration [expr { ($ymax - $ymin) / $speed}]]
        set x [expr {$x + $stepx}]
        ::maneuver1::goto  [dict create x [expr {$x}] y [expr {$ymin}] z [expr {$z}] yaw [expr 1.57079] duration 0]
    }
}