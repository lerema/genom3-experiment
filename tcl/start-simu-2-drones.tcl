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
    puts "Cannot guess who you are..."
    exit 2
} else {
    $g rpath  $::env(DRONE_VV_PATH)/lib/genom/$mw/plugins/
}

$g load optitrack
$g load pom -i pom1 as pom1
$g load maneuver -i maneuver1 as maneuver1
$g load rotorcraft -i rotorcraft1 as rotorcraft1
$g load nhfc -i nhfc1 as nhfc1
$g load pom -i pom2 as pom2
$g load maneuver -i maneuver2 as maneuver2
$g load rotorcraft -i rotorcraft2 as rotorcraft2
$g load nhfc -i nhfc2 as nhfc2

if {$use_hippo} {
    $g load hippo
}

if {$use_tf} {
    $g load tf2
}

if {$cam} {
    $g load CT_drone -i CT_drone1 as CT_drone1
    $g load CT_drone -i CT_drone2 as CT_drone2
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
        optitrack::connect { host marey host_port 1510 mcast 239.192.168.30 mcast_port 1511 }
    } else {
        optitrack::connect { host localhost host_port 1509 mcast "" mcast_port "" }
    }
    after 1000


    # POM
    pom1::connect_port { local measure/imu remote rotorcraft1/imu }
    pom2::connect_port { local measure/imu remote rotorcraft2/imu }
    if {$robot} {
        pom::connect_port { local measure/mocap remote optitrack/bodies/QR_1 }
    } else {
        pom1::connect_port { local measure/mag remote rotorcraft1/mag }
        pom1::connect_port { local measure/mocap remote optitrack/bodies/QR1 }

        pom2::connect_port { local measure/mag remote rotorcraft2/mag }
        pom2::connect_port { local measure/mocap remote optitrack/bodies/QR2 }
    }

    pom1::add_measurement { port imu x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
    pom1::add_measurement { port mocap x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}

    pom2::add_measurement { port imu x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
    pom2::add_measurement { port mocap x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}

    if {$simu} {
        pom1::add_measurement { port mag x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
        pom1::set_mag_field { magdir { x 23.816e-6 y -0.41e-6 z -39.829e-6 } }

        pom2::add_measurement { port mag x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
        pom2::set_mag_field { magdir { x 23.816e-6 y -0.41e-6 z -39.829e-6 } }
    }

    pom1::set_history_length { history_length 0.5 }
    pom2::set_history_length { history_length 0.5 }


    # MANEUVER
    maneuver1::connect_port { local state remote pom1/frame/robot }
    maneuver2::connect_port { local state remote pom2/frame/robot }
    if {$simu} {
        maneuver1::set_velocity_limit {v 2 w 1}
        maneuver2::set_velocity_limit {v 2 w 1}
    }


    # ROTORCRAFT
    if {$robot} {
        rotorcraft::connect { serial /dev/ttyUSB0 baud 500000 }
    } else {
        rotorcraft1::connect { serial /tmp/pty-quad1 baud 500000 }
        rotorcraft2::connect { serial /tmp/pty-quad2 baud 500000 }
    }
    rotorcraft1::connect_port { local rotor_input remote nhfc1/rotor_input }
    rotorcraft2::connect_port { local rotor_input remote nhfc2/rotor_input }

    if {$robot} {
        rotorcraft::set_sensor_rate { rate { imu 1000 mag 0 motor 16 battery 1 } }

        set imu_calib [read [open imu_calib.txt r]]
        rotorcraft::set_imu_calibration $imu_calib
    } else {
        rotorcraft1::set_sensor_rate { rate { imu 1000 mag 50 motor 16 battery 1 } }
        rotorcraft2::set_sensor_rate { rate { imu 1000 mag 50 motor 16 battery 1 } }
    }

    # NHFC
    # nhfc1::set_mass $m
    # nhfc1::set_geom $G $J

    # nhfc2::set_mass $m
    # nhfc2::set_geom $G $J

    nhfc1::set_gtmrp_geom {}
    nhfc2::set_gtmrp_geom {}

    if {$robot} {
        nhfc::set_servo_gain { gain {Kpxy 20 Kpz 25 Kqxy 3 Kqz 0.3 Kvxy 15
            Kvz 20  Kwxy 0.3 Kwz 0.03 Kixy 0.5  Kiz 3 } }
        } else {
        nhfc1::set_servo_gain { gain {Kpxy 20 Kpz 20 Kqxy 3  Kqz 0.3
            Kvxy 8 Kvz 8  Kwxy 0.3 Kwz 0.03 Kixy 0  Kiz 0 } }
            nhfc2::set_servo_gain { gain {Kpxy 20 Kpz 20 Kqxy 3  Kqz 0.3
                Kvxy 8 Kvz 8  Kwxy 0.3 Kwz 0.03 Kixy 0  Kiz 0 } }
            }
            nhfc1::set_emerg { emerg { descent 1.2 dx 0.1 dq 1 dv 0.3 dw 1 } }
            nhfc1::connect_port { local state remote pom1/frame/robot }
            nhfc1::connect_port { local reference remote maneuver1/desired }

            nhfc2::set_emerg { emerg { descent 1.2 dx 0.1 dq 1 dv 0.3 dw 1 } }
            nhfc2::connect_port { local state remote pom2/frame/robot }
            nhfc2::connect_port { local reference remote maneuver2/desired }

            if {$simu} {
            nhfc1::set_wlimit { wmin 16 wmax 100}
            nhfc2::set_wlimit { wmin 16 wmax 100}
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

            CT_drone2::Set_my_r 255
            CT_drone2::Set_my_g 238
            CT_drone2::Set_my_b 218
            CT_drone2::Set_my_seuil 40

            CT_drone2::connect_port { local Pose remote pom2/frame/robot }

            CT_drone2::SetCameraImageTopicName /quad2/down_camera_link/down_raw_image
            CT_drone2::SetCameraInfoTopicName /quad2/down_camera_link/down_info_image
        }

        if {$use_tf} {
            #tf2
            tf2::connect_port Poses/drone1 pom1/frame/robot
            tf2::connect_port Poses/drone1_pos pom1/frame/robot

            tf2::connect_port Poses/drone2 pom2/frame/robot
            tf2::connect_port Poses/drone2_pos pom2/frame/robot

            if {$cam} {
                tf2::connect_port OccupancyGrids/og1 CT_drone1/OccupancyGrid
                tf2::connect_port OccupancyGrids/og2 CT_drone2/OccupancyGrid
            }

            # tf2::Init

            # tf2::PublishStaticTF -- base drone1 0 0 0 0 0 0
            # tf2::PublishStaticTF -- base drone2 0 0 0 0 0 0
            # tf2::PublishStaticTF -- down_camera_link drone 0 0 0 0 1.5708 0

            tf2::AddDynamicTF drone1 world 10 1
            tf2::AddDynamicPosTF drone1_pos world 10 1

            tf2::AddDynamicTF drone2 world 10 1
            tf2::AddDynamicPosTF drone2_pos world 10 1

            tf2::AddOdometry {name drone1}
            tf2::AddTwistFromPose {name drone1 frame drone1_pos topic drone1_twist ms_period 10}
            tf2::AddWrenchFromPose {name drone1 frame drone1_pos topic drone1_wrench ms_period 10}


            tf2::AddOdometry {name drone2}
            tf2::AddTwistFromPose {name drone2 frame drone2_pos topic drone2_twist ms_period 10}
            tf2::AddWrenchFromPose {name drone2 frame drone2_pos topic drone2_wrench ms_period 10}

            if {$cam} {
                tf2::PublishStaticTF -- og1 world 0 0 0 0 0 0
                tf2::AddOccupancyGrid og1 og1 og1 50

                tf2::PublishStaticTF -- og2 world 0 0 0 0 0 0
                tf2::AddOccupancyGrid og2 og2 og2 50
            }
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

        pom::log_state ${dir_logs}/pom.log $log_rate_d
        pom::log_measurements ${dir_logs}/pom-measurements.log
        rotorcraft::log ${dir_logs}/rotorcraft.log $log_rate_d
        nhfc::log ${dir_logs}/nhfc.log $log_rate_d
        maneuver::log ${dir_logs}/maneuver.log $log_rate_d
        optitrack::set_logfile ${dir_logs}/optitrack.log
    }

    # SETUP

    proc setup {} {
        setup1
        setup2
    }


    proc setup1 {} {
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

    proc setup2 {} {
        global cam
        #    start_log
        rotorcraft2::start
        maneuver2::set_bounds -- -100 100 -100 100 -2 30 -10 10
        maneuver2::set_current_state
        nhfc2::set_current_position
        rotorcraft2::servo &
        maneuver2::take_off { height 0.15 duration 0 }
        nhfc2::servo &

        if {$cam} {
            CT_drone2::PublishOccupancyGrid &
        }

    }

    proc restart {} {
        rotorcraft::start
        maneuver::set_current_state
        nhfc::set_current_position
        rotorcraft::servo &
        maneuver::take_off { height 0.15 duration 0 }
        nhfc::servo &
    }

    proc land {} {
        global robot

        if {$robot} {
            maneuver::take_off {height 0.15 duration 0 }
        } else {
            maneuver::take_off {height 0.05 duration 0 }
        }
        rotorcraft::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
    }

    proc stop {} {
        rotorcraft::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
        rotorcraft::stop
    }

    proc stop_all {} {
        rotorcraft::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
        rotorcraft::stop
        nhfc::stop
        maneuver::stop
    }

    # STOP LOGS
    proc stop_log {} {
        pom::log_stop
        rotorcraft::log_stop
        nhfc::log_stop
        maneuver::log_stop
    }

    proc kill_all {} {
        global use_tf
        global cam

        optitrack::kill

        pom1::kill
        maneuver1::kill
        rotorcraft1::kill
        nhfc1::kill

        pom2::kill
        maneuver2::kill
        rotorcraft2::kill
        nhfc2::kill

        if {$use_tf} {
            tf2::kill
        }
        if {$cam} {
            CT_drone1::kill
            CT_drone2::kill
        }
    }

    proc track_maneuver {x y z} {
        ::maneuver2::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0] &
        nhfc2::connect_port { local reference remote maneuver2/desired }
    }

    proc track_CT_drone {} {
        nhfc2::connect_port { local reference remote CT_drone2/TargetPose }
        maneuver2::stop
    }

    if {$simu} {
        proc move {} {
            ::maneuver::set_bounds -- -20 20 -20 20 -2 10 -3.14 3.14
            ::maneuver::take_off 3 0
            after 1000
            ::maneuver::waypoint -- 1.5 0 4 0     0 0 0 0   0 0 0  0
            ::maneuver::waypoint -- 1 -0.5 1 3.14   0 0 0 0   0 0 0  0
            ::maneuver::waypoint -- 1 -0.5 4 0    0 0 0 0   0 0 0  0
            ::maneuver::waypoint --  1.5 0 4 0    0 0 0 0   0 0 0  0
            ::maneuver::wait
            after 1000
            ::maneuver::take_off 2 0
        }


        proc move2 {} {
            ::maneuver::set_current_state

            ::maneuver::set_bounds -- -100 100 -100 100 -2 30 -10 10
            ::maneuver::take_off 3.0 0
            after 1000
            ::maneuver::waypoint -- 10.0 0 5 0     0 0 0 0   0 0 0  0
            ::maneuver::waypoint -- 1 -8.5 8 3.14   0 0 0 0   0 0 0  0
            ::maneuver::waypoint -- 20 -25 7 0    0 0 0 0   0 0 0  0
            ::maneuver::waypoint --  30 0 8 0    0 0 0 0   0 0 0  0
            ::maneuver::waypoint --  0 0 6 0    0 0 0 0   0 0 0  0
            ::maneuver::wait
            after 1000
            ::maneuver::take_off 2.0 0
        }
    }

    proc carre {} {
        global robot

        if {$robot} {
            ::maneuver::goto {x 0.5 y 0.5 z 0.7 yaw 2 duration 0}
            ::maneuver::goto {x 0.5 y -0.5 z 0.5 yaw -2 duration 0}
            ::maneuver::goto {x -0.5 y -0.5 z 0.7 yaw 2 duration 0}
            ::maneuver::goto {x -0.5 y 0.5 z 0.3 yaw -2 duration 0}
        } else {
            ::maneuver::set_bounds -- -10 10 -10 10 -1 10 -4 4
            ::maneuver::goto {x 6 y 6 z 1.7 yaw 2 duration 0}
            ::maneuver::goto {x 6 y -6 z 2.7 yaw -2 duration 0}
            ::maneuver::goto {x -6 y -6 z 0.7 yaw 2 duration 0}
            ::maneuver::goto {x -6 y 6 z 3.7 yaw -2 duration 0}
        }
    }

    proc draw_H {x y z width height} {
        ::tf2::SetOdometryDisplay drone 0
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 1
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height/2.0}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height/2.0}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    }

    proc draw_P {x y z width height} {
        ::tf2::SetOdometryDisplay drone 0
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 1
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height*2.0/3.0}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*2.0/3.0}] yaw [expr 0] duration 0]
    }

    proc draw_O {x y z width height} {
        ::tf2::SetOdometryDisplay drone 0
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 1
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x + $width}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    }

    proc draw_I {x y z width height} {
        ::tf2::SetOdometryDisplay drone 0
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 1
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 0
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*1.2}] yaw [expr 0] duration 0]
        ::tf2::SetOdometryDisplay drone 1
        ::maneuver::goto  [dict create x [expr {$x}] y [expr {$y}] z [expr {$z + $height*1.2}] yaw [expr -3.14] duration 0]
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
            fly_around1
            fly_around2
        }

        proc fly_around1 {} {
            ::maneuver1::set_bounds -- -10 10 -10 10 -1 10 -4 4
            while 1 {
                if { [catch {maneuver1::goto [dict create x [expr {rand()*20 - 10}] y [expr {rand()*20 - 10}] z [expr {rand()*10 + 0.2}] yaw [expr {rand()*6.26 -3.14 }] duration 0]} result]} {
                    puts $result
                    return
                }
            }
        }

        proc fly_around2 {} {
            ::maneuver2::set_bounds -- -10 10 -10 10 -1 10 -4 4
            while 1 {
                if { [catch {maneuver2::goto [dict create x [expr {rand()*20 - 10}] y [expr {rand()*20 - 10}] z [expr {rand()*10 + 0.2}] yaw [expr {rand()*6.26 -3.14 }] duration 0]} result]} {
                    puts $result
                    return
                }
            }
        }
    }


    proc fly_survey_x {xmin ymin z xmax ymax stepy speed} {

        #    CT_drone::ReadROSImageUpdateFindings &

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

        #    CT_drone::ReadROSImageUpdateFindings &

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



    proc track_surveyer {xmin ymin z xmax ymax stepy speed} {
        ::maneuver1::set_bounds -- -100 100 -100 100 -2 30 -10 10
        ::maneuver2::set_bounds -- -100 100 -100 100 -2 30 -10 10

        CT_drone1::ReadROSImageUpdateFindings &
        CT_drone2::ReadROSImageFindTarget $z &

        ::maneuver1::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z}] yaw [expr 0] duration 0]
        ::maneuver2::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z * 2}] yaw [expr 0] duration 0]

        after 4000

        nhfc2::connect_port { local reference remote CT_drone2/TargetPose }
        maneuver2::stop

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
