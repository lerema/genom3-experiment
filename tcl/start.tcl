# tcl script to start the drone experiment, sumulation, indoor and outdoor.
#

set use_hippo 0
set simu -1
set cam 0
set gps 0
set robot -1
set use_tf 0
set mw pocolibs
set log_date [string trim [exec date +"%Y%m%d-%H%M%S"] "\""]
set log_dir /tmp/log-${log_date}

if { $::argc > 0 } {
    set i 1
    foreach arg $::argv {
        #puts "argument $i is $arg"
        #incr i
	if { $log_dir == ""} {
	    set log_dir $arg
	    file mkdir $log_dir
	    continue
	}
	switch -exact $arg {
	    -ros {
		set mw ros
	    }
	    -tf {
		set use_tf 1
	    }
	    -log_dir {
		set log_dir ""
	    }
	    -simu {
		set simu 1
	    }
	    -gps {
		set gps 1
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

puts "simu: $simu, cam: $cam, use_hippo: $use_hippo, use_tf: $use_tf, mw: $mw, log_dir: $log_dir"

# UAV PARAMETERS

# GENOMIX & RPATH
#set host localhost

package require genomix
#set g [genomix::connect localhost:8080]

set g [genomix::localhost]


if {$robot} {
    # $g rpath /home/shasthamsa/drone-experiment/lib/genom/pocolibs/plugins/
    $g rpath /home/felix/work/lib/genom/$mw/plugins/
} elseif {$::env(USER) eq "felix"} {
    $g rpath $::env(HOME)/work/lib/genom/$mw/plugins/
    $g rpath $::env(HOME)/openrobots/lib/genom/$mw/plugins/
} elseif {$::env(USER) eq "docker"} {
    $g rpath  /opt/openrobots/lib/genom/$mw/plugins/
} else {
    set devel_path /home/mjacquet/RIS/genom_devel
    $g rpath ${devel_path}/lib/genom/pocolibs/plugins/
}

if {$gps} {
    $g load gps
} else {
    $g load optitrack
}

$g load pom
#$g load d435
$g load maneuver
$g load joystick
$g load FoxgloveStudio
$g load rotorcraft
$g load nhfc

if {$use_hippo} {
    $g load hippo
}

if {$use_tf} {
    $g load tf2
}

if {$cam} {
    if {$simu} {
	$g load CT_drone
    } else {
	$g load d435
	$g load ColorTracker
	$g load arucotag
    }
}

proc init_gps {} {
    global gps
    global log_dir

    if {$gps} {
	
	# set the experiment in local 0,0,0 
#	set info [gps::info]
#	lassign [dict get $info info llh] latitude lat longitude long height h
	gps::set_reference {reference  {latitude 43.561685463999993 longitude 1.4769517219999999 height 194.02610000000001}}
# non-ground pose    {latitude 43.561690700999996 longitude 1.4769556049999999 height 194.9186}

	#start log now to avoir the set_reference jump...
	gps::log ${log_dir}/gps.log 1
    
	pom::set_mag_field { magdir {x 2.4016311504777697e-05 y -9.5490819586733245e-07 z -3.8850558715353451e-05}}

	pom::add_measurement {port gps x -0. y 0. z 0. roll 0. pitch 0. yaw 0.}
	pom::add_measurement { port mag x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
	pom::add_measurement { port imu x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}

    }
}

proc init_cam {} {
    global cam

    if {$cam} {
        d435::set_fps 30
        d435::set_size {size {w 640 h 480}}
        d435::connect 832112070817
	
        # ColorTracker
        ColorTracker::set_color {color {r 70 g 70 b 20 threshold 40}}
    	ColorTracker::set_object_size {object_width 0.195 object_height 0.3}
        ColorTracker::set_focal_length 1092
        ColorTracker::set_distance_threshold 1.0
	
        ColorTracker::connect_port {local DronePose remote pom/frame/robot}
        ColorTracker::connect_port {local Frame remote d435/frame/raw}
        ColorTracker::connect_port {local Intrinsics remote d435/intrinsics}
        ColorTracker::connect_port {local Extrinsics remote d435/extrinsics}
	
        FoxgloveStudio::connect_port "frames/d435/raw" "d435/frame/raw"
        FoxgloveStudio::add_port "d435/raw" "::FoxgloveStudio::or_sensor_frame"
        # FoxgloveStudio::connect_port "frames/ct" "ColorTracker/output"
        # FoxgloveStudio::add_port "ct" "::FoxgloveStudio::or_sensor_frame"
        
        after 2000
        FoxgloveStudio::start_foxglove_server
    }    
}

proc start_track {} {
    global cam

    if {$cam} {
        ColorTracker::color_track &
        ColorTracker::perform_tracking 1
    }
}

proc init {} {
    global gps
    global use_tf
    global simu
    global mw
    global cam
    global robot

    # OPTITRACK
    if {$robot} {
	if {$gps} {
	    # gps::set_timestamp_mode {tsmode ::gps::clock_utc}

	    # gps::connect /dev/ttyACM2 115200
	    gps::connect /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 115200
	    gps::connect_rtk gps-base 8083
	} else {
	    optitrack::connect { host muybridge host_port 1510 mcast 239.192.168.30 mcast_port 1511 }
	}
    } else {
	optitrack::connect { host localhost host_port 1509 mcast "" mcast_port 0 }
    }
    after 1000
    
    
    # POM
    pom::connect_port { local measure/imu remote rotorcraft/imu }
    if {$robot} {
	pom::connect_port { local measure/mag remote rotorcraft/mag }
	if {$gps} {
	    pom::connect_port {local measure/gps remote gps/state}
	} else {
	    pom::connect_port { local measure/mocap remote optitrack/bodies/Lerema }
	}
    } else {
	pom::connect_port { local measure/mag remote rotorcraft/mag }
	pom::connect_port { local measure/mocap remote optitrack/bodies/QR }
    }

    
    if {!$gps} {
	pom::add_measurement { port mocap x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
	pom::add_measurement { port imu x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
    }

    if {$simu} {
	pom::add_measurement { port mag x 0. y 0. z 0. roll 0. pitch 0. yaw 0.}
	pom::set_mag_field { magdir { x 23.816e-6 y -0.41e-6 z -39.829e-6 } }
    }

    pom::set_history_length { history_length 0.5 }
    
    
    # MANEUVER
    maneuver::connect_port { local state remote pom/frame/robot }
    if {$simu} {
	maneuver::set_velocity_limit {v 2 w 1}
    } else {
	maneuver::set_velocity_limit {v 0.5 w 0.5}
    }
    
    
    # if {$mw eq "ros"} {
    # 	js_maneuver::connect_port Joystick joystick/device/Logitech_Gamepad_F710
    # 	after 1000
    # } else {
    # 	js_maneuver::connect_port Joystick joystick/device/Logitech
    # }


    # js_maneuver::connect_service ext_velocity maneuver/ext_velocity

    # ROTORCRAFT
    if {$robot} {
	rotorcraft::connect { serial chimera-5 baud 0 }
	after 2000
    } else {
	if {$cam} {
	    rotorcraft::connect { serial /tmp/pty-mrsim-quadrotor-cam baud 500000 }
	} else {
	    rotorcraft::connect { serial /tmp/pty-mrsim-quadrotor baud 0 }
	}
    }
    

    if {$robot} {
	rotorcraft::set_sensor_rate { rate { imu 1000 mag 100 motor 16 battery 1 } }
	
	set imu_calib [read [open calibrations/imu_calib.txt r]]
	rotorcraft::set_imu_calibration $imu_calib
	# rotorcraft::set_zero 4
	rotorcraft::set_zero_velocity 4
	rotorcraft::set_imu_filter {
	    gfc {0 20 1 20 2 20}
	    afc {0 5 1 5 2 5}
	    mfc {0 20 1 20 2 20}
	}

	

    } else {
	rotorcraft::set_sensor_rate { rate { imu 1000 mag 0 motor 20 battery 1 } }
	rotorcraft::set_imu_filter {
	    gfc {0 20 1 20 2 20}
	    afc {0 5 1 5 2 5}
	    mfc {0 20 1 20 2 20}
	}
    }
    rotorcraft::connect_port { local rotor_input remote nhfc/rotor_input }


    
    # NHFC

    if {$robot} {
	nhfc::set_gtmrp_geom {rz -1 mass 1.845 cf 7.8e-4 ct 1e-5 }
	nhfc::set_emerg { emerg { descent 1.2 dx 0.1 dq 1 dv 0.3 dw 1 } }
	nhfc::set_wlimit { wmin 15 wmax 110}
    } else {
	::nhfc::set_gtmrp_geom {
	    rotors 4 cx 0 cy 0 cz 0 armlen 0.23 mass 1.5
	    rx 0 ry 0 rz -1 cf 6.5e-4 ct 1e-5
	}
	::nhfc::set_emerg {emerg {descent 0.1 dx 0.5 dq 1 dv 3 dw 3}}
    }
    
    if {$robot} {
	::nhfc::set_saturation {sat {x 0.2 v 0.1 ix 0}}
	nhfc::set_servo_gain { gain {Kpxy 20 Kpz 20 Kqxy 3 Kqz 0.3 Kvxy 15
	    Kvz 15  Kwxy 0.3 Kwz 0.03 Kixy 0.5  Kiz 3 } }
    } else {
	::nhfc::set_saturation {sat {x 1 v 1 ix 0}}
	::nhfc::set_servo_gain {
	    gain {
		Kpxy 5 Kpz 5 Kqxy 4 Kqz 0.1 Kvxy 6 Kvz 6 Kwxy 1 Kwz 0.1
		Kixy 0. Kiz 0.
	    }
	}
	::nhfc::set_control_mode {att_mode ::nhfc::tilt_prioritized}

    }
    
    nhfc::connect_port { local state remote pom/frame/robot }
    nhfc::connect_port { local reference remote maneuver/desired }

#    if {$simu} {
#	nhfc::set_wlimit { wmin 16 wmax 100}
#    }

    #CT_drone
    if {$simu && $cam} {
	CT_drone::Set_my_r 5
	CT_drone::Set_my_g 5
	CT_drone::Set_my_b 255
	CT_drone::Set_my_seuil 40

	CT_drone::connect_port { local Pose remote pom/frame/robot }

	CT_drone::SetCameraImageTopicName /quad1/down_camera_link/down_raw_image
	CT_drone::SetCameraInfoTopicName /quad1/down_camera_link/down_info_image
    }
    
    if {$use_tf} {
	#tf2
	tf2::connect_port Poses/drone pom/frame/robot
#	tf2::connect_port Poses/drone_pos pom/frame/robot
#	tf2::connect_port OccupancyGrids/og CT_drone/OccupancyGrid

#	tf2::Init
	
	tf2::PublishStaticTF -- base drone 0 0 0 0 0 0
	# f2::PublishStaticTF -- down_camera_link drone 0 0 0 0 1.5708 0

	tf2::AddDynamicTF drone drone world 10 1
	tf2::AddDynamicPosTF drone_pos drone world 10 1

	tf2::AddOdometry {frame_name drone}
	tf2::AddTwistFromPose {name drone frame drone_pos topic drone_twist ms_period 10}
	tf2::AddWrenchFromPose {name drone frame drone_pos topic drone_wrench ms_period 10}

#	tf2::PublishStaticTF -- og world 0 0 0 0 0 0
#	tf2::AddOccupancyGrid og og og 50

    }
}

# LOGS
proc start_log {} {
    global simu
    global gps
    global robot
    global log_dir
    
    file mkdir $log_dir
    

    set log_rate_d 1

    pom::log_state ${log_dir}/pom.log $log_rate_d
    pom::log_measurements ${log_dir}/pom-measurements.log
    rotorcraft::log ${log_dir}/rotorcraft.log $log_rate_d
    nhfc::log ${log_dir}/nhfc.log $log_rate_d
    maneuver::log ${log_dir}/maneuver.log $log_rate_d
    if {!$gps} {
	optitrack::set_logfile ${log_dir}/optitrack.log
    }
}

set pi 3.1415926535897932
set pi_2 1.57079632679

proc get_frame_robot_rpy {} {
    global pi
    
    set pos [pom::frame robot]
    lassign [dict get $pos frame att] w qw x qx y qy z qz
    set yaw [expr {atan2(2 * ($qw*$qz + $qx*$qy), 1 - 2 * ($qy*$qy + $qz*$qz))}]
    set pitch [expr {asin(2 * ($qw*$qy - $qz*$qx))}]
    set roll [expr {atan2(2 * ($qw*$qx + $qy*$qz), 1 - 2 * ($qx*$qx + $qy*$qy))}]
    
    puts [format "Roll: %g Pitch: %g Yaw: %g (deg)" [expr $roll*180/$pi] [expr $pitch*180/$pi] [expr $yaw*180/$pi]]
    
}

proc get_frame_robot_pos {} {
    global pi

    set pos [pom::frame robot]

    lassign [dict get $pos frame pos] x x1 y y1 z z1
    lassign [dict get $pos frame vel] vx vx1 vy vy1 vz vz1

    puts [format "x: %g y: %g z: %g (m)" $x1 $y1 $z1]
    puts [format "vx: %g vy: %g vz: %g (m/s)" $vx1 $vy1 $vz1]

    lassign [dict get $pos frame att] w qw x qx y qy z qz
    set yaw [expr {atan2(2 * ($qw*$qz + $qx*$qy), 1 - 2 * ($qy*$qy + $qz*$qz))}]
    set pitch [expr {asin(2 * ($qw*$qy - $qz*$qx))}]
    set roll [expr {atan2(2 * ($qw*$qx + $qy*$qz), 1 - 2 * ($qx*$qx + $qy*$qy))}]
    
    lassign [dict get $pos frame avel] wx wx wy wy wz wz
    
    puts [format "Roll: %g Pitch: %g Yaw: %g (deg)" [expr $roll*180/$pi] [expr $pitch*180/$pi] [expr $yaw*180/$pi]]
    puts [format "VRoll: %g VPitch: %g VYaw: %g (deg/s)" [expr $wx*180/$pi] [expr $wy*180/$pi] [expr $wz*180/$pi]]

}


proc get_frame_robot_xyz {} {
    set pos [pom::frame robot]
    lassign [dict get $pos frame pos] x x1 y y1 z z1
    puts [format "x: %g y: %g z: %g" $x1 $y1 $z1]
}

proc servo_cb {mod rqid} {
    catch { $rqid result } m
    
    puts stderr "$mod servo_cb: $m."
}

# SETUP
proc setup {} {
    global cam
    global simu

    start_log

    rotorcraft::start

    nhfc::set_current_position 
#    nhfc::set_position  -- -1.6281 0.10 1.8 0 
    nhfc::servo & servo_cb nhfc

    rotorcraft::servo & servo_cb rotorcraft

    maneuver::set_bounds -- -100 100 -100 100 -2 30 -10 10
    maneuver::set_current_state
    maneuver::take_off { height 0.25 duration 0 }

    if {$simu && $cam} {
	CT_drone::PublishOccupancyGrid &  
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
        rotorcraft::stop
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
    global gps

    pom::log_stop
    nhfc::log_stop
    maneuver::log_stop
    if {$gps} {
	gps::log_stop
    } else {
	rotorcraft::log_stop
    }
}

proc kill_all {} {
    global use_tf
    global cam
    global simu
    global gps

    if {$gps} {
	gps::kill
    } else {
	optitrack::kill
    }
    pom::kill
    maneuver::kill
    rotorcraft::kill
    joystick::kill
#    js_maneuver::kill
    nhfc::kill
    if {$use_tf} {
	tf2::kill
    }
    if {$cam} {
	if {$simu} {
	    CT_drone::kill
	} else {
	    ColorTracker::kill
	    arucotag::kill
	    d435::kill
	}
    }
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
	::maneuver::goto {x 1.0 y 1.0 z 0.4 yaw -1.4 duration 0}
	after 1000
	::maneuver::goto {x 1.0 y -1.0 z 0.5 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x -1.0 y -1.0 z 0.3 yaw 1.4 duration 0}
	after 1000
	::maneuver::goto {x -1.0 y 1.0 z 0.5 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x -0. y 0. z 0.5 yaw 0 duration 0}
    } else {
	::maneuver::set_bounds -- -10 10 -10 10 -1 10 -4 4
	::maneuver::goto {x 6 y 6 z 1.7 yaw 2 duration 0}
	::maneuver::goto {x 6 y -6 z 2.7 yaw -2 duration 0}
	::maneuver::goto {x -6 y -6 z 0.7 yaw 2 duration 0}
	::maneuver::goto {x -6 y 6 z 3.7 yaw -2 duration 0}
    }
}

proc arena {} {
    global gps

    if {$gps} {
	::maneuver::goto {x -2 y 11 z 1.0 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x -10 y 5 z 0.8 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x 0 y -8 z 1.0 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x 8 y -2 z 0.8 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x -2 y 11 z 1.0 yaw 0 duration 0}
	after 1000
	::maneuver::goto {x 0 y 0 z 0.8 yaw 0 duration 0}
	after 1000
	maneuver::take_off {height 0.15 duration 0 }
    } else {
	puts  "The inside arena is not big enough... use this procedure outside only."
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
    ::maneuver::set_bounds -- -10 10 -10 10 -1 10 -4 4
    while 1 {
	if { [catch {maneuver::goto [dict create x [expr {rand()*20 - 10}] y [expr {rand()*20 - 10}] z [expr {rand()*10 + 0.2}] yaw [expr {rand()*6.26 -3.14 }] duration 0]} result]} {
	    puts $result
	    return
	}
    }
}
}


proc fly_survey_x {xmin ymin z xmax ymax stepy speed} {

#    CT_drone::ReadROSImageUpdateFindings &

    ::maneuver::set_bounds -- -100 100 -100 100 -2 30 -10 10
    ::maneuver::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z}] yaw [expr 0] duration 0]
    set y $ymin
    while {$y < $ymax} {
	::maneuver::goto  [dict create x [expr {$xmax}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration [expr { ($xmax - $xmin)  / $speed}]]
	set y [expr {$y + $stepy}] 
	::maneuver::goto  [dict create x [expr {$xmax}] y [expr {$y}] z [expr {$z}] yaw [expr 3.1416] duration 0]
	::maneuver::goto  [dict create x [expr {$xmin}] y [expr {$y}] z [expr {$z}] yaw [expr 3.1416] duration [expr { ($xmax - $xmin) / $speed}]]
	set y [expr {$y + $stepy}] 
	::maneuver::goto  [dict create x [expr {$xmin}] y [expr {$y}] z [expr {$z}] yaw [expr 0] duration 0]
    }
    ::maneuver::goto  0 0 1 0 0
    maneuver::take_off {height 0.15 duration 0 }
}

proc fly_survey_y {xmin ymin z xmax ymax stepx speed} {

#    CT_drone::ReadROSImageUpdateFindings &

    ::maneuver::set_bounds -- -100 100 -100 100 -2 30 -10 10
    ::maneuver::goto  [dict create x [expr {$xmin}] y [expr {$ymin}] z [expr {$z}] yaw [expr 1.57079] duration 0]
    set x $xmin
    while {$x < $xmax} {
	::maneuver::goto  [dict create x [expr {$x}] y [expr {$ymax}] z [expr {$z}] yaw [expr 1.57079] duration [expr { ($ymax - $ymin)  / $speed}]]
	set x [expr {$x + $stepx}] 
	::maneuver::goto  [dict create x [expr {$x}] y [expr {$ymax}] z [expr {$z}] yaw [expr -1.57079] duration 0]
	::maneuver::goto  [dict create x [expr {$x}] y [expr {$ymin}] z [expr {$z}] yaw [expr -1.57079] duration [expr { ($ymax - $ymin) / $speed}]]
	set x [expr {$x + $stepx}] 
	::maneuver::goto  [dict create x [expr {$x}] y [expr {$ymin}] z [expr {$z}] yaw [expr 1.57079] duration 0]
    }
}
