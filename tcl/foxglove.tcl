package require genomix


# Load parameters / components to connect with FoxgloveStudio
set gps 0
set d435 0
set rotorcraft 0
set color_tracker 0
set aruco 0

# Parse arguments to enable/disable components
foreach arg $argv {
    switch $arg {
        "-gps" {
            set gps 1
        }
        "-d435" {
            set d435 1
        }
        "-rotorcraft" {
            set rotorcraft 1
        }
        "-color_tracker" {
            set color_tracker 1
        }
        "-aruco" {
            set aruco 1
        }
        "-all" {
            set gps 1
            set d435 1
            set rotorcraft 1
            set color_tracker 1
            set aruco 1
        }
    }
}

genomix::connect localhost:8080

genomix1 rpath /home/shasthamsa/drone-experiment/lib/genom/pocolibs/plugins
# genomix1 rpath /home/felix/work/lib/genom/pocolibs/plugins

genomix1 load FoxgloveStudio


if {$gps} {
    genomix1 load gps

    FoxgloveStudio::connect_port {local states/gps_state remote gps/state}
    FoxgloveStudio::connect_port {local gps/gps_info remote gps/info}

    FoxgloveStudio::add_port {port_name gps_state port_type ::FoxgloveStudio::or_pose_estimator_state}
    FoxgloveStudio::add_port {port_name gps_info port_type ::FoxgloveStudio::or_sensor_gps}
}

if {$d435} {
    genomix1 load d435

    FoxgloveStudio::connect_port {local frames/d435 remote d435/frame/raw}
    FoxgloveStudio::add_port {port_name d435 port_type ::FoxgloveStudio::or_frame}
}