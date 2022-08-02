#!/usr/bin/env bash
logdir="/tmp/log-$(date +"%Y%m%d-%H%M%S")"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"

# Source ROS workspace
# shellcheck source=/dev/null
source "${SCRIPT_DIR}"/../catkin_ws/devel/setup.bash
mkdir -p "$logdir"
rm -f core
h2 init
#Source workspace
# shellcheck source=/dev/null
source "$(dirname "$(file)")"/catkin_ws/devel/setup.bash

genomixd -v -v >"$logdir"/genomixd.log &
tmux \
    new-session "roslaunch quad-cam_gazebo quad-cam2.launch" \; \
    split-window -p 50 "tf2-pocolibs -f |& tee -i $logdir/tf2.log" \; \
    new-window "pom-pocolibs -i pom1 -f & tee -i $logdir/pom1.log" \; \
    split-window -p 83 "CT_drone-pocolibs -i CT_drone1 -f |& tee -i $logdir/CT_drone1.log" \; \
    split-window -p 80 "rotorcraft-pocolibs -i rotorcraft1 -f & tee -i $logdir/rotorcraft1.log" \; \
    split-window -p 75 "maneuver-pocolibs -i maneuver1 -f -p  |& tee -i $logdir/maneuver1.log" \; \
    split-window -p 66 "nhfc-pocolibs -i nhfc1 -f & tee -i $logdir/nhfc1.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -simu -cam -tf; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log" \; \
    split-window -h -t 0 "pom-pocolibs -i pom2 -f & tee -i $logdir/pom2.log" \; \
    split-window -h -t 2 "CT_drone-pocolibs -i CT_drone2 -f |& tee -i $logdir/CT_drone2.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -i rotorcraft2 -f & tee -i $logdir/rotorcraft2.log" \; \
    split-window -h -t 6 "maneuver-pocolibs -i maneuver2 -f -p  |& tee -i $logdir/maneuver2.log" \; \
    split-window -h -t 8 "nhfc-pocolibs -i nhfc2 -f & tee -i $logdir/nhfc2.log" \; \
    split-window -h -t 10 "optitrack-pocolibs -f & tee -i $logdir/optitrack.log" \; \
    selectp -t 10
