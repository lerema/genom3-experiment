#!/usr/bin/env bash
logdir="/tmp/log-$(date +"%Y%m%d-%H%M%S")"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"

# Source ROS workspace
# shellcheck source=/dev/null
source "${SCRIPT_DIR}"/../catkin_ws/devel/setup.bash
mkdir -p "$logdir"
rm -f core
h2 init
genomixd -v -v >"$logdir"/genomixd.log &
tmux \
    new-session "roslaunch quad-cam_gazebo quad-cam2.launch world_name:=quad2.world" \; \
    split-window -p 50 "tf2-pocolibs -i tf2 -f |& tee -i $logdir/tf2.log" \; \
    new-window "pom-pocolibs -i pom1 -f & tee -i $logdir/pom1.log" \; \
    split-window -p 80 "rotorcraft-pocolibs -i rotorcraft1 -f & tee -i $logdir/rotorcraft1.log" \; \
    split-window -p 75 "maneuver-pocolibs -i maneuver1 -f |& tee -i $logdir/maneuver1.log" \; \
    split-window -p 66 "nhfc-pocolibs -i nhfc1 -f & tee -i $logdir/nhfc1.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -simu -tf; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log" \; \
    split-window -h -t 0 "pom-pocolibs -i pom2 -f & tee -i $logdir/pom2.log" \; \
    split-window -h -t 2 "rotorcraft-pocolibs -i rotorcraft2 -f & tee -i $logdir/rotorcraft2.log" \; \
    split-window -h -t 4 "maneuver-pocolibs -i maneuver2 -f |& tee -i $logdir/maneuver2.log" \; \
    split-window -h -t 6 "nhfc-pocolibs -i nhfc2 -f & tee -i $logdir/nhfc2.log" \; \
    split-window -h -t 8 "optitrack-pocolibs -f & tee -i $logdir/optitrack.log" \; \
    selectp -t 8

#
