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
    new-session "roslaunch quad-cam_gazebo quad-cam.launch world_name:=quad.world" \; \
    split-window -p 66 "tf2-pocolibs -f |& tee -i $logdir/tf2.log" \; \
    new-window "optitrack-pocolibs -f -p |& tee -i $logdir/optitrack.log" \; \
    split-window -p 66 "maneuver-pocolibs -f -p  |& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -simu; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log" \; \
    split-window -h -t 0 "nhfc-pocolibs -f -p |& tee -i $logdir/nhfc.log" \; \
    split-window -h -t 2 "pom-pocolibs -f -p |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -f -p |& tee -i $logdir/rotorcraft.log" \; \
    selectp -t 4

#    split-window -p 75  "rosrun rviz rviz -d drone.rviz" \; \
#
