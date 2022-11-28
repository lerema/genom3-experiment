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

genomixd -v -v >"$logdir"/genomixd.log &

tmux \
    new-session "roslaunch quad-cam_gazebo quad-cam.launch world_name:=quad-cam-ar.world" \; \
    split-window -p 66 "tf2-pocolibs -f |& tee -i $logdir/tf2.log" \; \
    split-window -p 50 "CT_drone-pocolibs -f |& tee -i $logdir/CT_drone.log" \; \
    split-window -p 50 "arucotag-pocolibs -f |& tee -i $logdir/arucotag.log" \; \
    split-window -h -t 2 "camgazebo-pocolibs -f |& tee -i $logdir/arucotag.log" \; \
    split-window -h -t 4 "camviz-pocolibs -f |& tee -i $logdir/arucotag.log" \; \
    new-window "optitrack-pocolibs -f |& tee -i $logdir/optitrack.log" \; \
    split-window -p 66 "maneuver-pocolibs -f |& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -simu -cam -tf; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log" \; \
    split-window -h -t 0 "nhfc-pocolibs -f |& tee -i $logdir/nhfc.log" \; \
    split-window -h -t 2 "pom-pocolibs -f |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -f |& tee -i $logdir/rotorcraft.log" \; \
    selectp -t 4
