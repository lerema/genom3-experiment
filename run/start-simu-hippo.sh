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
    new-session "roslaunch quad-cam_gazebo quad-cam2.launch world_file:=quad.world" \; \
    split-window -p 75 "quad-hippo-pocolibs -f --profile=$logdir/quad -F 1 |& tee -i $logdir/quad-fiacre.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -hippo -simu; ${SCRIPT_DIR}/end.sh\" $logdir/eltclsh-fiacre.log" \; \
    selectp -t 2
