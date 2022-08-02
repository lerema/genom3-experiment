#!/usr/bin/env bash
logdir="/tmp/log-$(date +"%Y%m%d-%H%M%S")"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"

this_file_dir="$(dirname "$(file)")"
mkdir -p "$logdir"
rm -f core
h2 init
genomixd -v -v >"$logdir"/genomixd.log &
tmux \
    new-session "/gzserver_entrypoint.sh gzserver $this_file_dir/catkin_ws/src/quad-cam_gazebo/world/quad.world" \; \
    split-window -p 75 "quad-hippo-pocolibs -f --profile=$logdir/quad -F 1 |& tee -i $logdir/quad-fiacre.log" \; \
    split-window -p 50 "script -f -c \"eltclsh -simu; ${SCRIPT_DIR}/end.sh\" $logdir/eltclsh-fiacre.log" \; \
    selectp -t 2
