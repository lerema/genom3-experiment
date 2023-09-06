#!/usr/bin/env bash
logdir="/log/log-$(date +"%Y%m%d-%H%M%S")"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
DATA_PATH="$SCRIPT_DIR"/../data

# Load environment variables
# Source ROS workspace
# shellcheck source=/dev/null
mkdir -p "$logdir"
rm -f core
h2 init -d 500 -p 2000000000

genomixd -v -v >"$logdir"/genomixd.log &

if [ $# -eq 0 ] || [ "$1" == "--python" ]; then
    # Start Gazebo
    user_cmd="echo \"Use this terminal to access the Python script\"; bash ; ${SCRIPT_DIR}/end.sh; rm $DATA_PATH/*; sleep 1"
elif [ "$1" == "--tcl" ]; then
    user_cmd="script -f -c \"eltclsh -robot; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log"
fi

tmux \
    new-session "d435-pocolibs -f|& tee -i $logdir/d435.log" \; \
    split-window -p 66 "tf2-pocolibs -f |& tee -i $logdir/tf2.log" \; \
    split-window -p 50 "FoxgloveStudio-pocolibs -f |& tee -i $logdir/FoxgloveStudio.log" \; \
    split-window -p 50 "arucotag-pocolibs -f |& tee -i $logdir/arucotag.log" \; \
    split-window -h -t 0 "ColorTracker-pocolibs -f |& tee -i $logdir/ColorTracker.log" \; \
    split-window -h -t 2 "joystick-pocolibs -f |& tee -i $logdir/joystick.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -f|& tee -i $logdir/rotorcraft.log" \; \
    new-window "gps-pocolibs -f |& tee -i $logdir/optitrack.log" \; \
    split-window -p 66 "maneuver-pocolibs -f|& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "$user_cmd" \; \
    split-window -h -t 0 "nhfc-pocolibs -f |& tee -i $logdir/nhfc.log" \; \
    split-window -h -t 2 "pom-pocolibs -f |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "${SCRIPT_DIR}/../misc/joystick-control 0 |& tee -i $logdir/joystick_control.log" \; \
    selectp -t 4
