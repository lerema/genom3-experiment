#!/usr/bin/env bash
logdir="/tmp/log-$(date +"%Y%m%d-%H%M%S")"
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
DATA_PATH="$SCRIPT_DIR"/../data

# Load environment variables
drone_experiment # Added in install-scripts/setup-experiment.sh

# Source ROS workspace
# shellcheck source=/dev/null
source "${SCRIPT_DIR}"/../catkin_ws/devel/setup.bash
mkdir -p "$logdir"
rm -f core
h2 init -d 500 -p 200000000

genomixd -v -v >"$logdir"/genomixd.log &

if [ $# -eq 0 ] || [ "$1" == "--python" ]; then
    # Start Gazebo
    user_cmd="echo \"Use this terminal to access the Python script\"; bash ; ${SCRIPT_DIR}/end.sh; rm $DATA_PATH/*; sleep 1"
elif [ "$1" == "--tcl" ]; then
    # Start Gazebo
    user_cmd="script -f -c \"eltclsh -simu -cam -tf; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log"
fi

tmux \
    new-session "d435-pocolibs -f -i d4351 |& tee -i $logdir/d435.log" \; \
    split-window -p 66 "tf2-pocolibs -f |& tee -i $logdir/tf2.log" \; \
    split-window -p 50 "CT_drone-pocolibs -f -i CT_drone1 |& tee -i $logdir/CT_drone.log" \; \
    split-window -p 50 "arucotag-pocolibs -f -i arucotag1 |& tee -i $logdir/arucotag.log" \; \
    new-window "optitrack-pocolibs -f |& tee -i $logdir/optitrack.log" \; \
    split-window -p 66 "maneuver-pocolibs -f -i maneuver1 |& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "$user_cmd" \; \
    split-window -h -t 0 "nhfc-pocolibs -f -i nhfc1 |& tee -i $logdir/nhfc.log" \; \
    split-window -h -t 2 "pom-pocolibs -f -i pom1 |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -f -i rotorcraft1 |& tee -i $logdir/rotorcraft.log" \; \
    selectp -t 4
