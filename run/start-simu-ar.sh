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
h2 init -d 500 -p 2000000000
#Source workspace
# shellcheck source=/dev/null

genomixd -v -v >"$logdir"/genomixd.log &

if [ $# -eq 0 ] || [ "$1" == "--python" ]; then
    # Start Gazebo
    user_cmd="echo \"Use this terminal to access the Python script\"; bash ; ${SCRIPT_DIR}/end.sh; sleep 1"
elif [ "$1" == "--tcl" ]; then
    # Start Gazebo
    user_cmd="script -f -c \"eltclsh -simu -cam -tf; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log"
fi

tmux \
    new-session "gazebo ${SCRIPT_DIR}/../gazebo/worlds/single_drone.world --verbose" \; \
    split-window -p 66 "joystick-pocolibs -f |& tee -i $logdir/joystick.log" \; \
    split-window -p 50 "FoxgloveStudio-pocolibs -f -i FoxgloveStudio1 |& tee -i $logdir/FoxgloveStudio.log" \; \
    split-window -p 50 "arucotag-pocolibs -f -i arucotag1 |& tee -i $logdir/arucotag.log" \; \
    split-window -h -t 0 "ColorTracker-pocolibs -f -i ColorTracker1 |& tee -i $logdir/ColorTracker.log" \; \
    split-window -h -t 2 "camgazebo-pocolibs -f -i camgazebo1 |& tee -i $logdir/arucotag.log" \; \
    split-window -h -t 4 "camviz-pocolibs -f -i camviz1 |& tee -i $logdir/arucotag.log" \; \
    new-window "optitrack-pocolibs -f |& tee -i $logdir/optitrack.log" \; \
    split-window -p 66 "maneuver-pocolibs -f -i maneuver1 |& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "$user_cmd" \; \
    split-window -h -t 0 "nhfc-pocolibs -f -i nhfc1 |& tee -i $logdir/nhfc.log" \; \
    split-window -h -t 2 "pom-pocolibs -f -i pom1 |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "rotorcraft-pocolibs -f -i rotorcraft1 |& tee -i $logdir/rotorcraft.log" \; \
    selectp -t 4
