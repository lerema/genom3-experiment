#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
logdir="/log/log-$(date +"%Y%m%d-%H%M%S")"

mkdir -p "$logdir"
rm -f core
h2 init
genomixd -v -v >"$logdir"/genomixd.log &
tmux \
    new-session "nhfc-pocolibs -f -p |& tee -i $logdir/nhfc.log" \; \
    split-window -p 66 "maneuver-pocolibs -f -p  |& tee -i $logdir/maneuver.log" \; \
    split-window -p 50 "script -f -c \"eltclsh; ${SCRIPT_DIR}/end.sh; sleep 1\" $logdir/eltclsh.log" \; \
    split-window -h -t 0 "rotorcraft-pocolibs -f -p |& tee -i $logdir/rotorcraft.log" \; \
    split-window -h -t 2 "pom-pocolibs -f -p |& tee -i $logdir/pom.log" \; \
    split-window -h -t 4 "optitrack-pocolibs -f -p |& tee -i $logdir/optitrack.log" \; \
    selectp -t 4
