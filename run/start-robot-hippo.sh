#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "$(realpath "$0")")" && pwd)"
logdir="/log/log-$(date +"%Y%m%d-%H%M%S")"

mkdir -p "$logdir"
rm -f core
h2 init
genomixd -v -v >"$logdir"/genomixd.log &
tmux \
    new-session "quad-hippo-pocolibs -f --profile=$logdir/quad -p -F 1 |& tee -i $logdir/lerema-fiacre.log" \; \
    split-window -p 33 "script -f -c \"eltclsh -robot; ${SCRIPT_DIR}/end.sh\" $logdir/eltclsh-fiacre.log" \; \
    selectp -t 3
