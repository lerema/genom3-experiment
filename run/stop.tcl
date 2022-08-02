#!/bin/sh
# -*-Tcl-*- \
exec eltclsh "$0" ${1+"$@"}

set host localhost
set mw pocolibs

package require genomix
set g [genomix::connect $host:8080]

$g rpath  $::env(HOME)/work/lib/genom/$mw/plugins/

$g load rotorcraft

proc stop {} {
    rotorcraft::set_velocity { desired {0 0  1 0  2 0  3 0  4 0  5 0  6 0  7 0} }
    rotorcraft::stop
}

