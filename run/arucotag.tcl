#!/bin/sh
# -*-Tcl-*- \
exec tclsh "$0" ${1+"$@"}

# # GENOMIX & RPATH
# set devel_path $::env(GENOM_DEVEL)
# set host localhost

# package require genomix
# set g [genomix::connect $host:8080]
# $g rpath ${devel_path}/lib/genom/pocolibs/plugins/

$g load camgazebo
$g load camviz
$g load arucotag

::camgazebo::set_hfov 2
::camgazebo::set_format 640 480
::camgazebo::connect ~/quad/down_camera_link/down_camera/image
::camgazebo::set_extrinsics { ext_values {0 0 1 0 2 1 3 0 4 0 5 0 } }

::arucotag::connect_port frame camgazebo/frame/raw
::arucotag::connect_port intrinsics camgazebo/intrinsics
::arucotag::connect_port extrinsics camgazebo/extrinsics
::arucotag::set_length 0.08
::arucotag::output_frame 2
::arucotag::add_marker 10

::camviz::set_pix_size 5
::camviz::connect_port frame/camgazebo camgazebo/frame/raw
::camviz::add_camera camgazebo &
::camviz::connect_port pixel/tag1 arucotag/pixel_pose/1
::camviz::add_pixel_display tag1 camgazebo
::camviz::show 2
