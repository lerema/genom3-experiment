#!/usr/bin/env bash

# Order preserved.
ROBOTPKG_MODULES="architecture/genom3 architecture/genom3-pocolibs shell/eltclsh net/genomix supervision/tcl-genomix interfaces/openrobots2-idl simulation/mrsim-gazebo simulation/optitrack-gazebo path/libkdtp"
GENOM_MODULES="maneuver-genom3 nhfc-genom3 pom-genom3 rotorcraft-genom3 optitrack-genom3 felix-idl felix-g3utils ct_drone minnie-tf2 python-genomix hippo-genom3"
# Get python version similar to 3.8.
PYTHON_VERSION=$(python -c 'import sys; print(sys.version_info[0])').$(python -c 'import sys; print(sys.version_info[1])')

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$(dirname "$SCRIPT_DIR")"/..
INSTALL_DIR=$(realpath "$INSTALL_DIR")

function show_help {
    echo "Usage: setup-experiment.sh [--help] [--clean] [--genom] [--robotpkg] [--all] [--update]"
    echo "  --help: show this help"
    echo "  --clean: clean the experiment"
    echo "  --genom: install/reinstall the genom3 modules"
    echo "  --robotpkg: install/reinstall the robotpkg modules"
    echo "  --all: install/reinstall all the modules"
    echo "  --update: update the experiment"
    exit 0
}

if [ $# -eq 0 ]; then
    show_help
    exit 0
fi

if [ "$1" == "--help" ]; then
    show_help
fi

if [ "$1" == "--clean" ]; then
    echo "Cleaning the experiment"
    # TODO: complete the cleaning
    exit 0
fi

if [ "$1" == "--genom" ]; then
    INSTALL_GENOM=true
else
    INSTALL_GENOM=false
fi

if [ "$1" == "--robotpkg" ]; then
    INSTALL_ROBOTPKG=true
else
    INSTALL_ROBOTPKG=false
fi

if [ "$1" == "--all" ]; then
    INSTALL_GENOM=true
    INSTALL_ROBOTPKG=true
fi

if [ "$1" == "--update" ]; then
    echo "Updating the experiment"
fi

readonly SCRIPT_DIR INSTALL_DIR ROBOTPKG_MODULES GENOM_MODULES
build_start="$(date)"

# Check if ros, gazebo and rviz are installed
if ! [ -x "$(command -v roscore)" ]; then
    echo 'Error: ros 1 is not installed.' >&2
    exit 1
fi
if ! [ -x "$(command -v gazebo)" ]; then
    echo 'Error: gazebo is not installed.' >&2
    exit 1
fi
if ! [ -x "$(command -v rviz)" ]; then
    echo 'Error: rviz is not installed.' >&2
    exit 1
fi

# Install dependencies
sudo apt-get install -y bison python3-vcstool libudev-dev tmux \
    ros-"$ROS_DISTRO"-jsk-rviz-plugins asciidoctor

# Finally build ros simulation package
cd "${SCRIPT_DIR}"/../catkin_ws
catkin_make

# Pull dependencies
cd "$INSTALL_DIR"
vcs import -w 1 <"$SCRIPT_DIR"/drone-genom3.repos

# Install robot-pkg
if [ "$INSTALL_ROBOTPKG" = true ]; then
    printf "Installing robot-pkg...\n"
    # Remove existing installation of robot-pkg
    rm -rf "$INSTALL_DIR"/openrobots/etc/robotpkg.conf
    cd "$INSTALL_DIR"/robotpkg/bootstrap
    ./bootstrap --prefix="$INSTALL_DIR"/openrobots
fi

# EXPORT PACKAGE PATH
# TODO: add sed comparison to check if the path is already exported
echo "export DRONE_VV_PATH=""$INSTALL_DIR""" >>~/.bashrc
echo "export PATH=${INSTALL_DIR}/bin:${INSTALL_DIR}/sbin:${INSTALL_DIR}/openrobots/sbin:${INSTALL_DIR}/openrobots/bin:${PATH}
export PKG_CONFIG_PATH=${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/lib/pkgconfig/genom/pocolibs:${INSTALL_DIR}/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}

export PYTHONPATH=${INSTALL_DIR}/lib/python$PYTHON_VERSION/site-packages:${INSTALL_DIR}/openrobots/lib/python$PYTHON_VERSION/site-packages:${PYTHONPATH}

export GAZEBO_PLUGIN_PATH=${INSTALL_DIR}/openrobots/lib/gazebo:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${INSTALL_DIR}/openrobots/share/gazebo/models:$(realpath "$SCRIPT_DIR"/../models):${GAZEBO_MODEL_PATH}

export GENOM_TMPL_PATH=${INSTALL_DIR}/share/genom/site-templates:${INSTALL_DIR}/openrobots/share/genom/site-templates
" >>~/.bashrc

# shellcheck source=/dev/null
source ~/.bashrc

# Install robopkg
function install_robotpkg_modules {
    printf "Installing robot-pkg modules...\n"
    cd "$INSTALL_DIR"/robotpkg/"$1"
    make update confirm
}

# Create function to build and install GeNoM modules
function install_genom_modules {
    printf "Installing GeNoM module - %s...\n", "$1"
    cd "$INSTALL_DIR"/"$1"
    autoreconf -vif
    mkdir -p build
    cd build
    ../configure --prefix="$INSTALL_DIR" --with-templates=pocolibs/server,pocolibs/client/c
    make install
}

# TODO: automatically detect the base directory of the robot-pkg repository

if [ "$INSTALL_ROBOTPKG" = true ]; then
    for module in $ROBOTPKG_MODULES; do
        install_robotpkg_modules "$module"
    done
fi

# Install genom modules
if [ "$INSTALL_GENOM" = true ]; then
    for module in $GENOM_MODULES; do
        install_genom_modules "$module"
    done
fi

# Final setup

#shellcheck source=/dev/null
source ~/.bashrc
cd "$SCRIPT_DIR"/..

echo "Setup started - $build_start"
echo "Setup finished - $(date)"
