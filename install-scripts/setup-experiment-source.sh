#!/usr/bin/env bash

# Order preserved.
ROBOTPKG_MODULES="architecture/genom3 architecture/genom3-pocolibs architecture/genom3-ros shell/eltclsh net/genomix supervision/tcl-genomix interfaces/openrobots2-idl simulation/mrsim-gazebo simulation/optitrack-gazebo path/libkdtp supervision/py-python-genomix"
GENOM_MODULES="libkdtp maneuver-genom3 nhfc-genom3 pom-genom3 rotorcraft-genom3 optitrack-genom3 felix-idl felix-g3utils vision-idl d435-genom3 color-tracker-genom3" # arucotag-genom3 camgazebo-genom3 camviz-genom3 color-tracker-genom3"

# Get python version similar to 3.8.
PYTHON_VERSION=$(python3 -c 'import sys; print(sys.version_info[0])').$(python3 -c 'import sys; print(sys.version_info[1])')

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
    # Expect the experiment directory, delete all the other directories.
    DIR=$(ls "$INSTALL_DIR")
    for d in $DIR; do
        if [ "$d" != "genom3-experiment" ]; then # TODO: Name of the experiment directory is expected to be the same as repo.
            echo "Deleting $d"
            rm -rf "$INSTALL_DIR"/"$d"
        fi
    done
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

readonly SCRIPT_DIR INSTALL_DIR ROBOTPKG_MODULES GENOM_MODULES GENOM_ROS_MODULES PYTHON_VERSION
build_start="$(date)"

function setup_installation {
    echo "==================================================="
    echo "This script will not download robotpkg automatically. Please setup robotpkg manually."
    echo "This script will not install ros automatically. Please setup ros manually."
    echo "This script will not install gazebo automatically. Please setup gazebo manually."
    echo "This script will not install rviz automatically. Please setup rviz manually."
    echo "==================================================="
    read -r -p "Enter the robotpkg directory: " ROBOTPKG_DIR
    read -r -p "Enter the custom install path for robotpkg: " ROBOTPKG_INSTALL_DIR
    echo "==================================================="

    # Install dependencies
    sudo apt-get install -y bison python3-vcstool libudev-dev tmux \
    asciidoctor \
    libsdl2-dev

    # Pull dependencies
    cd "$INSTALL_DIR"
    vcs import -w 1 <"$SCRIPT_DIR"/drone-genom3-ws.repos

    # Install robot-pkg
    if [ "$INSTALL_ROBOTPKG" = true ]; then
        printf "Installing robot-pkg...\n"
        # Remove existing installation of robot-pkg
        rm -rf "$ROBOTPKG_INSTALL_DIR"/openrobots/etc/robotpkg.conf
        cd "$ROBOTPKG_DIR"/bootstrap
        ./bootstrap --prefix="$ROBOTPKG_INSTALL_DIR"/openrobots
    fi
    # shellcheck source=/dev/null
    eval "$(cat ~/.bashrc | tail -n +10)" # Hack to reload the bashrc
}

# Install robopkg
function install_robotpkg_modules {
    printf "Installing robot-pkg modules...\n"
    cd "$ROBOTPKG_DIR"/"$1"
    make update confirm
}

# Create function to build and install GeNoM modules
function install_genom_modules {
    printf "Installing GeNoM module - %s...\n", "$1"
    cd "$INSTALL_DIR"/"$1"
    autoreconf -vif
    mkdir -p build
    cd build
    if [ "$2" == "ros-template" ]; then
        ../configure --prefix="$INSTALL_DIR" --with-templates=ros/server,ros/client/c,ros/client/ros,pocolibs/server,pocolibs/client/c --disable-dependency-tracking
    else
        ../configure --prefix="$INSTALL_DIR" --with-templates=pocolibs/server,pocolibs/client/c
    fi
    make install
}

# Setup installation
setup_installation

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

# Install genom ros modules
if [ "$INSTALL_GENOM" = true ]; then
    for module in $GENOM_ROS_MODULES; do
        install_genom_modules "$module" "ros-template"
    done
fi

# Final setup

#shellcheck source=/dev/null
eval "$(cat ~/.bashrc | tail -n +10)" # Hack to reload the bashrc
cd "$SCRIPT_DIR"/..

echo "Setup started - $build_start"
echo "Setup finished - $(date)"

echo "YOUR INSTALLATION IS COMPLETE"
echo "==================================================="
echo "You may need to setup the following environment variables"
echo "==================================================="
# EXPORT PACKAGE PATH
echo "export DRONE_VV_PATH=""$INSTALL_DIR""" >>~/.bashrc
echo "export PATH=${INSTALL_DIR}/bin:${INSTALL_DIR}/sbin:${ROBOTPKG_INSTALL_DIR}/openrobots/sbin:${ROBOTPKG_INSTALL_DIR}/openrobots/bin:${PATH}
export PKG_CONFIG_PATH=${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/lib/pkgconfig/genom/pocolibs:${ROBOTPKG_INSTALL_DIR}/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}

export PYTHONPATH=${INSTALL_DIR}/lib/python$PYTHON_VERSION/site-packages:${ROBOTPKG_INSTALL_DIR}/openrobots/lib/python$PYTHON_VERSION/site-packages:${PYTHONPATH}

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}":${GAZEBO_PLUGIN_PATH}:${ROBOTPKG_INSTALL_DIR}/openrobots/lib/gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(realpath "$SCRIPT_DIR"/../catkin_ws/src/quad-cam_gazebo/models):${GAZEBO_MODEL_PATH}

export GENOM_TMPL_PATH=${INSTALL_DIR}/share/genom/site-templates:${INSTALL_DIR}/openrobots/share/genom/site-templates:${ROBOTPKG_INSTALL_DIR}/openrobots/share/genom/site-templates:${GENOM_TMPL_PATH}
"
echo "==================================================="
