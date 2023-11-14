#!/usr/bin/env bash

set -e

# Get ubuntu codename
UBUNTU_CODENAME=$(lsb_release -cs)

# Get python version similar to 3.8.
PYTHON_VERSION=$(python -c 'import sys; print(sys.version_info[0])').$(python -c 'import sys; print(sys.version_info[1])')

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="$(dirname "$SCRIPT_DIR")"/..
INSTALL_DIR=$(realpath "$INSTALL_DIR")

GENOM_MODULES="felix-idl vision-idl felix-g3utils rotorcraft-genom3 arucotag-genom3 camviz-genom3 camgazebo-genom3 ct_drone color-tracker-genom3 minnie-tf2 foxglove-genom3"
GENOM_ROS_MODULES=""
DEPENDENCIES_FILE="$SCRIPT_DIR"/drone-genom3-ws.repos

readonly SCRIPT_DIR INSTALL_DIR GENOM_MODULES GENOM_ROS_MODULES PYTHON_VERSION

function show_help {
    echo "Usage: setup-experiment.sh [--help] [--clean] [--genom] [--robotpkg] [--all] [--update]"
    echo "  --help: show this help"
    echo "  --clean: clean the experiment"
    echo "  --genom: install/reinstall the genom3 modules"
    echo "  --robotpkg: install/reinstall the robotpkg modules"
    echo "  --all: install/reinstall all the modules"
    echo "  --update: update the experiment"
    echo "  --ros-ws: setup the ros workspace"
    exit 0
}

function install_robotpkg_apt {
    # TODO: Fix ubuntu version
    printf "Setting up robotpkg source list...\n"
    sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub focal robotpkg
EOF

    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |
        sudo apt-key add -

    sudo apt-get update || true
    printf "Installing robot-pkg modules...\n"
    sudo apt-get install -y \
        robotpkg-pom-genom3+codels+openprs+pocolibs-client-c+pocolibs-server \
        robotpkg-nhfc-genom3+codels+openprs+pocolibs-client-c+pocolibs-server \
        robotpkg-maneuver-genom3+codels+openprs+pocolibs-client-c+pocolibs-server \
        robotpkg-optitrack-gazebo \
        robotpkg-optitrack-genom3+codels+openprs+pocolibs-client-c+pocolibs-server \
        robotpkg-genom3-ros \
        robotpkg-genom3-pocolibs \
        robotpkg-genomix \
        robotpkg-tcl-genomix \
        robotpkg-openrobots2-idl \
        robotpkg-mrsim-gazebo \
        robotpkg-libkdtp \
        robotpkg-eltclsh \
        robotpkg-py38-python-genomix
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

if [ $# -eq 0 ]; then
    show_help
fi

if [ "$1" == "--help" ]; then
    show_help
fi

if [ "$1" == "--clean" ]; then
    echo "Cleaning the experiment"
    # Expect the experiment directory, delete all the other directories.
    sudo apt-get remove -y robotpkg-* # Remove robotpkg modules
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
    ROS_WS=true
fi

if [ "$1" == "--update" ]; then
    echo "Updating the experiment"
fi

if [ "$1" == "--ros-ws" ]; then
    ROS_WS=true
else
    ROS_WS=false
fi

build_start="$(date)"

# Check if ros, gazebo and rviz are installed
source /opt/ros/noetic/setup.bash || eval "$(cat /opt/ros/noetic/setup.bash | tail -n +10)"
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
sudo apt-get install -y bison python3-vcstool libudev-dev tmux git \
    python3-rospkg python3-pip \
    ros-"$ROS_DISTRO"-jsk-rviz-plugins asciidoctor \
    ros-"$ROS_DISTRO"-ros-comm ros-"$ROS_DISTRO"-ros ros-"$ROS_DISTRO"-common-msgs ros-"$ROS_DISTRO"-octomap-msgs

# Finally build ros simulation package
if [ "$ROS_WS" = true ]; then
    echo "Building ros simulation package"
    cd "${SCRIPT_DIR}"/../catkin_ws
    catkin_make
fi

# EXPORT PACKAGE PATH
if [ ! -d "$HOME" ]; then
    mkdir "$HOME"
fi

# Create a bashrc file if it does not exist
if [ ! -f "$HOME/.bashrc" ]; then
    touch "$HOME/.bashrc"
fi

if ! grep -qF "function drone_experiment {" "$HOME/.bashrc"; then
    echo "export DRONE_VV_PATH=""$INSTALL_DIR""" >>$HOME/.bashrc
    echo "
    function drone_experiment {
    export PATH=/opt/openrobots/bin:${INSTALL_DIR}/bin:${INSTALL_DIR}/sbin:${INSTALL_DIR}/openrobots/sbin:${INSTALL_DIR}/openrobots/bin:${PATH}
    export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/lib/pkgconfig/genom/pocolibs:${INSTALL_DIR}/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}

    export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:${INSTALL_DIR}/lib/python$PYTHON_VERSION/site-packages:${INSTALL_DIR}/openrobots/lib/python$PYTHON_VERSION/site-packages:${PYTHONPATH}

    export GAZEBO_PLUGIN_PATH=/opt/openrobots/lib/gazebo:${INSTALL_DIR}/openrobots/lib/gazebo:${GAZEBO_PLUGIN_PATH}
    export GAZEBO_MODEL_PATH=/opt/openrobots/share/gazebo/models:${INSTALL_DIR}/openrobots/share/gazebo/models:"$SCRIPT_DIR"/../gazebo/models:${GAZEBO_MODEL_PATH}

    export GENOM_TMPL_PATH=/opt/openrobots/share/genom/site-templates:${INSTALL_DIR}/share/genom/site-templates:${INSTALL_DIR}/openrobots/share/genom/site-templates
    }
    " >>$HOME/.bashrc
fi

# shellcheck source=/dev/null
eval "$(cat $HOME/.bashrc | tail -n +10)" || source $HOME/.bashrc

# TODO: automatically detect the base directory of the robot-pkg repository
# Pull dependencies
cd "$INSTALL_DIR"
vcs import -w 1 <"$DEPENDENCIES_FILE"

if [ "$INSTALL_ROBOTPKG" = true ]; then
    install_robotpkg_apt
fi

# Install genom modules
if [ "$INSTALL_GENOM" = true ]; then
    drone_experiment # Function to source the path from bashrc
    for module in $GENOM_MODULES; do
        install_genom_modules "$module"
    done
fi

# Install genom ros modules
if [ "$INSTALL_GENOM" = true ]; then
    drone_experiment # Function to source the path from bashrc

    for module in $GENOM_ROS_MODULES; do
        install_genom_modules "$module" "ros-template"
    done
fi

# Final setup

#shellcheck source=/dev/null
eval "$(cat $HOME/.bashrc | tail -n +10)" || source $HOME/.bashrc
cd "$SCRIPT_DIR"/..

echo "Setup started - $build_start"
echo "Setup finished - $(date)"
