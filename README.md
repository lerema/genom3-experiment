# DRONE EXPERIMENT

This is a Genom3 based experiment for drones in search and survey application. The experiment depends on [Genom3](https://git.openrobots.org/projects/genom3) framework, [Robotpkg](http://robotpkg.openrobots.org/) modules and [ROS](https://www.ros.org/). The experiment is using [Unified Planning](https://github.com/aiplan4eu/unified-planning) for planning and [OpenCV](https://opencv.org/) for computer vision.

## Pre-requesties

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Python 3.8
- OpenCV4

The experiment has been tested on Ubuntu 20.04 with ROS Noetic and Gazebo 11. But it should work on other versions of Ubuntu, ROS1 and Gazebo.

## Setup

To setup the experiment, we should already have ROS and Gazebo installed. If not, please follow the installation instructions for [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install). Once ROS and Gazebo are installed and configured, we can install the experiment.

To setup the experiment workspace, we can use the following commands:

```bash
# Setup the workspace
mkdir -p drone-experiment && cd drone-experiment
git clone https://github.com/franklinselva/genom3-experiment.git
```

### Genom3

The easier way to install genom3 and robotpkg modules is to launch the following commands:

```bash
# Setup the experiment
bash install-scripts/setup-experiment.sh --all
```

The `setup-experiment.sh` script has the following options:
```bash
Usage: bash install-scripts/setup-experiment.sh [--help] [--clean] [--genom] [--robotpkg] [--all] [--update]
  --help: show this help
  --clean: clean the experiment
  --genom: install/reinstall the genom3 modules
  --robotpkg: install/reinstall the robotpkg modules
  --all: install/reinstall all the modules
  --update: update the experiment
```

> **Note: `setup-experiment.sh --clean` will remove the installed modules related to the experiment. But the environment variables in `~/.bashrc` will not be removed. So, we need to remove the environment variables manually.**

### Python
The experiment depends on Python3 by default. To install the python api, we can use the following command:

```bash
# In the root of the repository
python3 -m pip install .
```

> **Note: Once the setup is finished, we can relaunch the terminal or run `source ~/.bashrc` to update the environment variables.**


## Usage

To start the experiment, you need to launch the files from `run/` directory. For example, the following command starts the experiment with a single drone:

```bash
# To start the world with single drone equipped with camera
bash run/start-simu-cam.sh

# To start the world with single drone equipped with camera and AR marker based environment
bash run/start-simu-ar.sh
```

To start the sample experiment with the python api, run the following command:

```bash
# Simple example for a single drone
python3 src/sample_app.py
```

To start the UP demo, you could run the following command:

```bash
# UP Demo script
python3 src/up_demo.py
```
