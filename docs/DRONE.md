# Information about the drone setup

## CPU Information

```bash
Vulnerability Itlb multihit:     KVM: Mitigation: VMX disabled
Architecture:                    x86_64
CPU op-mode(s):                  32-bit, 64-bit
Byte Order:                      Little Endian
Address sizes:                   39 bits physical, 48 bits virtual
CPU(s):                          4
On-line CPU(s) list:             0-3
Thread(s) per core:              2
Core(s) per socket:              2
Socket(s):                       1
NUMA node(s):                    1
Vendor ID:                       GenuineIntel
CPU family:                      6
Model:                           142
Model name:                      Intel(R) Core(TM) i7-7567U CPU @ 3.50GHz
Stepping:                        9
CPU MHz:                         900.040
CPU max MHz:                     4000.0000
CPU min MHz:                     400.0000
BogoMIPS:                        6999.82
Virtualization:                  VT-x
L1d cache:                       64 KiB
L1i cache:                       64 KiB
L2 cache:                        512 KiB
L3 cache:                        4 MiB
NUMA node0 CPU(s):               0-3
```

## Installation

To setup the experiment, we should already have ROS installed. If not, please follow the installation instructions for [ROS](http://wiki.ros.org/noetic/Installation/Ubuntu). Once ROS is installed and configured, we can install the experiment.

To setup the experiment workspace, we can use the following commands:

```bash
# Setup the workspace
mkdir -p drone-experiment && cd drone-experiment
git clone https://github.com/lerema/genom3-experiment.git
```

The easier way to install genom3 and robotpkg modules is to launch the following commands:

```bash
# Move to the root of the repository
cd genom3-experiment

# Source the ROS environment to get the ROS environment variables
source /opt/ros/noetic/setup.bash

# Setup the experiment
bash install-scripts/setup-experiment-robot.sh --all
```

> **Note: The setup script asks for `su` permission to install robotpkg and ros packages using system package manager**

The `setup-experiment.sh` script has the following options:
```bash
Usage: bash install-scripts/setup-experiment-robot.sh [--help] [--clean] [--genom] [--robotpkg] [--all] [--update] [--ros-ws]
  --help: show this help
  --clean: clean the experiment
  --genom: install/reinstall the genom3 modules
  --robotpkg: install/reinstall the robotpkg modules
  --all: install/reinstall all the modules
  --update: update the experiment
  --ros-ws: setup the ros workspace
```

> **Note: `setup-experiment.sh --clean` will remove the installed modules related to the experiment. But the environment variables in `~/.bashrc` will not be removed. So, we need to remove the environment variables manually.**

### Python
The experiment depends on Python3 by default. To install the python api, we can use the following command:

```bash
# In the root of the repository
# Upgrade pip to the latest version. One of the dependency uses `pyproject.toml` which is supported by pip >= 22.3
python3 -m pip install --upgrade pip
# Install dependencies for the experiment
python3 -m pip install -r requirements.txt
# Install the drone python api
python3 -m pip install .
```

> **Note: Once the setup is finished, we can relaunch the terminal or run `source ~/.bashrc` to update the environment variables.**


## Usage

To start the experiment, you need to launch the files from `run/` directory. For example, the following command starts the experiment with a single drone:

```bash
# To start the world with single drone equipped with camera and AR marker based environment
drone_experiment && bash run/start-simu-ar.sh --python # --python/--tcl. --python is optional
```

The above command will load the gazebo world, rviz and genom3 components. The terminal will be launching each component along with a empty terminal for the user. Please resize the terminal to your comfort. The user can use the empty terminal to run the python api or the UP demo.

To start the sample experiment with the python api, run the following command:

```bash
# Test flight with the python api
# From the root of the repository
python3 src/test_runs/move.py
```

To start the UP demo, you could run the following command:

```bash
# UP Demo script
# From the root of the repository
python3 src/up_demo.py
```
> **Note: To restart the experiment, it is recommended to close all the terminals and relaunch the experiment.**
