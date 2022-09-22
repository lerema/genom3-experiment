# DRONE EXPERIMENT

## Installation

The easier way to setup repository is to launch the following command:

```bash
bash install-scripts/setup-experiment.sh --all
```

If we are expecting different operations, we can use the following command:

```bash
bash install-scripts/setup-experiment.sh --help
```

To install the python api, we can use the following command:

```bash
# In the root of the repository
pip install -e .
```

## Usage

To start the experiment, you need to launch the files from `run/` directory. For example, the following command starts the experiment with a single drone:

```bash
bash run/start-simu-cam.sh
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
