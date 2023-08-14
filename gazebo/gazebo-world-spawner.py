#!/usr/bin/env python3
import os
import subprocess
import time

EXPERIMENT_PATH = os.environ.get("DRONE_VV_PATH") + "/genom3-experiment"


def spawn_gazebo_world(world_path):
    command = f"gz model -w {world_path}"
    process = subprocess.Popen(command, shell=True)

    return process


def spawn_gazebo_model(model_name, model_path, x, y, z):
    # Command to spawn the model using Gazebo's command-line tool
    command = f"gz model-f {model_path} -m {model_name} -x {x} -y {y} -z {z}"

    # Launch Gazebo with the specified model spawn command
    process = subprocess.Popen(command, shell=True)

    # Wait for Gazebo to finish spawning the model
    time.sleep(5)  # Adjust this delay as needed

    # Return the process object so it can be terminated later if needed
    return process


if __name__ == "__main__":
    # Spawn model
    world_name = f"{EXPERIMENT_PATH}/gazebo/worlds/single_drone.world"
    if not os.path.exists(world_name):
        print(f"World {world_name} does not exist")
        exit(1)

    gazebo_process = spawn_gazebo_world(world_name)
    print(f"World {world_name} spawned")

    # Spawn model
    spawn_positions = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (-1.0, 0.0, 0.0),
    ]
    for i, model_name in enumerate(["plate"]):
        model_path = f"{EXPERIMENT_PATH}/gazebo/models/{model_name}/model.sdf"
        if not os.path.exists(model_path):
            print(f"Model {model_path} does not exist")
            exit(1)

        gazebo_process = spawn_gazebo_model(model_name, model_path, 1, 0, 0)
        print(f"Model {model_name} spawned")

        time.sleep(5)
