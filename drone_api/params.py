# Copyright 2022 Selvakumar H S, LAAS-CNRS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

""""Params for drones"""

# TODO: Check params for real robot


class DroneCommon:
    def __call__(self, id=0, is_robot=False):
        # To access both experiments
        id = "" if id == 0 else str(id)

        OPTITRACK = {
            "host": "marey" if is_robot else "localhost",
            "host_port": "1510" if is_robot else "1509",
            "mcast": "239.192.168.30" if is_robot else "",
            "mcast_port": "1511" if is_robot else "",
        }

        MANEUVER = {
            "ports": ("state", f"pom{id}/frame/robot"),
            "set_velocity_limit": (5, 2) if is_robot else (2, 1),
        }

        ROTORCRAFT = {
            "connect": ("/dev/ttyUSB0", 500000)
            if is_robot
            else (f"/tmp/pty-quad{id}", 500000),
            "ports": ("rotor_input", f"nhfc{id}/rotor_input"),
            "set_sensor_rate": (1000, 0, 16, 1) if is_robot else (1000, 50, 16, 1),
        }

        NHFC = {
            "ports": [
                ("state", f"pom{id}/frame/robot"),
                ("reference", f"maneuver{id}/desired"),
            ],
            "set_emerg": (1.2, 0.1, 1, 0.3, 1),
            "servo_gain": (20, 25, 3, 0.3, 15, 20, 0.3, 0.03, 0.5, 3)
            if is_robot
            else (20, 20, 3, 0.3, 8, 8, 0.3, 0.03, 0, 0),
            "wlimit": (0, 0) if is_robot else (16, 100),
        }

        TF2 = {
            "ports": [
                (f"Poses/drone{id}", f"pom{id}/frame/robot"),
                (f"Poses/drone{id}_pos", f"pom{id}/frame/robot"),
                (f"OccupancyGrids/og{id}", f"CT_drone{id}/OccupancyGrid"),
            ],
            "dynamic_tf": {
                "frame_name": f"drone{id}",
                "port_name": f"drone{id}",
                "parent_frame": "world",
                "ms_period": 10,
                "undef_in_orig": True,
            },
            "dynamic_tf_pos": {
                "frame_name": f"drone{id}",
                "port_name": f"drone{id}_pos",
                "parent_frame": "world",
                "ms_period": 10,
                "undef_in_orig": True,
            },
            "odometry": f"drone{id}",
            "twist_from_pose": {
                "name": f"drone{id}",
                "frame": f"drone{id}_pos",
                "topic": f"drone{id}_twist",
                "ms_period": 10,
            },
            "wrench_from_pose": {
                "name": f"drone{id}",
                "frame": f"drone{id}_pos",
                "topic": f"drone{id}_wrench",
                "ms_period": 10,
            },
            "static_transform": [
                {
                    "name": f"og{id}",
                    "parent_frame": "world",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "roll": 0,
                    "pitch": 0,
                    "yaw": 0,
                },
                {  # TODO: Remove this
                    "name": "base",
                    "parent_frame": "drone",
                    "x": 0,
                    "y": 0,
                    "z": 0,
                    "roll": 0,
                    "pitch": 0,
                    "yaw": 0,
                },
            ],
            "occupancy_grid": {
                "name": f"og{id}",
                "frame": f"og{id}",
                "topic": f"og{id}",
                "ms_period": 50,
            },
        }

        CT_DRONE = {
            "rgb": (5, 5, 255),
            "threshold": 40,
            "ports": ("Pose", f"pom{id}/frame/robot"),
            "image_topic": f"/quad{str(id)}/down_camera_link/down_raw_image",
            "image_info_topic": f"/quad{str(id)}/down_camera_link/down_info_image",
        }

        # if id == "":
        #     id = 1
        # else:
        #     id = int(id) + 1

        POM = {
            "ports": [
                ("measure/imu", f"rotorcraft{id}/imu"),
                ("measure/mocap", f"optitrack/bodies/QR_{id}"),
            ]
            if is_robot
            else [
                ("measure/imu", f"rotorcraft{id}/imu"),
                ("measure/mocap", f"optitrack/bodies/QR{id}"),
                ("measure/mag", f"rotorcraft{id}/mag"),
            ],
            "add_measurements": {"imu": (0, 0, 0, 0, 0, 0), "mocap": (0, 0, 0, 0, 0, 0)}
            if is_robot
            else {
                "imu": (0, 0, 0, 0, 0, 0),
                "mocap": (0, 0, 0, 0, 0, 0),
                "mag": (0, 0, 0, 0, 0, 0),
            },
            "set_mag_field": (0, 0, 0)
            if is_robot
            else (23.816e-6, -0.41e-6, -39.829e-6),
            "history_length": 0.5,
        }

        ARUCOTAG = {
            "length": 0.2,
            "ports": [
                ("frame", "camgazebo/frame/raw"),
                ("drone", "pom/frame/robot"),
                ("intrinsics", "camgazebo/intrinsics"),
                ("extrinsics", "camgazebo/extrinsics"),
            ],
            "length": 0.08,
            "output_frame": 2,  # 0: camera, 1: drone, 2: world
            "markers": [10, 11, 12, 13],
        }

        CAM_GAZEBO = {
            "hfov": 2,
            "x_resolution": 640,
            "y_resolution": 480,
            "port": f"~/quad{id}/down_camera_link/down_camera/image",
            "extrinsics": {"ext_values": [0, 0, 1, 0, 2, 1]},  # , 3, 0, 4, 0, 5, 0]},
        }

        CAM_VIZ = {
            "pixel_size": 3,
            "ports": [
                # TODO: Add multiple ports
                ("frame/camgazebo", f"camgazebo/frame/raw"),
                # ("pixel/tag1", "arucotag/pixel_pose/1"),
            ],
            "camera": "camgazebo",
            "pixel_display": "tag1",
        }

        return {
            "optitrack": OPTITRACK,
            "pom": POM,
            "maneuver": MANEUVER,
            "rotorcraft": ROTORCRAFT,
            "nhfc": NHFC,
            "CT_drone": CT_DRONE,
            "tf2": TF2,
            "arucotag": ARUCOTAG,
            "camgazebo": CAM_GAZEBO,
            "camviz": CAM_VIZ,
        }


DRONES = [DroneCommon()(id=i, is_robot=False) for i in range(0, 3)]
