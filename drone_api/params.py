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

ROBOT_NAME = "Lerema"

# pylint: disable=too-few-public-methods, invalid-name


class DroneCommon:
    def __call__(self, drone_id=0, is_robot=False):
        # To access both experiments
        drone_id = "" if drone_id == 0 else str(drone_id)

        OPTITRACK = {}
        if is_robot:
            OPTITRACK["host"] = "muybridge"
            OPTITRACK["host_port"] = "1510"
            OPTITRACK["mcast"] = "239.192.168.30"
            OPTITRACK["mcast_port"] = "1511"
        else:
            OPTITRACK["host"] = "localhost"
            OPTITRACK["host_port"] = "1509"
            OPTITRACK["mcast"] = ""
            OPTITRACK["mcast_port"] = ""

        MANEUVER = {
            "ports": ("state", f"pom{drone_id}/frame/robot"),
        }
        if is_robot:
            MANEUVER["set_velocity_limit"] = (1, 1)
            MANEUVER["bounds"] = {
                "xmin": -4,
                "xmax": 4,
                "ymin": -4,
                "ymax": 4,
                "zmin": 0.5,
                "zmax": 2.5,
                "yawmin": -10,
                "yawmax": 10,
            }
        else:
            MANEUVER["set_velocity_limit"] = (2, 1)
            MANEUVER["bounds"] = {
                "xmin": -100,
                "xmax": 100,
                "ymin": -100,
                "ymax": 100,
                "zmin": -2,
                "zmax": 30,
                "yawmin": -10,
                "yawmax": 10,
            }

        ROTORCRAFT = {}
        if is_robot:
            ROTORCRAFT["connect"] = ("/dev/ttyUSB0", 500000)
            ROTORCRAFT["ports"] = ("rotor_input", f"nhfc{drone_id}/rotor_input")
            ROTORCRAFT["set_sensor_rate"] = (1000, 0, 16, 1)
        else:
            ROTORCRAFT["connect"] = (f"/tmp/pty-quad{drone_id}", 0)
            ROTORCRAFT["ports"] = ("rotor_input", f"nhfc{drone_id}/rotor_input")
            ROTORCRAFT["set_sensor_rate"] = (1000, 0, 16, 1)
            ROTORCRAFT["imu_filter"] = {
                "gfc": [20, 20, 20],
                "afc": [5, 5, 5],
                "mfc": [20, 20, 20],
            }

        NHFC = {
            "ports": [
                ("state", f"pom{drone_id}/frame/robot"),
                ("reference", f"maneuver{drone_id}/desired"),
            ],
            "set_emerg": (0.1, 0.5, 1, 3, 3),
        }

        if is_robot:
            NHFC["geometry"] = {"rz": -1}
            NHFC["servo_gain"] = (20, 25, 3, 0.3, 15, 20, 0.3, 0.03, 0.5, 3)
            NHFC["wlimit"] = None
        else:
            NHFC["geometry"] = {
                "rotors": 4,
                "cx": 0,
                "cy": 0,
                "cz": 0,
                "armlen": 0.23,
                "mass": 1.28,
                "rx": 0,
                "ry": 0,
                "rz": -1,
                "cf": 6.5e-4,
                "ct": 1e-5,
            }
            NHFC["servo_gain"] = (5, 5, 4, 0.1, 6, 6, 1, 0.1, 0, 0)
            NHFC["wlimit"] = (16, 100)

        TF2 = {
            "ports": [
                (f"Poses/drone{drone_id}", f"pom{drone_id}/frame/robot"),
                (f"Poses/drone{drone_id}_pos", f"pom{drone_id}/frame/robot"),
                (f"OccupancyGrids/og{drone_id}", f"CT_drone{drone_id}/OccupancyGrid"),
            ],
            "dynamic_tf": {
                "frame_name": f"drone{drone_id}",
                "port_name": f"drone{drone_id}",
                "parent_frame": "world",
                "ms_period": 10,
                "undef_in_orig": True,
            },
            "dynamic_tf_pos": {
                "frame_name": f"drone{drone_id}",
                "port_name": f"drone{drone_id}_pos",
                "parent_frame": "world",
                "ms_period": 10,
                "undef_in_orig": True,
            },
            "odometry": f"drone{drone_id}",
            "twist_from_pose": {
                "name": f"drone{drone_id}",
                "frame": f"drone{drone_id}_pos",
                "topic": f"drone{drone_id}_twist",
                "ms_period": 10,
            },
            "wrench_from_pose": {
                "name": f"drone{drone_id}",
                "frame": f"drone{drone_id}_pos",
                "topic": f"drone{drone_id}_wrench",
                "ms_period": 10,
            },
            "static_transform": [
                {
                    "name": f"og{drone_id}",
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
                "name": f"og{drone_id}",
                "frame": f"og{drone_id}",
                "topic": f"og{drone_id}",
                "ms_period": 50,
            },
        }

        CT_DRONE = {
            "rgb": (5, 5, 255),
            "threshold": 40,
            "ports": ("Pose", f"pom{drone_id}/frame/robot"),
            "image_topic": f"/quad{str(drone_id)}/down_camera_link/down_raw_image",
            "image_info_topic": f"/quad{str(drone_id)}/down_camera_link/down_info_image",
        }

        COLOR_TRACKER = {
            "rgb": (1, 1, 140),  # blue
            "threshold": 40,
            "distance_tolerance": 1.0,
        }
        if is_robot:
            COLOR_TRACKER["ports"] = [
                ("DronePose", f"pom{drone_id}/frame/robot"),
                ("Frame", "d435/frame/raw"),
                ("Intrinsics", "d435/intrinsics"),
                ("Extrinsics", "d435/extrinsics"),
            ]
        else:
            COLOR_TRACKER["ports"] = [
                ("DronePose", f"pom{drone_id}/frame/robot"),
                ("Frame", f"camgazebo{drone_id}/frame/raw"),
                ("Intrinsics", f"camgazebo{drone_id}/intrinsics"),
                ("Extrinsics", f"camgazebo{drone_id}/extrinsics"),
            ]

        POM = {
            "history_length": 0.5,
        }
        if is_robot:
            POM["ports"] = [
                ("measure/imu", f"rotorcraft{drone_id}/imu"),
                ("measure/mocap", "optitrack/bodies/Lerema"),
            ]
            POM["add_measurements"] = {
                "imu": (0, 0, 0, 0, 0, 0),
                "mocap": (0, 0, 0, 0, 0, 0),
            }

            POM["set_mag_field"] = None
        else:
            POM["ports"] = [
                ("measure/imu", f"rotorcraft{drone_id}/imu"),
                ("measure/mocap", f"optitrack/bodies/QR{drone_id}"),
                ("measure/mag", f"rotorcraft{drone_id}/mag"),
            ]
            POM["add_measurements"] = {
                "imu": (0, 0, 0, 0, 0, 0),
                "mocap": (0, 0, 0, 0, 0, 0),
                "mag": (0, 0, 0, 0, 0, 0),
            }
            POM["set_mag_field"] = (23.8e-06, -0.4e-06, -39.8e-06)

        ARUCOTAG = {
            "length": 0.2,
            "output_frame": 2,  # 0: camera, 1: drone, 2: world
            "markers": [10],  # , 11, 12, 13],
        }
        if is_robot:
            # Robot params
            ARUCOTAG["ports"] = [
                ("frame", "d435/frame/raw"),
                ("intrinsics", "d435/intrinsics"),
                ("extrinsics", "d435/extrinsics"),
            ]
        else:
            ARUCOTAG["ports"] = [
                ("frame", f"camgazebo{drone_id}/frame/raw"),
                ("intrinsics", f"camgazebo{drone_id}/intrinsics"),
                ("extrinsics", f"camgazebo{drone_id}/extrinsics"),
            ]

        CAM_GAZEBO = {
            "hfov": 2,
            "x_resolution": 640,
            "y_resolution": 480,
            "port": f"~/quad{drone_id}/down_camera_link/down_camera/image",
            "extrinsics": {"ext_values": [0, 0, 1, 0, 2, 1]},  # , 3, 0, 4, 0, 5, 0]},
        }

        D435 = {
            "fps": 30,
            "x_resolution": 640,
            "y_resolution": 480,
            "serial_number": "832112070817",
        }

        CAM_VIZ = {
            "pixel_size": 3,
            "ports": [
                # TODO: Add multiple ports
                ("frame/camgazebo", f"camgazebo{drone_id}/frame/raw"),
                # ("pixel/tag1", "arucotag/pixel_pose/1"),
            ],
            "camera": f"camgazebo{drone_id}",
            "pixel_display": "tag1",
        }

        components = {
            "optitrack": OPTITRACK,
            "pom": POM,
            "maneuver": MANEUVER,
            "rotorcraft": ROTORCRAFT,
            "nhfc": NHFC,
            "CT_drone": CT_DRONE,
            "ColorTracker": COLOR_TRACKER,
            "tf2": TF2,
            "arucotag": ARUCOTAG,
            "camgazebo": CAM_GAZEBO,
            "camviz": CAM_VIZ,
        }

        if is_robot:
            components.pop("camgazebo")
            components.pop("camviz")
            components["d435"] = D435

        return components


DRONES = [DroneCommon()(drone_id=i, is_robot=False) for i in range(0, 3)]
