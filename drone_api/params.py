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
    def __call__(self, drone_id=0, is_robot=False, is_outdoor=False):
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

        GPS = {
            "port": ("/dev/ttyACM0", 115200),
            "rtk_port": ("gps-base", 8083),
            "reference": {
                "latitude": 43.561685463999993,
                "longitude": 1.4769517219999999,
                "height": 194.02610000000001,
            },
        }

        MANEUVER = {
            "ports": ("state", f"pom{drone_id}/frame/robot"),
        }
        if is_robot:
            MANEUVER["set_velocity_limit"] = (0.2, 1)
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
            ROTORCRAFT["connect"] = ("chimera-5", 500000)
            ROTORCRAFT["ports"] = ("rotor_input", f"nhfc{drone_id}/rotor_input")
            ROTORCRAFT["set_sensor_rate"] = (1000, 100, 16, 1)
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
            NHFC["geometry"] = {"rz": -1, "mass": 1.845, "cf": 7.8e-4, "ct": 1e-5}
            NHFC["servo_gain"] = (20, 20, 3, 0.3, 15, 15, 0.3, 0.03, 0.5, 3)
            NHFC["wlimit"] = (15, 110)
            NHFC["saturation"] = {"x": 0.2, "v": 0.1, "ix": 0}
        else:
            NHFC["saturation"] = {"x": 1, "v": 1, "ix": 0}
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

        if is_robot:
            COLOR_TRACKER = {
                "rgb": (100, 30, 30),  # red in d435
                "threshold": 40,
                "distance_tolerance": 1.0,
                "object_size": {"width": 0.195, "height": 0.3},
                "focal_length": 1092.0,  # In pixels
                "map_size": {"width": 10.0, "height": 10.0},
                "camera_pose": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": -0.14,  # m
                    "roll": 0.0,
                    "pitch": 0.0,  # 180 deg
                    "yaw": 0.0,  # 45 deg
                },
            }
            COLOR_TRACKER["ports"] = [
                ("DronePose", f"pom{drone_id}/frame/robot"),
                ("Frame", "d435/frame/raw"),
                ("Intrinsics", "d435/intrinsics"),
                ("Extrinsics", "d435/extrinsics"),
            ]
        else:
            COLOR_TRACKER = {
                "rgb": (1, 1, 255),  # blue (1, 1, 140)
                "threshold": 40,
                "distance_tolerance": 1.0,
                "object_size": {"width": 0.5, "height": 0.5},
                "focal_length": 480.0,  # In pixels
                "map_size": {"width": 10.0, "height": 10.0},
                "camera_pose": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": -0.14,  # m
                    "roll": 0.0,
                    "pitch": 0,  # 180 deg
                    "yaw": 0.0,
                },
            }
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
            ports = [
                ("measure/imu", f"rotorcraft{drone_id}/imu"),
                ("measure/mag", f"rotorcraft{drone_id}/mag"),
            ]
            if not is_outdoor:
                ports.append(("measure/mocap", "optitrack/bodies/Lerema"))
            else:
                ports.append(("measure/gps", "gps/state"))
            POM["ports"] = ports

            if is_outdoor:
                POM["add_measurements"] = {
                    "gps": (0, 0, 0, 0, 0, 0),
                    "mag": (0, 0, 0, 0, 0, 0),
                    "imu": (0, 0, 0, 0, 0, 0),
                }
            else:
                POM["add_measurements"] = {
                    "imu": (0, 0, 0, 0, 0, 0),
                    "mocap": (0, 0, 0, 0, 0, 0),
                }

            POM["set_mag_field"] = (
                2.4016311504777697e-05,
                -9.5490819586733245e-07,
                -3.8850558715353451e-05,
            )
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
            "length": 0.095,
            "output_frame": 2,  # 0: camera, 1: drone, 2: world
            "markers": [10],  # , 11, 12, 13],
        }

        FOXGLOVE = {}

        if is_robot:
            FOXGLOVE = {
                "ports": [
                    ("frames/d435", "d435/frame/raw"),
                    (
                        f"frames/CT{drone_id}",
                        f"ColorTracker{drone_id}/output/frame",
                    ),
                    (
                        f"frames/CT{drone_id}_mask",
                        f"ColorTracker{drone_id}/output/mask",
                    ),
                    (
                        f"frames/aruco{drone_id}",
                        f"arucotag{drone_id}/output",
                    ),
                    (f"measure/imu", f"rotorcraft{drone_id}/imu"),
                    (f"measure/mag", f"rotorcraft{drone_id}/mag"),
                    (f"states/drone", f"pom{drone_id}/frame/robot"),
                    # ("gps/gps_info", "gps/info"),
                    # ("states/gps_state", "gps/state"),
                ],
                "ports_info": [
                    ("d435", "::FoxgloveStudio::or_sensor_frame"),
                    (f"CT{drone_id}", "::FoxgloveStudio::or_sensor_frame"),
                    (f"CT{drone_id}_mask", "::FoxgloveStudio::or_sensor_frame"),
                    (f"aruco{drone_id}", "::FoxgloveStudio::or_sensor_frame"),
                    ("drone", "::FoxgloveStudio::or_pose_estimator_state"),
                    ("imu", "::FoxgloveStudio::or_sensor_imu"),
                    ("mag", "::FoxgloveStudio::or_sensor_magnetometer"),
                    # ("gps_info", "::FoxgloveStudio::or_sensor_gps"),
                    # ("gps_state", "::FoxgloveStudio::or_pose_estimator_state"),
                ],
            }
        else:
            FOXGLOVE = {
                "ports": [
                    (f"frames/gazebo{drone_id}", f"camgazebo{drone_id}/frame/raw"),
                    (
                        f"frames/CT{drone_id}",
                        f"ColorTracker{drone_id}/output/frame",
                    ),
                    (
                        f"frames/CT{drone_id}_mask",
                        f"ColorTracker{drone_id}/output/mask",
                    ),
                    (
                        f"frames/aruco{drone_id}",
                        f"arucotag{drone_id}/output",
                    ),
                    (f"measure/imu", f"rotorcraft{drone_id}/imu"),
                    (f"measure/mag", f"rotorcraft{drone_id}/mag"),
                    (f"states/drone", f"pom{drone_id}/frame/robot"),
                ],
                "ports_info": [
                    (f"gazebo{drone_id}", "::FoxgloveStudio::or_sensor_frame"),
                    (f"CT{drone_id}", "::FoxgloveStudio::or_sensor_frame"),
                    (f"CT{drone_id}_mask", "::FoxgloveStudio::or_sensor_frame"),
                    (f"aruco{drone_id}", "::FoxgloveStudio::or_sensor_frame"),
                    ("drone", "::FoxgloveStudio::or_pose_estimator_state"),
                    ("imu", "::FoxgloveStudio::or_sensor_imu"),
                    ("mag", "::FoxgloveStudio::or_sensor_magnetometer"),
                ],
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
            "FoxgloveStudio": FOXGLOVE,
        }

        if is_robot:
            components.pop("camgazebo")
            components.pop("camviz")
            components["d435"] = D435

        if is_outdoor:
            components["gps"] = GPS
            components.pop("optitrack")

        return components


DRONES = [DroneCommon()(drone_id=i, is_robot=False) for i in range(0, 3)]
