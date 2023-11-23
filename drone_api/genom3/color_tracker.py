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

import logging

logger = logging.getLogger("[ColorTracker]")

from drone_api import USE_ROBOT


class ColorTracker:
    """Connect to ColorTracker and load all pocolib modules"""

    def __init__(self, component, params):
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """ColorTracker component"""

        self.component.set_color(
            color={
                "r": self.params["rgb"][0],
                "g": self.params["rgb"][1],
                "b": self.params["rgb"][2],
                "threshold": self.params["threshold"],
            }
        )

        if not USE_ROBOT:
            self.component.show_image_frames(1)

        self.component.set_object_size(**self.params["object_size"])
        self.component.set_focal_length(self.params["focal_length"])
        self.component.set_camera_pose(camera_pose=self.params["camera_pose"])
        self.component.set_distance_threshold(self.params["distance_tolerance"])
        self.component.set_map_size(
            self.params["map_size"]["width"], self.params["map_size"]["height"]
        )
        for local, remote in self.params["ports"]:
            self.component.connect_port(local=local, remote=remote)

        return self

    def start(self):
        """Start the CTDrone component"""
        logger.info("Starting CT_Drone")
        self.component.PublishOccupancyGrid(ack=self.ack)

    def stop(self):
        """Stop the CTDrone component"""
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "ColorTracker"
