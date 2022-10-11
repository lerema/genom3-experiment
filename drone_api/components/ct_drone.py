# Copyright 2022 Selvakumar H S
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

logger = logging.getLogger("[CTDrone]")


class CTDrone:
    def __init__(self, component, params):
        """Connect to CTDrone and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """CTDrone component"""

        try:
            self.component.Set_my_r(self.params["rgb"][0])
            self.component.Set_my_g(self.params["rgb"][1])
            self.component.Set_my_b(self.params["rgb"][2])
            self.component.Set_my_seuil(self.params["threshold"])
            self.component.connect_port(
                {"local": self.params["ports"][0], "remote": self.params["ports"][1]}
            )
            self.component.SetCameraImageTopicName(self.params["image_topic"])
            self.component.SetCameraInfoTopicName(self.params["image_info_topic"])
        except Exception as e:
            logger.error(f"Failed to connect to CTDrone. Throws {e}")
            raise e
        finally:
            logger.info("Connected to CTDrone")

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
        return "CT_drone"
