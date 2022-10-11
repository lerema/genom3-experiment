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

logger = logging.getLogger("[ArucoTag]")


class ArucoTag:
    def __init__(self, component, params):
        """Connect to ArucoTag and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """Aruco Tag component"""

        try:
            for (local, remote) in self.params["ports"]:
                self.component.connect_port({"local": local, "remote": remote})
            self.component.set_length(self.params["length"])
            self.component.output_frame(self.params["output_frame"])
            self.component.add_marker(self.params["markers"])
        except Exception as e:
            logger.error(f"Failed to connect to ArucoTag. Throws {e}")
            raise e
        finally:
            logger.info("Connected to ArucoTag")

    def stop(self):
        """Stop the ArucoTag component"""
        self.component.stop()

    def __str__(self) -> str:
        return "arucotag"
