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

logger = logging.getLogger("[Foxglove]")


class FoxgloveStudio:
    def __init__(self, component, params) -> None:
        """Connect foxglove component and setup the component"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """Configure the component"""
        logger.info("Connecting to Foxglove")

        # Connect ports to the component
        for local, remote in self.params["ports"]:
            self.component.connect_port(local=local, remote=remote)

        # Add port info to the component
        for port_name, port_type in self.params["ports_info"]:
            self.component.add_port(port_name, port_type)

        self.component.start_foxglove_server()
        logger.info("Connected to Foxglove")

        return self

    def start(self):
        """Start the Optitrack component"""

    def stop(self):
        """Stop the Optitrack component"""
        return

    def kill(self):
        """Kill the Optitrack component"""
        logger.info("Killing Optitrack")
        self.component.kill()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "FoxgloveStudio"
