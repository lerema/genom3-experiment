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

logger = logging.getLogger("[Optittrack]")


class Optitrack:
    def __init__(self, component, params) -> None:
        """
        Connect to Optitrack and load all pocolib modules
        """
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """
        Connect to Optitrack and load all pocolib modules
        """
        try:
            logger.info("Connecting to Optitrack")
            self.component.connect(
                {
                    "host": self.params["host"],
                    "host_port": self.params["host_port"],
                    "mcast": self.params["mcast"],
                    "mcast_port": self.params["mcast_port"],
                }
            )
        except Exception as e:
            logger.error(f"Failed to connect to Optitrack. Throws {e}")
            raise e
        finally:
            logger.info("Connected to Optitrack")

        return self

    def start(self):
        """Start the Optitrack component"""
        return

    def stop(self):
        """Stop the Optitrack component"""
        return

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "optitrack"
