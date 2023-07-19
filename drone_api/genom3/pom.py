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

logger = logging.getLogger("[POM]")


class POM:
    def __init__(self, component, params) -> None:
        """
        Connect to POM and load all pocolib modules
        """
        self.component = component
        self.params = params[str(self)]
        self.ack = True

    def __call__(self):
        """Connect to component port"""

        try:
            logger.info("Connecting to POM")
            for port in self.params["ports"]:
                logger.info(f"Connecting to port {port[0]}:{port[1]}")
                self._connect_port(port[0], port[1])

            for measurement in self.params["add_measurements"].keys():
                self._add_measurement(measurement)

            self._set_mag_field(
                x=self.params["set_mag_field"][0],
                y=self.params["set_mag_field"][1],
                z=self.params["set_mag_field"][2],
            )
            self.component.set_history_length(
                {"history_length": self.params["history_length"]}
            )
        except Exception as e:
            logger.error(f"Failed to connect to POM. Throws {e}")
            raise e
        finally:
            logger.info("Connected to POM")

        return self

    def start(self):
        """Start the POM component"""
        return

    def stop(self):
        """Stop the POM component"""
        return

    def kill(self):
        """Kill the POM component"""
        logger.info("Killing POM")
        self.component.kill()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "pom"

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})

    def _add_measurement(self, port, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        return self.component.add_measurement(
            {
                "port": port,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }
        )

    def _set_mag_field(self, x=0.0, y=0.0, z=0.0):
        return self.component.set_mag_field({"magdir": {"x": x, "y": y, "z": z}})
