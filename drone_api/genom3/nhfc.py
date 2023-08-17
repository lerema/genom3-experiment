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

logger = logging.getLogger("[NHFC]")


class NHFC:
    def __init__(self, component, params):
        """Connect to NHFC and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        self.component.set_gtmrp_geom(**self.params["geometry"])
        self._servo_gain(
            self.params["servo_gain"][0],
            self.params["servo_gain"][1],
            self.params["servo_gain"][2],
            self.params["servo_gain"][3],
            self.params["servo_gain"][4],
            self.params["servo_gain"][5],
            self.params["servo_gain"][6],
            self.params["servo_gain"][7],
            self.params["servo_gain"][8],
            self.params["servo_gain"][9],
        )
        for port in self.params["ports"]:
            self.component.connect_port({"local": port[0], "remote": port[1]})

        self.component.set_emerg(
            {
                "emerg": {
                    "descent": self.params["set_emerg"][0],
                    "dx": self.params["set_emerg"][1],
                    "dq": self.params["set_emerg"][2],
                    "dv": self.params["set_emerg"][3],
                    "dw": self.params["set_emerg"][4],
                }
            }
        )
        if self.params["wlimit"]:
            self.component.set_wlimit(
                {"wmin": self.params["wlimit"][0], "wmax": self.params["wlimit"][1]}
            )
        logger.info("Connected to NHFC")

        return self

    def start(self):
        """Start the NHFC component"""
        logger.info("Starting NHFC")
        self.component.set_current_position()
        self.component.servo(ack=self.ack)

    def stop(self):
        """Stop the NHFC component"""
        self.component.stop()

    def kill(self):
        """Kill the NHFC component"""
        logger.info("Killing NHFC")
        self.component.kill()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "nhfc"

    def _servo_gain(self, Kpxy, Kpz, Kqxy, Kqz, Kvxy, Kvz, Kwxy, Kwz, Kixy, Kiz):
        return self.component.set_servo_gain(
            {
                "gain": {
                    "Kpxy": Kpxy,
                    "Kpz": Kpz,
                    "Kqxy": Kqxy,
                    "Kqz": Kqz,
                    "Kvxy": Kvxy,
                    "Kvz": Kvz,
                    "Kwxy": Kwxy,
                    "Kwz": Kwz,
                    "Kixy": Kixy,
                    "Kiz": Kiz,
                }
            }
        )

    def _connect_port(self, local, remote):
        return self.component.connect_port({"local": local, "remote": remote})
