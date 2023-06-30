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

logger = logging.getLogger("[D435]")


class D435Camera:
    def __init__(self, component, params):
        """Connect to Intel Realsense  Camera through D435 pocolibs and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """D435 Genom component"""

        try:
            self.component.set_fps(self.params["fps"])
            self.component.set_size(
                self.params["x_resolution"], self.params["y_resolution"]
            )
            self.component.connect(self.params["serial_number"])
        except Exception as e:
            logger.error(f"Failed to connect to D435. Throws {e}")
            raise e
        finally:
            logger.info("Connected to D435")

        return self

    def stop(self):
        """Stop the D435 component"""
        self.component.stop()

    def __str__(self):
        return "d435"
