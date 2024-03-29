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
from genomix.event import GenoMError
import logging

logger = logging.getLogger("[CamViz]")


class CamViz:
    def __init__(self, component, params):
        """Connect to Gazebo through Camgazebo pocolibs and load all pocolib modules"""
        self.component = component
        self.ack = True
        self.params = params[str(self)]

    def __call__(self):
        """CamViz component"""

        try:
            self.component.set_pix_size(self.params["pixel_size"])
            self.component.add_camera(self.params["camera"], ack=self.ack)
            for local, remote in self.params["ports"]:
                print(f"Connecting to {local} on {remote}")
                self.component.connect_port({"local": local, "remote": remote})
            # self.component.add_pixel_display(
            #     self.params["pixel_display"], self.params["camera"]
            # )
            # self.component.show(ratio=2, ack=self.ack)
        except GenoMError as e:
            if "already" in str(e):
                logger.warning("CamViz already defined")
        except Exception as e:
            logger.error(f"Failed to connect to CamViz. Throws {e}")
            raise e
        finally:
            logger.info("Connected to CamViz")

        return self

    def stop(self):
        """Stop the CamViz component"""
        self.component.stop()

    def __str__(self):
        return "camviz"
