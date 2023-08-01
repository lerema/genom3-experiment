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

logger = logging.getLogger("[TF2]")


class TF2:
    def __init__(self, component, params):
        """Connect to TF2 and load all pocolib modules"""
        self.component = component
        self.params = params[str(self)]
        self.ack = True

    def __call__(self):
        """TF2 component"""

        try:
            for port in self.params["ports"]:
                self._connect_port(port[0], port[1])
            self._add_dynamic_tf(**self.params["dynamic_tf"])
            self._add_dynamic_pos_tf(**self.params["dynamic_tf_pos"])
            self._add_odometry(self.params["odometry"])

            self.component.AddTwistFromPose(self.params["twist_from_pose"])
            self.component.AddWrenchFromPose(self.params["wrench_from_pose"])
            for tf in self.params["static_transform"]:
                self._publish_static_tf(**tf)
            self.component.AddOccupancyGrid(self.params["occupancy_grid"])
        except GenoMError as exception:
            if "already_defined" in str(exception):
                logging.warning("TF2 already defined")
        except Exception as exception:
            logging.error(f"Failed to connect to TF2. Throws {exception}")
            raise exception
        finally:
            logging.info("Connected to TF2")

        return self

    def start(self):
        return

    def stop(self):
        self.component.stop()

    def kill(self):
        self.component.stop()

    def __del__(self):
        try:
            self.component.kill()
        except RuntimeError:
            logger.info(f"Unloaded {str(self)}")

    def __str__(self) -> str:
        return "tf2"

    def _connect_port(self, local, remote):
        return self.component.connect_port(local=local, remote=remote)

    def _add_odometry(self, name):
        return self.component.AddOdometry({"name": name})

    def _publish_static_tf(self, **kwargs):
        return self.component.PublishStaticTF(kwargs, ack=self.ack)

    def _add_dynamic_tf(self, **kwargs):
        return self.component.AddDynamicTF(kwargs)

    def _add_dynamic_pos_tf(self, **kwargs):
        return self.component.AddDynamicPosTF(kwargs)
