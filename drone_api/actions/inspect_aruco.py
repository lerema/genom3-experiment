"""Inspect Aruco Markers action.

Localize coloured objects in the scene.
"""
import logging

from drone_api.core.data import JSONSerializer

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class DetectArucotag:
    """Inspect Aruco Marker Action"""

    def __init__(self, components, robot_id=0):
        self._components = ["arucotag"]
        self.arucotag = components["arucotag"].component
        self.ack = True
        self._status = None

        self._data = JSONSerializer()
        self._aruco_id = 0

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        logger.info("Inspecting Aruco Markers")
        result = self.arucotag.pose("10")

        try:
            self._update_data(result)
        except TypeError:
            return False

        return True

    def _update_data(self, result):
        x = result["pose"]["pos"]["x"]
        y = result["pose"]["pos"]["y"]
        z = result["pose"]["pos"]["z"]

        self._data.update("ENV.NO_ARUCOS", self._aruco_id + 1)
        self._data.update(f"ENV.ARUCOS.{self._aruco_id}.POSE", [x, y, z])
        self._data.update(f"ENV.PLATES.{self._aruco_id}.INSPECTED", True)

        self._aruco_id += 1
