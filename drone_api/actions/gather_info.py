"""gather info action.

Basically moves to the home station and sends the info to the server.
"""
import logging

from genomix import Status

from drone_api.core.data import JSONSerializer

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class GatherInfo:
    """Sending Info by Move action for the drone"""

    def __init__(self, components, robot_id=0):
        self._components = ["maneuver", "CT_drone"]
        self.maneuver = components["maneuver"].component
        self.ct_drone = components["CT_drone"].component
        self.ack = True

        self._status = None
        self._id = robot_id

        self._data = JSONSerializer()

        self.x = 0.0
        self.y = 0.0
        self.z = 1.0
        self.yaw = 0.0

        self._plates_info = []

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        logger.info(f"Gathering info for robot {self._id}")

        self.ct_drone.MakeBlobMap()
        self.ct_drone.LocalizeBlob()

        result = self.ct_drone.Get_plates()
        self._plates_info = self._prepare_plates_info(result)

        result = self.maneuver.goto(
            {"x": 0.0, "y": 0.0, "z": self.z, "yaw": self.yaw, "duration": 0},
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        # TODO: Creates problem during replanning
        # if self._plates_info:
        #     self.ct_drone.ClearFindings(ack=self.ack)

        return result

    def callback(self, request):
        self._status = request.status

        if self._status == Status.done:
            self._data.update(f"ROBOTS.{self._id}.pose", [self.x, self.y, self.z])

            if self._plates_info:
                for info in self._plates_info:
                    self._data.update(f"ENV.PLATES.{info['id']}.ID", info["id"])
                    self._data.update(f"ENV.PLATES.{info['id']}.POSE", info["pose"])
                    self._data.update(
                        f"ENV.PLATES.{info['id']}.NAME", f"plate_{info['id']}"
                    )
                self._data.update(f"ENV.NO_PLATES", len(self._plates_info))
            else:
                self._data.update(f"ENV.NO_PLATES", 0)
                self._data.update("ENV.PLATES", {})

            # All color plates have ARUCO markers
            self._data.update(f"ENV.NO_ARUCOS", len(self._plates_info))

    def _prepare_plates_info(self, result):
        plates_info = []

        for plate in result["plates"]["seq"]:
            x = plate["coord"]["x"]
            y = plate["coord"]["y"]
            z = plate["coord"]["z"]
            plates_info.append({"pose": [x, y, z], "id": plate["index"]})

        return plates_info
