"""Land action."""
import logging

from genomix import Status

from drone_api.core.data import JSONSerializer

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Land:
    """Land action for the drone"""

    _is_robot = False

    def __init__(self, components, robot_id=0):
        self._components = ["rotorcraft", "maneuver"]
        self.rotorcraft, self.maneuver = (
            components["rotorcraft"].genomix,
            components["maneuver"].genomix,
        )
        self.ack = True
        self._status = None
        self._id = robot_id

        self._data = JSONSerializer()

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        logger.info("Landing")

        self.rotorcraft.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        result = self.maneuver.take_off(
            {"height": 0.15 if self._is_robot else 0.05, "duration": 0},
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        return result

    def callback(self, request):
        self._status = request.status

        if self._status == Status.done:
            # TODO: Add data to the database
            pass
