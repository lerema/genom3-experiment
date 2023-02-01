"""Stop action.

Kills the controller instantly.
"""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Stop:
    """Stop action for the drone"""

    logger.info("Stopping the controller")

    def __init__(self, components):
        self._components = ["rotorcraft"]
        self.rotorcraft = components["rotorcraft"].component
        self.ack = True
        self._status = None

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        self.rotorcraft.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )
        result = self.rotorcraft.stop(
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        return result

    def callback(self, request):
        self._status = request.status
