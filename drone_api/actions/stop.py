"""Stop action.

Kills the controller instantly.
"""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


class Stop:
    """Stop action for the drone"""

    logger.info("Stopping the controller")

    def __init__(self, components, **kwargs):
        self.rotorcraft = components["rotorcraft"].component
        self.ack = True

    def __call__(self):
        self.rotorcraft.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, ack=self.ack
        )
        result = self.rotorcraft.stop()

        return result
