"""Land action."""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)

class Land:
    """Land action for the drone"""

    _is_robot = False

    def __init__(self, components):
        self.rotorcraft, self.maneuver = (
            components["rotorcraft"].component,
            components["maneuver"].component,
        )
        self.ack = True

    def __call__(self):
        logger.info("Landing")

        self.rotorcraft.set_velocity(
            {"desired": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
        )

        result = self.maneuver.take_off(
            {"height": 0.15 if self._is_robot else 0.05, "duration": 0}, ack=self.ack
        )
        result = self.maneuver.wait()

        return result