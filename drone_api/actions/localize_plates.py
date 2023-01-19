"""Localize plates action.

Localize coloured objects in the scene.
"""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)

class LocalizePlates:
    """Localize coloured plates"""

    def __init__(self, components):
        self.ct_drone = components["CT_drone"].component
        self.ack = True

    def __call__(self, **kwargs):
        logger.info("Localizing plates")
        self.ct_drone.ReadROSImageFindTarget(z=1.0, ack=self.ack)
        return True

    def monitor(self, **kwargs):
        try:
            pose = self.ct_drone.TargetPose()
            return pose.pos
        except Exception as e:
            return {}
