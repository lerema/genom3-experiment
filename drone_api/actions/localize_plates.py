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
        self._status = None

    def __call__(self, **kwargs):
        logger.info("Localizing plates")
        result = self.ct_drone.ReadROSImageFindTarget(
            z=1.0,
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        result = self.ct_drone.ReadROSImageUpdateFindings(
            ack=self.ack if "ack" not in kwargs else kwargs["ack"],
            callback=self.callback if "callback" not in kwargs else kwargs["callback"],
        )

        return result

    def callback(self, request):
        self._status = request.status