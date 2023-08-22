"""Localize plates action.

Localize coloured objects in the scene.
"""
import logging

logger = logging.getLogger("[Actions]")
logger.setLevel(logging.INFO)


# This class is not used in the current version. The behavior is integrated in the Survey action.
class CTDroneLocalizePlates:
    """Localize coloured plates"""

    def __init__(self, components, robot_id=0):
        self._components = ["CT_drone"]
        self.ct_drone = components["CT_drone"].genomix
        self.ack = True
        self._status = None

    @property
    def components(self):
        return self._components

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


class LocalizePlates:
    """Localize coloured plates"""

    def __init__(self, components, robot_id=0):
        self._components = ["ColorTracker"]
        self.color_tracker = components["ColorTracker"].genomix
        self.ack = True
        self._status = None

    @property
    def components(self):
        return self._components

    def __call__(self, **kwargs):
        logger.info("Localizing plates")
        # Start the activity
        result = self.color_tracker.color_track(ack=self.ack)

        self.color_tracker.perform_tracking(True)

        return result

    def callback(self, request):
        self._status = request.status

    def __del__(self):
        self.color_tracker.perform_tracking(False)
