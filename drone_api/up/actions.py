import genomix
import logging

from drone_api.core.data import JSONSerializer

from .user_types import Area, Location, Robot

logger = logging.getLogger("[UP]")
logger.setLevel(logging.INFO)


class Move:
    """UP Action Representation for the Move action"""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs.get("robot", None)
        self._l_from = kwargs.get("l_from", None)
        self._l_to = kwargs.get("l_to", None)
        self._ack = ack

    def __call__(self, robot: Robot, l_from: Location, l_to: Location):
        self._robot = robot
        self._l_from = l_from
        self._l_to = l_to

        if JSONSerializer().get(f"ROBOTS.{self._robot.id}.location_name") != str(
            self._l_from
        ):
            logger.error(
                f"Robot is not in the right location to start the move action. Acquired location: {JSONSerializer().get(f'ROBOTS.{self._robot.id}.location_name') != str(self._l_from)}"
            )
            return False  # Robot is not in the right location to start the action

        result = self._robot.actions.move(
            l_from=self._process_location(l_from), l_to=self._process_location(l_to)
        )

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                JSONSerializer().update(
                    f"ROBOTS.{self._robot.id}.location_name", str(self._l_to)
                )
                logger.info(f"Move action completed")
                return True
            elif handle.status == genomix.Status.error:
                logger.error(f"Move action failed")
                return False

    def _process_location(self, location: Location):
        return {
            "x": location.x,
            "y": location.y,
            "z": location.z,
            "yaw": 0.0,
        }


class Survey:
    """UP Action Representation for the Survey action"""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs.get("robot", None)
        self._l_from = kwargs.get("l_from", None)
        if "area" in kwargs:
            self._area = self._process_area(kwargs["area"])
        else:
            self._area = None
        self._ack = ack

    def __call__(self, robot: Robot, area: Area, l_from: Location):
        self._robot = robot
        self._area = self._process_area(area)
        self._l_from = l_from

        if JSONSerializer().get(f"ROBOTS.{self._robot.id}.location_name") != str(
            self._l_from
        ):
            logger.error(
                f"Robot is not in the right location to start the move action. Acquired location: {JSONSerializer().get(f'ROBOTS.{self._robot.id}.location_name') != str(self._l_from)}"
            )
            return False  # Robot is not in the right location to start the action

        result = self._robot.actions.survey(area=self._area)

        surveyed = self.wait_for_completion(result)

        # Move back to the original location
        if surveyed:
            result = self._robot.actions.move(
                l_from=self._process_location(l_from),
                l_to=self._process_location(l_from),
            )

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                logger.info(f"Survey action completed")
                return True
            elif handle.status == genomix.Status.error:
                logger.error(f"Survey action failed")
                return False

    def _process_area(self, area: Area):
        return {
            "xmin": -area.survey_size,
            "xmax": area.survey_size,
            "ymin": -area.survey_size,
            "ymax": area.survey_size,
            "z": area.survey_height,
            "yaw": 0,
        }

    def _process_location(self, location: Location):
        return {
            "x": location.x,
            "y": location.y,
            "z": location.z,
            "yaw": 0.0,
        }


class GatherInfo:
    """UP Action Representation for the GatherInfo action"""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs.get("robot", None)
        self._ack = ack

    def __call__(self, robot: Robot):
        self._robot = robot
        result = self._robot.actions.gather_info()

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                logger.info(f"GatherInfo action completed")
                return True
            elif handle.status == genomix.Status.error:
                logger.error(f"GatherInfo action failed")
                return False
