import genomix

from drone_api.core.data import JSONSerializer

from .user_types import Area, Location, Robot


class Move:
    """UP Action Representation for the Move action"""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs["robot"]
        self._l_from = kwargs["l_from"]
        self._l_to = kwargs["l_to"]
        self._ack = ack

    def __call__(self, robot: Robot, l_from: Location, l_to: Location):
        self._robot = robot
        self._l_from = l_from
        self._l_to = l_to

        if JSONSerializer().get(f"ROBOTS.{self._robot.id}.location_name") != str(
            self._l_from
        ):
            return False  # Robot is not in the right location to start the action

        result = self._robot.actions.move(l_from=l_from, l_to=l_to)

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
                return True
            elif handle.status == genomix.Status.error:
                return False


class Survey:
    """UP Action Representation for the Survey action"""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs["robot"]
        self._l_from = kwargs["l_from"]
        self._area = self._process_area(kwargs["area"])
        self._ack = ack

    def __call__(self, robot: Robot, area: Area, l_from: Location):
        self._robot = robot
        self._area = self._process_area(area)
        self._l_from = l_from

        if JSONSerializer().get(f"ROBOTS.{self._robot.id}.location_name") != str(
            self._l_from
        ):
            return False  # Robot is not in the right location to start the action

        result = self._robot.actions.survey(area=area)

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                return True
            elif handle.status == genomix.Status.error:
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


class GatherInfo:
    """UP Action Representation for the GatherInfo action"""

    def __init__(self, ack=False, **kwargs):
        self._ack = ack

    def __call__(self):
        result = self._robot.actions.gather_info()

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                return True
            elif handle.status == genomix.Status.error:
                return False
