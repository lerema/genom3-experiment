import genomix
import logging
import networkx as nx

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
        JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", False)
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                JSONSerializer().update(
                    f"ROBOTS.{self._robot.id}.location_name", str(self._l_to)
                )
                logger.info("Move action completed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return True
            elif handle.status == genomix.Status.error:
                logger.error("Move action failed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
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
                f"Robot is not in the right location to start the move action. Acquired location: {JSONSerializer().get(f'ROBOTS.{self._robot.id}.location_name')} != {str(self._l_from)}"
            )
            return False  # Robot is not in the right location to start the action

        # Start Localizing the color plates
        self._robot.actions.localize_plates()
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
        JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", False)
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                logger.info("Survey action completed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return True
            elif handle.status == genomix.Status.error:
                logger.error("Survey action failed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
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
        JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", False)
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                logger.info("GatherInfo action completed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return True
            elif handle.status == genomix.Status.error:
                logger.error("GatherInfo action failed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return False


class OptimizeDistance:
    """UP Action Representtion for the Optimize Distance Action.

    This action doesn't depend on Drone API but tried to reorder the plate ID based on the distance.
    """

    def __init__(self, **kwargs):
        pass

    def __call__(self, robot: Robot) -> None:
        self._plates_info = self.get_plates_info()
        JSONSerializer().update(f"ROBOTS.{robot.id}.is_available", False)

        [current_x, current_y, _] = self.get_robot_pose(robot.id)

        # Reorder the plates info based on number of blobs detected
        plate_poses = []
        for plate in self._plates_info.values():
            plate_poses.append(plate["POSE"])

        # Sort the plates based on number of blobs detected
        plate_poses.sort(
            key=lambda x: ((x[0] - current_x) ** 2 + (x[1] - current_y) ** 2) ** 0.5
        )

        # Shortest path
        # self.shortest_path((current_x, current_y), plate_poses)

        self.set_plates_info(plate_poses)

        # Update Optimization state
        JSONSerializer().update("ENV.PLATES.ORDER_OPTIMIZED", True)
        JSONSerializer().update(f"ROBOTS.{robot.id}.is_available", True)

        return True

    def get_robot_pose(self, id_: int):
        """Get robot pose."""
        return JSONSerializer().get(f"ROBOTS.{id_}.pose")

    def get_plates_info(self):
        """Get the plate information."""
        data = {}
        for plate_id in range(JSONSerializer().get("ENV.NO_PLATES")):
            data[plate_id] = JSONSerializer().get(f"ENV.PLATES.{plate_id + 1}")
            logger.debug(f"Getting the plate information {plate_id + 1}")

        return data

    def set_plates_info(self, plates_info):
        """Push the plates information."""
        for i, data in enumerate(plates_info):
            JSONSerializer().update(f"ENV.PLATES.{i + 1}", data)

    def shortest_path(self, current_pose, plate_points):
        # Create a weighted graph where nodes are the points and edges are the distances between them
        G = nx.Graph()
        points = [current_pose]
        points.extend(plate_points)
        for i, point1 in enumerate(points):
            for j, point2 in enumerate(points):
                if i < j:
                    distance = (
                        (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2
                    ) ** 0.5
                    G.add_edge(i, j, weight=distance)
        # Find the shortest path from the current pose to all other points
        distances, path = nx.single_source_dijkstra(G, 0)
        # Order the points based on the shortest path
        ordered_points = [points[i] for i in sorted(path.keys(), key=lambda x: path[x])]
        return ordered_points


class InspectPlate:
    """UP Action representation for UP action."""

    def __init__(self, ack=False, **kwargs):
        self._robot = kwargs.get("robot", None)
        self._location = kwargs.get("locaiton", None)

        self._ack = ack

    def __call__(self, robot: Robot, location: Location):
        """Inspect Plates action"""
        self._robot = robot
        self._location = location

        if JSONSerializer().get(f"ROBOTS.{self._robot.id}.location_name") != str(
            self._location
        ):
            logger.error(
                f"Robot is not in the right location to start the move action. Acquired location: {JSONSerializer().get(f'ROBOTS.{self._robot.id}.location_name') != str(self._location)}"
            )
            return False  # Robot is not in the right location to start the action

        closer_location = self._process_location(self._location)
        closer_location["z"] = 0.5
        moved = self._robot.actions.move(
            l_from=self._process_location(self._location),
            l_to=closer_location,
        )

        moved = self.wait_for_completion(moved)

        result = None
        if moved:
            location_id = int(self._location.name.split("_")[-1])
            result = self._robot.actions.detect_arucotag(location_id=location_id)

            if not result:
                return result

            result = self._robot.actions.move(
                l_from=closer_location, l_to=self._process_location(self._location)
            )

        if self._ack:
            return result

        return self.wait_for_completion(result)

    def wait_for_completion(self, handle):
        JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", False)
        while True:
            genomix.update()
            if handle.status == genomix.Status.done:
                JSONSerializer().update(
                    f"ROBOTS.{self._robot.id}.location_name", str(self._location)
                )
                logger.info("Inspect Plate action completed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return True
            elif handle.status == genomix.Status.error:
                logger.error("Inspect Plate action failed")
                JSONSerializer().update(f"ROBOTS.{self._robot.id}.is_available", True)
                return False

    def _process_location(self, location: Location):
        return {
            "x": location.x,
            "y": location.y,
            "z": location.z,
            "yaw": 0.0,
        }
