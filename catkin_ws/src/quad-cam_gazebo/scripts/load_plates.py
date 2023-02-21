"""Spawn plates and aruco markers in gazebo."""
import rospy
import rospkg
import tf
import random
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

# This package directory
dir = rospkg.RosPack().get_path("quad-cam_gazebo")


class Points:
    def __init__(self, n=10, r=1, center=(0, 0), mindist=0.2, maxtrials=1000):
        self.success = False
        self.n = n
        self.r = r
        self.center = np.array(center)
        self.d = mindist
        self.points = np.ones((self.n, 2)) * 10 * r + self.center
        self.c = 0
        self.trials = 0
        self.maxtrials = maxtrials
        self.tx = "rad: {}, center: {}, min. dist: {} ".format(self.r, center, self.d)
        self.fill()

    def dist(self, p, x):
        if len(p.shape) > 1:
            return np.sqrt(np.sum((p - x) ** 2, axis=1))
        else:
            return np.sqrt(np.sum((p - x) ** 2))

    def newpoint(self):
        x = (np.random.rand(2) - 0.5) * 2
        x = x * self.r - self.center
        if self.dist(self.center, x) < self.r:
            self.trials += 1
            if np.all(self.dist(self.points, x) > self.d):
                self.points[self.c, :] = x
                self.c += 1

    def fill(self):
        while self.trials < self.maxtrials and self.c < self.n:
            self.newpoint()
        self.points = self.points[self.dist(self.points, self.center) < self.r, :]
        if len(self.points) == self.n:
            self.success = True
        self.tx += "\n{} of {} found ({} trials)".format(
            len(self.points), self.n, self.trials
        )

    def __repr__(self):
        return self.tx


def random_poses(no_plates, distance, area_size):
    """Generate random poses for plates."""
    poses = []
    z = 0.05  # height of the plate
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    p = Points(n=no_plates, r=area_size, mindist=distance)
    for x, y in zip(p.points[:, 0], p.points[:, 1]):
        poses.append(Pose(Point(x=x, y=y, z=z), Quaternion(*q)))
    return poses


if __name__ == "__main__":
    rospy.init_node("load_plates")
    # Load plates
    # parameters
    no_plates = rospy.get_param("~no_plates", 5)
    distance = rospy.get_param("~distance", 3.5)
    area_size = rospy.get_param("~area_size", 4.0)

    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_plate = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # Delete plates
    rospy.wait_for_service("gazebo/delete_model")
    delete_plate = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    # Load aruco markers
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_aruco = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    # Delete aruco markers
    rospy.wait_for_service("gazebo/delete_model")
    delete_aruco = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    plate_poses = random_poses(no_plates, distance, area_size)
    plate_model = open(f"{dir}/models/plate/model.sdf", "r").read()
    plates = []
    for i in range(no_plates):
        plates.append("plate" + str(i + 1))

    for plate, pose in zip(plates, plate_poses):
        rospy.loginfo(f"Spawning {plate}")
        spawn_plate(plate, plate_model, "", pose, "world")
