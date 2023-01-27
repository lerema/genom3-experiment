"""Spawn plates and aruco markers in gazebo."""
import rospy
import rospkg
import tf
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion

# This package directory
dir = rospkg.RosPack().get_path('quad-cam_gazebo')

def random_poses(no_plates, distance, area_size):
    """Generate random poses for plates."""
    poses = []
    for i in range(no_plates):
        x = (area_size - distance) * (2 * random.random() - 1)
        y = (area_size - distance) * (2 * random.random() - 1)
        z = 0.05 # height of the plate
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        poses.append(Pose(Point(x=x, y=y, z=z), Quaternion(*q)))
    return poses

if __name__ == '__main__':
    rospy.init_node('load_plates')
    # Load plates
    # parameters
    no_plates = rospy.get_param('~no_plates', 5)
    distance = rospy.get_param('~distance', 2.5)
    area_size = rospy.get_param('~area_size', 10.0)

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_plate = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    # Delete plates
    rospy.wait_for_service('gazebo/delete_model')
    delete_plate = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    # Load aruco markers
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_aruco = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    # Delete aruco markers
    rospy.wait_for_service('gazebo/delete_model')
    delete_aruco = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    plate_poses = random_poses(no_plates, distance, area_size)
    plate_model = open(f"{dir}/models/plate/model.sdf", "r").read()
    plates = []
    for i in range(no_plates):
        plates.append('plate' + str(i + 1))

    for plate, pose in zip(plates, plate_poses):
        rospy.loginfo(f"Spawning {plate}")
        spawn_plate(plate, plate_model, "", pose, "world")