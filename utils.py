from geometry_msgs.msg import Pose, PoseWithCovariance
import numpy as np

def dict_to_point_obj(input, _2D=False):
    if _2D:
        return np.array([input['x'], input['y']])
    return np.array([input['x'], input['y'], input['z']])

def dict_to_ros_pose(input):
    pose = Pose()
    pose.position.x = input['position']['x']
    pose.position.y = input['position']['y']
    pose.position.z = input['position']['z']
    pose.orientation.x = input['quaternion']['x']
    pose.orientation.y = input['quaternion']['y']
    pose.orientation.z = input['quaternion']['z']
    pose.orientation.w = input['quaternion']['w']
    return pose

def obj_to_ros_pose(position, orientation):
    """
    :param position: np array
    :param orientation: np array
    :return:
    """
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

def array_to_dict_pos(input):
    return {
        "x": input[0],
        "y": input[1],
        "z": input[2]
    }
def array_to_dict_orient(input):
    return {
        "x": input[0],
        "y": input[1],
        "z": input[2],
        "w": input[3]
    }
def quaternion_from_yaw(yaw_angle_rad):
    # Assuming rotation around Z-axis only
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw_angle_rad / 2.0)
    qw = np.cos(yaw_angle_rad / 2.0)
    return [qx, qy, qz, qw]