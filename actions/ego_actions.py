from core.action import Action
import utils

from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped

class SpawnEgo(Action):
    def __init__(self, position, orientation):
        super().__init__(condition=None, one_shot=True)
        self.position = position
        self.orientation = orientation

    def _do(self, actor):
        """
        Spawn the ego vehicle and request (re-)localization
        :param position:
        :param orientation:
        :return:
        """
        ros_pose = utils.obj_to_ros_pose(self.position, self.orientation)
        cov = [0.0] * 36
        cov[0] = 1e-4  # x
        cov[7] = 1e-4  # y
        cov[35] = 1e-4  # yaw

        # publish a pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = actor.client_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = ros_pose
        msg.pose.covariance = cov

        actor.client_node.ego_pose_publisher.publish(msg)

        ok = actor.client_node.re_localization(ros_pose, cov)
        if ok:
            actor.client_node.get_logger().info("Spawned ego and re-localized successfully.")
            # re-localization succeeded
            # self.ads_internal_status = AdsInternalStatus.GOAL_SET
            # self.loop()
        else:
            actor.client_node.get_logger().error("Failed to localize ego.")

class SetGoalPose(Action):
    def __init__(self, position, orientation):
        super().__init__(one_shot=True)
        self.position = position
        self.orientation = orientation

    def _do(self, actor):
        """
        Set goal for autonomous driving
        :param goal_position:
        :param goal_orientation:
        :return:
        """
        ros_goal = utils.obj_to_ros_pose(self.position, self.orientation)
        actor.client_node.set_goal(ros_goal)


class ActivateAutonomousMode(Action):
    def __init__(self):
        super().__init__(one_shot=True)

    def _do(self, actor):
        pass
