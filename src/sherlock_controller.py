#!/usr/bin/env python  

import rospy
import actionlib
from math import radians
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion


class SherlockController:

    def __init__(self):
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.reference_frame = rospy.get_param('~reference_frame', 'map')
        self.time_to_wait = rospy.get_param('~time_to_wait', 1)
        self.goal_poses = rospy.get_param('~goal_poses', [])


    # Make the robot move to the goal pose
    def move_to_goal(self, x_goal, y_goal, yaw):

        # Wait for the action server to be available
        self.ac.wait_for_server()
        rospy.loginfo("MoveBaseAction server ready")

        goal = self.create_goal(x_goal, y_goal, yaw)

        rospy.loginfo("Sending goal location: x={}, y={}, yaw={}".format(x_goal, y_goal, yaw))
        self.ac.send_goal(goal)

        # Wait for the result
        self.ac.wait_for_result()

        if(self.ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("The robot has reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False


    def create_goal(self, x_goal, y_goal, yaw):
        goal = MoveBaseGoal()

        # Set the header parameters
        goal.target_pose.header.frame_id = self.reference_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set position
        goal.target_pose.pose.position = Point(x_goal, y_goal, 0)

        # Set orientation
        q = quaternion_from_euler(0, 0, radians(yaw))
        goal.target_pose.pose.orientation = Quaternion(*q)

        return goal


    def spin(self):
        rospy.loginfo('Sherlock controller started...')

        for goal_pose in self.goal_poses:
            x_goal = goal_pose[0]
            y_goal = goal_pose[1]
            yaw = goal_pose[2]

            self.move_to_goal(x_goal, y_goal, yaw)

            rospy.sleep(self.time_to_wait)



if __name__ == '__main__':
    rospy.init_node('sherlock_controller')

    try:
        sc = SherlockController()
        sc.spin()
    except rospy.ROSInterruptException:
        pass
