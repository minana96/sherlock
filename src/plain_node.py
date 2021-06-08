#!/usr/bin/env python  

import rospy


if __name__ == '__main__':
    rospy.init_node('plain_node')

    # Initial delay
    rospy.loginfo('Plain node started')
    rospy.sleep(10)
    rospy.loginfo('Plain node ended')
