#!/usr/bin/env python  

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout

class SherlockObjRecognition:

    def __init__(self):
        self.objects_subscriber = rospy.Subscriber("/objects", Float32MultiArray, self.callback_objects)
        rospy.loginfo("SherlockObjRecognition results subscriber started")

    def callback_objects(self, msg):
        recognised_objects = msg.data
        
        self.currently_recognised = set(recognised_objects[::12]) 
        self.print_object_ids()
    
    def print_object_ids(self):
        if len(self.currently_recognised) == 0:
            rospy.loginfo("No object detected")
        else:
            object_output = ", ".join([("Object " + str(int(obj_id))) for obj_id in self.currently_recognised])
            rospy.loginfo(object_output + " detected!")


if __name__ == '__main__':
    rospy.init_node('sherlock_obj_recognition_output')
    try:
        SherlockObjRecognition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass