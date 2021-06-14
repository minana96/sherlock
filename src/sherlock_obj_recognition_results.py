#!/usr/bin/env python  

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout
from datetime import datetime

class SherlockObjRecognition:

    def __init__(self):
        self.objects_subscriber = rospy.Subscriber("/objects", Float32MultiArray, self.callback_objects)

    def callback_objects(self, msg):
        current_time = datetime.now()
        time_as_string = "(" + current_time.strftime("%H:%M:%S.%f")[:-3] + ")"
        recognised_objects = msg.data
        
        self.currently_recognised = set(recognised_objects[::12]) 
        self.print_object_ids(time_as_string)
    
    def print_object_ids(self, current_time):
        if len(self.currently_recognised) == 0:
            rospy.loginfo(current_time + " No object recognised")
        else:
            object_output = ", ".join([("Object " + str(int(obj_id))) for obj_id in self.currently_recognised])
            rospy.loginfo(current_time + " " + object_output + " detected!")


if __name__ == '__main__':
    rospy.init_node('sherlock_obj_recognition_output')
    try:
        SherlockObjRecognition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass