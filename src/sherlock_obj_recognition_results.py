#!/usr/bin/env python  

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout


class SherlockObjRecognition:

    def __init__(self):
        self.objects_subscriber = rospy.Subscriber("/objects", Float32MultiArray, self.callback_objects)
        self.previously_recognised = set()
        self.object_offset = 12
        self.object_ids = rospy.get_param('~object_ids', {})


    def callback_objects(self, msg):
        recognised_objects = msg.data
        currently_recognised = set(recognised_objects[::self.object_offset])

        # If recognised objects in this frame are different than ones in the previous frame
        if not ((self.previously_recognised <= currently_recognised) and (currently_recognised <= self.previously_recognised)):
            self.print_set(currently_recognised)

        self.previously_recognised = currently_recognised

    
    def print_set(self, set_of_object_ids):
        if len(set_of_object_ids) == 0:
            rospy.loginfo("No object recognised")
            return

        try:
            object_output = ", ".join([self.object_ids[str(int(obj_id))] for obj_id in set_of_object_ids])
            rospy.loginfo(object_output + " recognised")
        except KeyError:
            rospy.logerr("Invalid key!")



if __name__ == '__main__':
    rospy.init_node('sherlock_obj_recognition_output')
    SherlockObjRecognition()
    rospy.spin()