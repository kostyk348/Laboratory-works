#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def subscriber_callback(msg):
    rospy.loginfo("Tred overflow happen")

def subscribe_tred_new():
    rospy.init_node('anon_subscriber_new', anonymous=False)
    rospy.Subscriber('anon_tred_new',Int32,subscriber_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_tred_new()
    except rospy.ROSInterruptException:
        pass
