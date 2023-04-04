#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def subscriber_callback(msg):
    rospy.loginfo('Bump number:{}'.format(msg.data))

def subscribe_tred():
    rospy.init_node('anon_subscriber', anonymous=False)
    rospy.Subscriber('anon_tred', Int32, subscriber_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscribe_tred()
    except rospy.ROSInterruptException:
        pass
