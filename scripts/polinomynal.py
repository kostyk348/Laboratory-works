#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32



def subscriber_callback(msg):
    rospy.loginfo('Bump number in power:{}'.format(msg.data))


def publish_numbers():
    pub = rospy.Publisher('pubnum', Int32, queue_size=10)
    rospy.init_node('request', anonymous=False)
    rate = rospy.Rate(10)  # 10Hz
    rospy.Subscriber('result', Int32, subscriber_callback)
    if subscriber_callback is not None:
        while not rospy.is_shutdown():
           nums = input("Enter a list of numbers separated by commas: ").split(",")
           rospy.set_param('/ros_glob_param', nums)


if __name__ == '__main__':
    try:
        publish_numbers()
    except rospy.ROSInterruptException:
        pass
