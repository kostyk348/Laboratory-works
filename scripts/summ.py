#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def subscriber_callback(msg):
    rospy.loginfo('Bump number in power:{}'.format(msg.data))

def subscribe():
    rospy.init_node('summarize', anonymous=False)
    rospy.Subscriber('summ', Int32, subscriber_callback)
    pubsumm = rospy.Publisher('result', Int32, queue_size=10)
    while not rospy.is_shutdown():
         rospy.sleep(20)

         summic =rospy.get_param('/ros_glob_param_polinom')
         total = sum(summic)
         print(total)

if __name__ == '__main__':
    try:
        subscribe()
    except rospy.ROSInterruptException:
        pass
