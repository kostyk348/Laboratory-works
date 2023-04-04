#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def publish_cringe():
    pub = rospy.Publisher('anon_tred', Int32, queue_size=10)
    pub_4ch = rospy.Publisher('anon_tred_new', Int32, queue_size=10)
    rospy.init_node('cringe_publisher', anonymous=False)
    rate = rospy.Rate(10) # 10Hz
    num = 0
    while not rospy.is_shutdown():
        if num % 2 == 0:
            pub.publish(num)
        num += 1
        if num == 101:
            num = 0
            pub_4ch.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cringe()
    except rospy.ROSInterruptException:
        pass
	
