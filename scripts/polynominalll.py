#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32


def subscriber_callback(msg):
    rospy.loginfo('Bump number in power:{}'.format(msg.data))


def subscribe_tred():
    
    rospy.init_node('polynominal', anonymous=False)
    rospy.Subscriber('pubnum', Int32, subscriber_callback)
    pubsumm = rospy.Publisher('summ', Int32, queue_size=10)
    while not rospy.is_shutdown():
        rospy.sleep(15)

        numi = rospy.get_param('/ros_glob_param')
        exponents = [int(numi) ** (i+1) for i, numi in enumerate(numi)]
        rospy.sleep(1)
        print(exponents)
        rospy.set_param('/ros_glob_param_polinom', exponents)


if __name__ == '__main__':
    try:
        subscribe_tred()
    except rospy.ROSInterruptException:
        pass
