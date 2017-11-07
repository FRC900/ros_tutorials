#!/usr/bin/env python2

import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher("chatter", String, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(4)  # In Hertz
    while not rospy.is_shutdown():
        send = "Hello World: " + str(rospy.get_time())
        rospy.loginfo(send)
        pub.publish(send)
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
