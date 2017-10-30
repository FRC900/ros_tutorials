#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard " + str(rospy.get_caller_id()), data.data)

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()  # Keeps Python running until node is killed

if __name__ == "__main__":
    listener()
