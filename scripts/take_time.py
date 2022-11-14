#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs import range
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool

# PUBS ---------------------------------
start_time = rospy.Publisher("competion/start", Bool, queue_size=1)


def start_cb(msg):
    global start
    start = msg.data


if __name__ == '__main__':
    rospy.init_node('rcx_take_time')
    rospy.Subscriber("competion/start", Bool, start_cb)
    global start
    start = False

    while not rospy.is_shutdown():
        if(start):
            print("--------Start---------------")

        rospy.spin()
