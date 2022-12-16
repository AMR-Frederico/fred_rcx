#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs import range
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Bool

loop_hz = 5
start = False

# PUBS ---------------------------------
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
led_strip_pub = rospy.Publisher("/cmd/led_strip/color", Float32, queue_size=10)

encoder_left_msg = 0
encoder_right_msg = 0

cmd_vel = Twist()

ticks_target_right = 2100
ticks_target_left = 2100


def encoder_left(msg):
    global encoder_left_msg
    encoder_left_msg = msg.data


def encoder_right(msg):
    global encoder_right_msg
    encoder_right_msg = msg.data


def start_cb(msg):
    global start
    start = msg.data


if __name__ == '__main__':
    rospy.init_node('rcx_take_time')
    rospy.Subscriber("competion/start", Bool, start_cb)

    rospy.Subscriber("power/status/distance/ticks/right",
                     Float32, encoder_right)
    rospy.Subscriber("/power/status/distance/ticks/left",
                     Float32, encoder_left)

    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        cmd_vel.linear.x = 0
        if(start):
            print(
                f"procurando -- LEFT : {encoder_left_msg} RIGHT : {encoder_right_msg}")
            cmd_vel.linear.x = 10
            if((encoder_left_msg > ticks_target_left) and (encoder_right_msg > ticks_target_right)):
                print("chegou")
                led_strip_pub.publish(1)
                cmd_vel.linear.x = 0

        cmd_vel_pub.publish(cmd_vel)
        rate.sleep()
