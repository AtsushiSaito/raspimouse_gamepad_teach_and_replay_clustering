#!/usr/bin/env python
# Copyright 2017 Ryo Okazaki
# Copyright 2017 Ryuichi Ueda
# Released under the BSD License.

import rospy
from raspimouse_ros_2.msg import ButtonValues, LedValues

class LedControl(object):
    def __init__(self):
        rospy.Subscriber('/buttons', ButtonValues, self.button_callback, queue_size=1)
        self._led_pub = rospy.Publisher('/leds', LedValues, queue_size=1)

    def button_callback(self, btn_msg):
	leds = LedValues()
	leds.left_side = btn_msg.front_toggle
	leds.left_forward = btn_msg.mid_toggle
	self._led_pub.publish(leds)

if __name__ == '__main__':
    rospy.init_node('ledcontrol')
    ledc = LedControl()
    rospy.spin()
