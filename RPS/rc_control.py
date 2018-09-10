#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

rospy.init_node('rc_control')

release = rospy.ServiceProxy('/release', Trigger)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

mode = 'MANUAL'
switch = 0

def get_mode(data):
	global mode
	mode = data.mode

def rc_callback(data):
        global mode, switch
        if data.channels[5] < 1100:
		if switch == 0:
			switch = 1
        elif data.channels[5] > 1900:
		if switch == 1:
			switch = 0

rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
rospy.Subscriber('/mavros/state', State, get_mode)

rospy.loginfo('rc_control inited')
rospy.spin()



