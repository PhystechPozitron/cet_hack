#!/usr/bin/env python

import rospy
import thread
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import SetMode
from std_srvs.srv import Trigger
from clever.srv import Navigate


armed = False
switch = 0

rospy.init_node('rps_init')

auto_arm = bool(rospy.get_param('~auto_arm', ''))
#offboard_mode = rospy.get_param('~offboard_mode')
flight_time = float(rospy.get_param('~flight_time'))

def action():
	global switch, armed

	navigate(x=1, y=0, z=0, frame_id='fcu_horiz', auto_arm=auto_arm)

	cnt = 0
	while cnt < 10*flight_time:
		if switch == 0:
			return 1
   
		cnt += 1
		rospy.sleep(0.1)

	land()
	while armed:
		if switch == 0:
			return 1

		rospy.sleep(0.1)

def get_state(data):
	global armed
	armed = data.armed

def rc_callback(data):
	global switch
	if data.channels[5] < 1100:
		if switch == 1:
			switch = 0
	elif data.channels[5] > 1900:
		if switch == 0:
			switch = 1

def flight(param):
	global switch

	while not rospy.is_shutdown():
		while switch == 0:
			rospy.sleep(0.1)

		if action() == 1:
			release()
			set_mode(0, 'STABILIZED')
			rospy.loginfo('switch mode to STABILIZED')

		while switch == 1:
			rospy.sleep(0.1)

def loop():
	r = rospy.Rate(20)

	while not rospy.is_shutdown():
		
		# publish dummy msg to init LPE
		msg = PoseStamped()
		msg.header.stamp = rospy.get_rostime()
		msg.pose.orientation.w = 1
		vision_pub.publish(msg)

		r.sleep()

navigate = rospy.ServiceProxy("navigate",Navigate)
land = rospy.ServiceProxy("land",Trigger)
release = rospy.ServiceProxy('/release', Trigger)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)

rc_sub = rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
state_sub = rospy.Subscriber('/mavros/state', State, get_state)

thread.start_new_thread(flight, ([],))
loop()