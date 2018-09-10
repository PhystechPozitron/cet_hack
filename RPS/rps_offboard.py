#!/usr/bin/env python

import rospy
import numpy
import math

from geometry_msgs.msg import PoseStamped, Vector3
from mavros_msgs.msg import State, AttitudeTarget
from clever.srv import Navigate
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix

from threading import Lock

#--- parameters --#

rospy.init_node('rps_offboard')
rospy.loginfo('initializing rps_offboard...')

lock = Lock()
AT = AttitudeTarget
land_timeout = 2

coeffs = {}
coeffs['thrust_0'] = float(rospy.get_param('~thrust_0'))
coeffs['thrust_D'] = float(rospy.get_param('~thrust_D'))
coeffs['thrust_P'] = float(rospy.get_param('~thrust_P'))
coeffs['thrust_I'] = float(rospy.get_param('~thrust_I'))
coeffs['pitch_0'] = float(rospy.get_param('~pitch_0'))
coeffs['pitch_D'] = float(rospy.get_param('~pitch_D')) 
coeffs['pitch_P'] = float(rospy.get_param('~pitch_P'))
coeffs['pitch_I'] = float(rospy.get_param('~pitch_I'))
coeffs['pitch_max'] = float(rospy.get_param('~pitch_max'))
coeffs['roll_0'] = float(rospy.get_param('~roll_0'))
coeffs['roll_D'] = float(rospy.get_param('~roll_D'))
coeffs['roll_P'] = float(rospy.get_param('~roll_P'))
coeffs['roll_I'] = float(rospy.get_param('~roll_I'))
coeffs['roll_max'] = float(rospy.get_param('~roll_max'))

camera_angle = float(rospy.get_param('~camera_angle'))
takeoff_time = float(rospy.get_param('~takeoff_time'))
hold_yaw = bool(rospy.get_param('~hold_yaw'))

#--- functions ---#


def get_state(data):
	global state
	state = data


def get_att(data):
	global att
	q = [0 for i in range(4)]

	q[0] = data.pose.orientation.x
	q[1] = data.pose.orientation.y
	q[2] = data.pose.orientation.z
	q[3] = data.pose.orientation.w
	att = euler_from_quaternion(q)


def get_position(data):
	global pose, Ix, Iy, Iz, pose_list

	if data.header.frame_id != '':
		pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
	else:
		pose = None
		pose_list = None
		Ix, Iy, Iz = 0, 0, 0


def transform_frame(vec, att):

	# from 'main_camera' to 'fcu'
	a = camera_angle
	vec[0], vec[1], vec[2] = vec[1]*math.cos(a) + vec[2]*math.sin(a), -vec[0], vec[2]*math.cos(a) - vec[1]*math.sin(a)

	# from 'fcu' to 'fcu_horiz'
	roll, pitch = att[0], att[1]	
	rmat = euler_matrix(roll, pitch, 0, 'rxyz')
	rmat = rmat[0:3, 0:3]
	vec = numpy.array([[vec[0]],[vec[1]],[vec[2]]])
	vec = rmat.dot(vec)

	return float(vec[0]), float(vec[1]), float(vec[2])


def get_PID(Px, Py, Pz):
	global pose_list, Ix, Iy, Iz

	# calculate P
	if pose_list is None:
		pose_list = [[Px, Py, Pz] for i in range(5)]
		Px_0, Py_0, Pz_0 = Px, Py, Pz
	else:
		pl = [pose_list[i + 1] for i in range(4)]
		pl = pl + [[Px, Py, Pz]]
		pose_list = pl

		sum_0 = [0, 0, 0]
		sum = [0, 0, 0]
		for i in range(3):
			for j in range(4):
				sum_0[i] += pl[j][i]
			for j in range(1, 5):
				sum[i] += pl[j][i]

		Px_0, Py_0, Pz_0 = sum_0[0]/4, sum_0[1]/4, sum_0[2]/4
		Px, Py, Pz = sum[0]/4, sum[1]/4, sum[2]/4

	# calculate I
	Ix += 0.01*Px
	Iy += 0.01*Py
	Iz += 0.01*Pz

	# calculate D
	Dx, Dy, Dz = Px - Px_0, Py - Py_0, Pz - Pz_0

	return Px, Py, Pz, Ix, Iy, Iz, Dx, Dy, Dz


def calculate_setpoint(att, pose, sp_pose, sp_yaw):

	# calculate PID in frame 'fcu_horiz'

	pose = list(pose)
	x, y, z = transform_frame(pose, att)
	sx, sy, sz = sp_pose[0], sp_pose[1], sp_pose[2]

	Px, Py, Pz, Ix, Iy, Iz, Dx, Dy, Dz = get_PID(x + sx, y + sy, z + sz)	
	print('Px = {0:.2f}, Py = {1:.2f}, Pz = {2:.2f}'.format(Px, Py, Pz))

	# calculate attitude

	roll = coeffs['roll_0'] + Dy*coeffs['roll_D'] + Py*coeffs['roll_P'] + Iy*coeffs['roll_I']
	if abs(roll) > coeffs['roll_max']:
		roll = math.copysign(coeffs['roll_max'], roll)

	pitch = coeffs['pitch_0'] - Dx*coeffs['pitch_D'] - Px*coeffs['pitch_P'] - Ix*coeffs['pitch_I']
	if abs(pitch) > coeffs['pitch_max']:
		pitch = math.copysign(coeffs['pitch_max'], pitch)

	if hold_yaw:
		yaw = sp_yaw
	else:
		yaw = att[2]

	print('sp_yaw = {0:.2f} '.format(yaw))
	sp_att = [roll, pitch, yaw]  # roll, pitch, yaw 

	# calculate thrust
	
	sp_thrust = coeffs['thrust_0'] - Dz*coeffs['thrust_D'] - Pz*coeffs['thrust_P'] - Iz*coeffs['thrust_I']
	if sp_thrust < 0:
		sp_thrust = 0
	if sp_thrust > 1:
		sp_thrust = 1
	print('roll = {0:.2f}, pitch = {1:.2f}, yaw = {2:.2f}, thrust = {3:.2f}'.format(roll, pitch, att[2], sp_thrust))
	
	return sp_att, sp_thrust


def get_message(sp_att, sp_thrust):

	msg = AttitudeTarget()
	msg.header.stamp = rospy.get_rostime() 
	msg.type_mask = AT.IGNORE_ROLL_RATE + AT.IGNORE_PITCH_RATE + AT.IGNORE_YAW_RATE

	q = quaternion_from_euler(sp_att[0],sp_att[1],sp_att[2])
	msg.orientation.x = q[0]
	msg.orientation.y = q[1]
	msg.orientation.z = q[2]
	msg.orientation.w = q[3]
	msg.thrust = sp_thrust

	return msg


def offboard_and_arm(auto_arm):
	global state

	# switch mode to 'OFFBOARD'
	if state.mode != 'OFFBOARD':
		rospy.sleep(0.2)
		set_mode(0, 'OFFBOARD')
		rospy.loginfo('Switch mode to "OFFBOARD"')

	# arming
	if (not state.armed) and (auto_arm):
		arming(True)
		rospy.loginfo('Arming')

#--- services ---#

def navigate(data):
	global att, pose, sp_pose, sp_yaw, takeoff_cnt

	if data.frame_id != 'fcu_horiz':
		rospy.logwarn('sp_pose must be given in "fcu_horiz" ')
		return {'message': 'sp_pose must be given in "fcu_horiz" '}

	try:
		with lock:
			sp_pose = [data.x, data.y, data.z]
			
			takeoff_cnt = 0

			sp_yaw = att[2]
			sp_att = [coeffs['roll_0'], coeffs['pitch_0'], sp_yaw]
			sp_thrust = coeffs['thrust_0'] 

			msg = get_message(sp_att, sp_thrust)
			att_pub.publish(msg)

			offboard_and_arm(data.auto_arm)

		rospy.loginfo('rps_offboard: navigate')
		return {'success': True}

	except Exception as e:
		rospy.logerr(str(e))
		return {'success': False, 'message': str(e)}


def land(data):
	global state, sp_pose, Ix, Iy, Iz

	land_cnt = 0
	while (state.mode != 'AUTO.LAND') and (land_cnt < 10*land_timeout):
		set_mode(0,'AUTO.LAND')
		rospy.sleep(0.1)
		land_cnt += 1

	if land_cnt == 10*land_timeout:
		rospy.loginfo('"AUTO.LAND" timeout')
		return {'success': False}
	else:
		Ix, Iy, Iz = 0, 0, 0
		sp_pose = None
		rospy.loginfo('Switch mode to "AUTO.LAND"')
		return {'success': True}


def release(data):
	global sp_pose, Ix, Iy, Iz

	Ix, Iy, Iz = 0, 0, 0
	sp_pose = None
	rospy.loginfo('rps_offboard: release')
	return {'success': True}

#--- main loop ---#

def loop():
	global att, pose, sp_pose, sp_yaw, takeoff_cnt
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		if not (sp_pose is None):
			with lock:
				if (pose is None):
					sp_att = [coeffs['roll_0'], coeffs['pitch_0'], sp_yaw]
					sp_thrust = coeffs['thrust_0']
					print('no pose')

				elif takeoff_cnt is None:
					sp_att, sp_thrust = calculate_setpoint(att, pose, sp_pose, sp_yaw)
				else:
					sp_att = [coeffs['roll_0'], coeffs['pitch_0'], sp_yaw]
					sp_thrust = coeffs['thrust_0']

					print('taking off...')
					takeoff_cnt += 1
					if takeoff_cnt >= takeoff_time*10:
						takeoff_cnt = None

				msg = get_message(sp_att, sp_thrust)
				att_pub.publish(msg)

		r.sleep()

#--- actions ---#

att = [0, 0, 1.57]
pose = None 
sp_pose = None
sp_yaw = 1.57
state = None

pose_list = None
Ix, Iy, Iz = 0, 0, 0  
takeoff_cnt = None

state_sub = rospy.Subscriber('/mavros/state', State, get_state)
pose_sub = rospy.Subscriber('/RPS/pose', PoseStamped, get_position)
att_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, get_att) 

att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode, persistent=True)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, persistent=True)

rospy.Service('navigate', Navigate, navigate)
rospy.Service('land', Trigger, land)
rospy.Service('release', Trigger, release)

rospy.loginfo('rps_offboard inited')
loop()
