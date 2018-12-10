#!/usr/bin/env python

import rospy
import numpy
import math

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from sensor_msgs.msg import BatteryState, Imu

from clever.srv import Navigate
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger

from cv2 import Rodrigues
from tf.transformations import quaternion_from_euler, euler_from_quaternion, euler_matrix
from threading import Lock
lock = Lock()

rospy.init_node('rps_offboard')
rospy.loginfo('initializing rps_offboard...')

#--- constants ---#

AT = AttitudeTarget
f = 200 # system frequency
sp_f = 20 # frequency of sending messages to the flight controller 

#--- parameters --#
camera_angle = float(rospy.get_param('~camera_angle', 1.57))
landing_time = float(rospy.get_param('~landing_time', 4))

params = {}
params['thrust_0'] = float(rospy.get_param('~thrust_0', 0.3))
params['thrust_min'] = float(rospy.get_param('~thrust_min', 0.25))
params['thrust_max'] = float(rospy.get_param('~thrust_max', 0.45))
params['thrust_compensation'] = float(rospy.get_param('~thrust_compensation'))

params['pitch_0'] = float(rospy.get_param('~pitch_0', 0))
params['pitch_max'] = float(rospy.get_param('~pitch_max', 0.3))

params['roll_0'] = float(rospy.get_param('~roll_0', 0))
params['roll_max'] = float(rospy.get_param('~roll_max', 0.3))

params['filter_pose'] = float(rospy.get_param('~filter_pose', 120.))
params['filter_yaw'] = float(rospy.get_param('~filter_yaw', 0.03))

params['Px'] = float(rospy.get_param('~Px'))
params['Py'] = float(rospy.get_param('~Py'))
params['Pz'] = float(rospy.get_param('~Pz'))

params['Ix'] = float(rospy.get_param('~Ix')) 
params['Iy'] = float(rospy.get_param('~Iy'))
params['Iz'] = float(rospy.get_param('~Iz'))

params['Dx'] = float(rospy.get_param('~Dx'))
params['Dy'] = float(rospy.get_param('~Dy'))
params['Dz'] = float(rospy.get_param('~Dz'))

params['I_yaw'] = float(rospy.get_param('~I_yaw')) 

#--- functions ---#

def calculate_abg():
	l = params['filter_pose']/(f**2)
	r = (4 + l - math.sqrt(8*l + l**2))/4

	alpha = 1 - r**2
	beta = 2*(2 - alpha) - 4*math.sqrt(1 - alpha)
	gamma = beta**2/(2*alpha)
	return alpha, beta, gamma


def get_state(data):
	global state
	state = data


def set_init_thrust(data):
	global init_thrust

	perc = data.percentage
	thrust_0 = params['thrust_0']
	comp = params['thrust_compensation']
	init_thrust = thrust_0 + comp*(1 - perc)
	print('charge: ' + str(int(perc*100)) + '%')


def get_att(data):
	global att
	q = [0 for i in range(4)]

	q[0] = data.pose.orientation.x
	q[1] = data.pose.orientation.y
	q[2] = data.pose.orientation.z
	q[3] = data.pose.orientation.w
	att = euler_from_quaternion(q)


def transform_frame(vec, orient, att, frame_id):

	vec = numpy.array(vec)

	if frame_id == 'aruco_map':
		# from 'aruco_map' to 'main_camera'
		rmat, j = Rodrigues(orient)
		vec.transpose()
		vec = rmat.dot(vec)
		vec.transpose()
	elif frame_id == 'main_camera':
		pass

	# from 'main_camera' to 'fcu'
	a = camera_angle
	vec[0], vec[1], vec[2] = vec[1]*math.cos(a) + vec[2]*math.sin(a), -vec[0], vec[2]*math.cos(a) - vec[1]*math.sin(a)

	# from 'fcu' to 'fcu_horiz'
	roll, pitch = att[0], att[1]	
	rmat = euler_matrix(roll, pitch, 0, 'rxyz')
	rmat = rmat[0:3, 0:3]
	vec.transpose()
	vec = rmat.dot(vec)
	vec.transpose()

	return list(vec)


def calc_heading(orient, att):
	norm_vec = [0, 0, 1] # norm vector to markers'/leds' plate
	n = transform_frame(norm_vec, orient, att, 'aruco_map')
	heading_meas = math.asin(n[1]/math.sqrt(n[0]**2 + n[1]**2))
	return heading_meas


def get_position(data):
	global att, pose, orient
	global heading

	if data.header.frame_id == 'main_camera':
		x, y, z = data.pose.position.x, data.pose.position.y, data.pose.position.z 
		orient = numpy.array([[data.pose.orientation.x], [data.pose.orientation.y], [data.pose.orientation.z]])
		pose = transform_frame([x, y, z], orient, att, 'main_camera')
		heading_meas = calc_heading(orient, att)
		if heading is None:
			heading = heading_meas
		else:
			heading = params['filter_yaw']*heading_meas + (1 - params['filter_yaw'])*heading
	else:
		pose = None
		orient = None
		heading = None


def calculate_force(ab_list, abg_list):
	force = [0 for i in range(3)]

	for i in range(3):
		if i == 0:
			value_type = 'x'
			sign = -1
		elif i == 1:
			value_type = 'y'
			sign = 1
		elif i == 2:
			value_type = 'z'
			sign = -1	

		P, I, D = params['P' + value_type], params['I' + value_type], params['D' + value_type]
		Pe, Ie, De = ab_list[i][0], ab_list[i][2], abg_list[i][1] 
		force[i] = sign*(P*Pe + I*Ie + D*De)
		if i == 2:
			force[i] += init_thrust
   
	return force


def edit_filter_lists(offset, ab_list, abg_list):

	if ab_list is None:
		ab_list = [[] for i in range(3)]
		abg_list = [[] for i in range(3)]
		for i in range(3):
			Pe, De, Ie = offset[i], 0, 0
			ab_list[i] = [Pe, De, Ie]
			Pe, De, Ae = offset[i], 0, 0
			abg_list[i] = [Pe, De, Ae]
	else:
		for i in range(3):
			# edit ab_filter_list
			Pe_meas = offset[i]
			Pe_pred = ab_list[i][0] + ab_list[i][1]/f
			Pe = alpha*(Pe_meas - Pe_pred) + Pe_pred
			Ie = ab_list[i][2] + Pe/f
			De_pred = ab_list[i][1]
			De = beta*f*(Pe_meas - Pe_pred) + De_pred

			ab_list[i] = [Pe, De, Ie]

			# edit abg_filter_list
			Pe_pred = abg_list[i][0] + abg_list[i][1]/f + abg_list[i][2]/(2*(f**2))  
			Pe = alpha*(Pe_meas - Pe_pred) + Pe_pred
			De_pred = abg_list[i][1] + abg_list[i][2]/f
			De = beta*f*(Pe_meas - Pe_pred) + De_pred
			Ae_pred = abg_list[i][2]
			Ae = gamma*2*(f**2)*(Pe_meas - Pe_pred) + Ae_pred

			abg_list[i] = [Pe, De, Ae]

	return ab_list, abg_list


def get_setpoint(att, orient, target):

	if target is None:
		setpoint = None
	else:
		x, y, z = target[0], target[1], target[2]
		if target[3] == 'aruco_map':
			sp_pose = transform_frame([x, y, z], orient, att, 'aruco_map')
		elif target[3] == 'fcu_horiz':
			sp_pose = [x, y, z]
		sp_heading = target[4]
		setpoint = [sp_pose[0], sp_pose[1], sp_pose[2], sp_heading]

	return setpoint


def get_sp_att(sp_att, pose, heading, setpoint, ab_list, abg_list):
			
	if not(pose is None) and not(setpoint is None):

		# edit offset list and calculate vector of force
		offset = [pose[i] - setpoint[i] for i in range(3)]
		ab_list, abg_list = edit_filter_lists(offset, ab_list, abg_list)
		force = calculate_force(ab_list, abg_list)

		# calculate thrust	
		sp_thrust = math.sqrt(force[0]**2 + force[1]**2 + (force[2])**2)
		if sp_thrust < params['thrust_min']:
			sp_thrust = params['thrust_min']
		if sp_thrust > params['thrust_max']:
			sp_thrust = params['thrust_max']

		# calculate roll and pitch	
		sp_roll = params['roll_0'] + force[1]/sp_thrust
		if abs(sp_roll) > params['roll_max']:
			sp_roll = math.copysign(params['roll_max'], sp_roll)

		sp_pitch = params['pitch_0'] + force[0]/sp_thrust  
		if abs(sp_pitch) > params['pitch_max']:
			sp_pitch = math.copysign(params['pitch_max'], sp_pitch)

		# calculate yaw
		sp_heading = setpoint[3]
		if not(heading is None):
			sp_yaw = sp_att[2] - params['I_yaw']*(heading - sp_heading)/f 
		else:
			sp_yaw = sp_att[2]

	else:
		ab_list, abg_list = None, None
		sp_roll = params['roll_0'] 
		sp_pitch = params['pitch_0']
		sp_yaw = sp_att[2]
		sp_thrust = sp_att[3] - 5*(sp_att[3] - params['thrust_min'])/(f*landing_time) 

	sp_att = [sp_roll, sp_pitch, sp_yaw, sp_thrust]
	return sp_att, ab_list, abg_list


def get_message(sp_att):

	msg = AttitudeTarget()
	msg.header.stamp = rospy.get_rostime() 
	msg.type_mask = AT.IGNORE_ROLL_RATE + AT.IGNORE_PITCH_RATE + AT.IGNORE_YAW_RATE

	q = quaternion_from_euler(sp_att[0],sp_att[1],sp_att[2])
	msg.orientation.x = q[0]
	msg.orientation.y = q[1]
	msg.orientation.z = q[2]
	msg.orientation.w = q[3]
	msg.thrust = sp_att[3]

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

def disarm():
	global state

	if state.armed:
		arming(False)
		rospy.loginfo('Disarming')

def debug(pose, ab_list, abg_list, sp_att):
	string = '{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f} {5:.6f} {6:.6f} {7:.6f} {8:.6f}'
	if not (ab_list is None) and not (pose is None): 
		print(string.format(pose[0], pose[1], pose[2], ab_list[0][0], ab_list[1][0], ab_list[2][0], sp_att[0], sp_att[1], sp_att[3]))
	else:
		print(string.format(0, 0, 0, 0, 0, 0, sp_att[0], sp_att[1], sp_att[3]))

#--- services ---#

def navigate(data):
	global pose, target

	if (data.frame_id != 'aruco_map') and (data.frame_id != 'fcu_horiz'):
		rospy.logwarn('Incorrect frame_id for target')
		return {'message': 'Incorrect frame_id for target'}

	try:		
		if not(pose is None):

			# set target
			with lock:
				target = [data.x, data.y, data.z, data.frame_id, 0, 'flight']

			# switch mode to OFFBOARD
			offboard_and_arm(data.auto_arm)

			rospy.loginfo('rps_offboard: navigate')
			return {'success': True}
		else:
			rospy.loginfo('no position data, flight is impossible')
			return {'success': False}

	except Exception as e:
		rospy.logerr(str(e))
		return {'success': False, 'message': str(e)}


def land(data):
	global target
	if not(target is None):
		with lock:
			target = [0, 0, 0, '', 0, 'landing']

	rospy.loginfo('starting to land...')
	return {'success': True}


def release(data):
	global target

	target = None
	rospy.loginfo('rps_offboard: release')
	return {'success': True}

#--- main loop ---#

def loop():
	global att, heading, pose, orient, target
	r = rospy.Rate(f)
	timer = 0
	land_cnt = 0
	ab_list, abg_list = None, None
	sp_att = [params['roll_0'], params['pitch_0'], att[2], init_thrust]

	while not rospy.is_shutdown():
		if timer == f:
			timer = 0
		if not (target is None):

			if target[5] == 'flight':
				land_cnt = 0
				if pose is None:
					setpoint = None
					with lock:
						target[5] = 'landing'
				else:
					setpoint = get_setpoint(att, orient, target)
			else:
				land_cnt += 1
				setpoint = None
				if land_cnt >= f*landing_time:
					target = None
					disarm()

			# send sp to flight controller
			sp_att, ab_list, abg_list = get_sp_att(sp_att, pose, heading, setpoint, ab_list, abg_list)
			if timer % (f//sp_f) == 0:
				debug(pose, ab_list, abg_list, sp_att)
				msg = get_message(sp_att)
				att_pub.publish(msg)
		else:
			land_cnt = 0
			ab_list, abg_list = None, None
			sp_att = [params['roll_0'], params['pitch_0'], att[2], init_thrust]

		r.sleep()
		timer += 1

#--- actions ---#

alpha, beta, gamma = calculate_abg()

init_thrust = params['thrust_0']
att = [0, 0, 1.57]
pose = None 
orient = None
target = None
state = None
heading = None 

state_sub = rospy.Subscriber('/mavros/state', State, get_state)
pose_sub = rospy.Subscriber('/RPS/pose', PoseStamped, get_position)
att_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, get_att)
battery_sub = rospy.Subscriber('/mavros/battery', BatteryState, set_init_thrust)

att_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode, persistent=True)
arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool, persistent=True)

rospy.Service('navigate', Navigate, navigate)
rospy.Service('land', Trigger, land)
rospy.Service('release', Trigger, release)

rospy.loginfo('rps_offboard inited')
loop()
