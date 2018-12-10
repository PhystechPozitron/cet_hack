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
sp_f = 20 # frequency of setpoints for the flight controller
filter_xyz = [140., 120., 120.] # model_err/measure_err ratio for the x, y, z filtering
filter_yaw = 0.03 # Kalman index for the yaw filtering

#--- parameters --#
coeffs = {}
coeffs['thrust_0'] = float(rospy.get_param('~thrust_0'))
coeffs['thrust_D'] = float(rospy.get_param('~thrust_D'))
coeffs['thrust_P'] = float(rospy.get_param('~thrust_P'))
coeffs['thrust_I'] = float(rospy.get_param('~thrust_I'))
coeffs['thrust_min'] = float(rospy.get_param('~thrust_min', 0.25))
coeffs['thrust_max'] = float(rospy.get_param('~thrust_max', 0.54))
coeffs['thrust_compensation'] = float(rospy.get_param('~thrust_compensation'))
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
coeffs['yaw_I'] = float(rospy.get_param('~yaw_I')) 

takeoff_time = float(rospy.get_param('~takeoff_time'))
navigate_time = float(rospy.get_param('~navigate_time'))
landing_time = float(rospy.get_param('~landing_time', 4))

camera_angle = float(rospy.get_param('~camera_angle'))

#--- functions ---#

def calculate_abg():
	alpha, beta, gamma = [], [], []
	l, r = [], []
	for i in range(3):
		l.append(filter_xyz[i]/(f**2)) 
		r.append((4 + l[i] - math.sqrt(8*l[i] + l[i]**2))/4)

		alpha.append(1 - r[i]**2)
		beta.append(2*(2 - alpha[i]) - 4*math.sqrt(1 - alpha[i]))
		gamma.append((beta[i]**2)/(2*alpha[i]))
	return alpha, beta, gamma


def get_state(data):
	global state
	state = data


def set_init_thrust(data):
	global init_thrust

	perc = data.percentage
	thrust_0 = coeffs['thrust_0']
	comp = coeffs['thrust_compensation']
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
			heading = filter_yaw*heading_meas + (1 - filter_yaw)*heading
	else:
		pose = None
		orient = None
		heading = None


def calculate_force():
	global ab_filter_list, abg_filter_list
	force = [0 for i in range(3)]

	for i in range(3):
		if i == 0:
			value_type = 'pitch'
			sign = -1
		elif i == 1:
			value_type = 'roll'
			sign = 1
		elif i == 2:
			value_type = 'thrust'
			sign = -1	

		P, I, D = coeffs[value_type + '_P'], coeffs[value_type + '_I'], coeffs[value_type + '_D']
		Pe, Ie, De = ab_filter_list[i][0], ab_filter_list[i][2], abg_filter_list[i][1] 
		force[i] = sign*(P*Pe + I*Ie + D*De)
		if i == 2:
			force[i] += init_thrust
   
	return force


def edit_filter_lists(offset):
	global ab_filter_list, abg_filter_list

	if ab_filter_list is None:
		ab_filter_list = [[] for i in range(3)]
		abg_filter_list = [[] for i in range(3)]
		for i in range(3):
			Pe, De, Ie = offset[i], 0, 0
			ab_filter_list[i] = [Pe, De, Ie]
			Pe, De, Ae = offset[i], 0, 0
			abg_filter_list[i] = [Pe, De, Ae]
	else:
		for i in range(3):
			# edit ab_filter_list
			Pe_meas = offset[i]
			Pe_pred = ab_filter_list[i][0] + ab_filter_list[i][1]/f
			Pe = alpha[i]*(Pe_meas - Pe_pred) + Pe_pred
			Ie = ab_filter_list[i][2] + Pe/f
			De_pred = ab_filter_list[i][1]
			De = beta[i]*f*(Pe_meas - Pe_pred) + De_pred

			ab_filter_list[i] = [Pe, De, Ie]

			# edit abg_filter_list
			Pe_pred = abg_filter_list[i][0] + abg_filter_list[i][1]/f + abg_filter_list[i][2]/(2*(f**2))  
			Pe = alpha[i]*(Pe_meas - Pe_pred) + Pe_pred
			De_pred = abg_filter_list[i][1] + abg_filter_list[i][2]/f
			De = beta[i]*f*(Pe_meas - Pe_pred) + De_pred
			Ae_pred = abg_filter_list[i][2]
			Ae = gamma[i]*2*(f**2)*(Pe_meas - Pe_pred) + Ae_pred

			abg_filter_list[i] = [Pe, De, Ae]


def get_sp_pose(att, pose, orient, target, flight_cnt):

	if target is None:
		sp_pose = None
	else:
		x, y, z = target[0], target[1], target[2]
		if target[3] == 'aruco_map':
			tar = transform_frame([x, y, z], orient, att, 'aruco_map')
		elif target[3] == 'fcu_horiz':
			tar = [x, y, z]
	
		rel = flight_cnt/(f*navigate_time)	
		sp_pose = [pose[i] + rel*(tar[i] - pose[i]) for i in range(3)]

	return sp_pose


def get_sp_att(pose, yaw, heading, sp_pose, sp_heading, init_thrust, mode, flight_cnt):
	global ab_filter_list
			
	if mode == 'landing':
		ab_filter_list = None
		sp_roll = coeffs['roll_0'] 
		sp_pitch = coeffs['pitch_0']
		rel = flight_cnt/(f*landing_time)
		sp_thrust = init_thrust - rel*(init_thrust - coeffs['thrust_min']) 
		sp_yaw = yaw

	elif (mode == 'positioning') and not(pose is None) and not(sp_pose is None):
		# edit offset list and calculate vector of force
		offset = [pose[i] - sp_pose[i] for i in range(3)]
		edit_filter_lists(offset)
		force = calculate_force()

		# calculate thrust	
		sp_thrust = math.sqrt(force[0]**2 + force[1]**2 + (force[2])**2)
		if sp_thrust < coeffs['thrust_min']:
			sp_thrust = coeffs['thrust_min']
		if sp_thrust > coeffs['thrust_max']:
			sp_thrust = coeffs['thrust_max']

		# calculate roll and pitch	
		sp_roll = coeffs['roll_0'] + force[1]/sp_thrust
		if abs(sp_roll) > coeffs['roll_max']:
			sp_roll = math.copysign(coeffs['roll_max'], sp_roll)

		sp_pitch = coeffs['pitch_0'] + force[0]/sp_thrust  
		if abs(sp_pitch) > coeffs['pitch_max']:
			sp_pitch = math.copysign(coeffs['pitch_max'], sp_pitch)

		# calculate yaw
		if not(heading is None) and not(sp_heading is None):
			sp_yaw = yaw - coeffs['yaw_I']*(heading - sp_heading)/f 
		else:
			sp_yaw = yaw

	else:
		ab_filter_list = None
		sp_roll = coeffs['roll_0'] 
		sp_pitch = coeffs['pitch_0']
		sp_thrust = init_thrust
		sp_yaw = yaw

	sp_att = [sp_roll, sp_pitch, sp_yaw, sp_thrust]
	return sp_att


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

	if (state.armed):
		arming(False)
		rospy.loginfo('Disarming')

#--- services ---#

def navigate(data):
	global target

	if (data.frame_id != 'aruco_map') and (data.frame_id != 'fcu_horiz'):
		rospy.logwarn('Incorrect frame_id for target')
		return {'message': 'Incorrect frame_id for target'}

	try:		
		# set target mode
		if data.auto_arm:
			order = 'take_off'
		else:
			order = 'fly_to_point' 

		# set target
		with lock:
			target = [data.x, data.y, data.z, data.frame_id, 0, order]

		# switch mode to OFFBOARD
		offboard_and_arm(data.auto_arm)

		rospy.loginfo('rps_offboard: navigate')
		return {'success': True}

	except Exception as e:
		rospy.logerr(str(e))
		return {'success': False, 'message': str(e)}


def land(data):
	global state, target
	order = 'land'
	with lock:
		target = [0, 0, 0, '', 0, order]

	rospy.loginfo('starting to land...')
	return {'success': True}


def release(data):
	global target

	target = None
	rospy.loginfo('rps_offboard: release')
	return {'success': True}

#--- main loop ---#

def loop():
	global att, heading, pose, orient, target, init_thrust
	global ab_filter_list, abg_filter_list
	r = rospy.Rate(f)
	timer = 0
	order = ''
	mode = ''
	flight_cnt = 0

	while not rospy.is_shutdown():
		if timer == f:
			timer = 0
		if not (target is None):

			# order processing
			if order != target[5]:
				if target[5] == 'take_off':
					mode = 'takeoff'
				elif target[5] == 'fly_to_point':
					mode = 'positioning'
				elif target[5] == 'land':
					mode = 'landing'
				flight_cnt = 0
				yaw = att[2]  
			order = target[5]

			# mode actions
			if mode == 'takeoff':
				flight_cnt += 1
				sp_pose = None
				sp_heading = None
				if flight_cnt >= f*takeoff_time:
					flight_cnt = 0
					mode = 'positioning'
			elif mode == 'positioning':
				if pose is None:
					flight_cnt = 0
					sp_pose = None
					sp_heading = None
				else:
					if flight_cnt == 0:
						init_pose = list(pose)
					if flight_cnt % (f//sp_f) == 0:
						sp_pose = get_sp_pose(att, init_pose, orient, target, flight_cnt)
						sp_heading = target[4]
					if flight_cnt < f*navigate_time:
						flight_cnt += 1
			elif mode == 'landing':
				flight_cnt += 1
				sp_pose = None
				sp_heading = None
				if flight_cnt >= f*landing_time:
					target = None
					disarm()

			# send sp to flight controller
			sp_att = get_sp_att(pose, yaw, heading, sp_pose, sp_heading, init_thrust, mode, flight_cnt)
			yaw = sp_att[2]
			if timer % (f//sp_f) == 0:
				if not (ab_filter_list is None) and not (pose is None):
					print('{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f} {5:.6f} {6:.6f} {7:.6f} {8:.6f}'.format(pose[0], pose[1], pose[2], ab_filter_list[0][0], ab_filter_list[1][0], ab_filter_list[2][0], abg_filter_list[0][1], abg_filter_list[1][1], abg_filter_list[2][1]))
				msg = get_message(sp_att)
				att_pub.publish(msg)
		else:
			order = ''
			mode = ''
			flight_cnt = 0

		r.sleep()
		timer += 1

#--- actions ---#

init_thrust = coeffs['thrust_0']
att = [0, 0, 1.57]
pose = None 
orient = None
target = None
state = None
heading = None
ab_filter_list = None
abg_filter_list = None 
alpha, beta, gamma = calculate_abg()

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
