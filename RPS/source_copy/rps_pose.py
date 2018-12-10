#!/usr/bin/env python

import rospy
import cv2
import numpy
import yaml

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# calculation of camera's position, orientation and velocity

def rps_get_pose_msg(self,frame_id):
	
	msg = PoseStamped()
	msg.header.stamp = rospy.get_rostime()
	msg.header.frame_id = frame_id
	
	if frame_id != '':
		msg.pose.position.x = self.pose[0]
		msg.pose.position.y = self.pose[1]
		msg.pose.position.z = self.pose[2]

		msg.pose.orientation.x = self.orient[0]
		msg.pose.orientation.y = self.orient[1]
		msg.pose.orientation.z = self.orient[2]
		msg.pose.orientation.w = 1

	return msg


def rps_calculate_and_publish(self):

	# calculate pose and velocity
	if self.markers_detected > 0:
		self.fail_cnt = 0
		self.pose = -self.tvec
		self.orient = self.rvec

	else:
		if not (self.fail_cnt is None):
			self.fail_cnt += 1

			if self.fail_cnt == 5:
				self.pose = None
				self.fail_cnt = None

		else:
			self.pose = None

	# publish pose
	if not(self.pose is None):
		msg = self.get_pose_msg('main_camera')
	else:
		msg = self.get_pose_msg('') 
	
	self.pose_pub.publish(msg)

#--- detection of aruco markers ---#

def rps_detect(self,data):

	# detect
	self.raw_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
	corners, ids, rejected_objects = cv2.aruco.detectMarkers(self.raw_img, self.dict, parameters=self.param)

	# calculate rvec, tvec -> pose, orient, vel
	if not (ids is None):
		if not (self.rvec is None) and not(self.tvec is None):
			self.markers_detected, self.rvec, self.tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix,self.dist_coeffs, self.rvec, self.tvec, useExtrinsicGuess=True) 
		else:
			self.markers_detected, self.rvec, self.tvec = cv2.aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix,self.dist_coeffs)
	else:
		self.markers_detected = 0

	self.calculate_and_publish()

	# debug
	#if self.markers_detected > 0:
		#print('number = {0}, x = {1:.3f}, y = {2:.3f}, z = {3:.3f}'.format(self.markers_detected, -self.tvec[0][0], -self.tvec[1][0], -self.tvec[2][0]))
	#else:
		#print('no pose')

	# send img to /RPS/debug
	if self.img_pub.get_num_connections() > 0:
		self.img = cv2.aruco.drawDetectedMarkers(self.raw_img, corners, ids)
		if self.markers_detected > 0:
			self.img = cv2.aruco.drawAxis(self.img, self.camera_matrix, self.dist_coeffs, self.rvec, self.tvec, 0.4)
		self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, 'bgr8'))

	# image of board -> /RPS/board_image
	if not(self.board_img is None) and (self.board_img_pub.get_num_connections() > 0):
		self.board_img_pub.publish(self.bridge.cv2_to_imgmsg(self.board_img, 'mono8'))

#--- board and camera matrix ---#

def rps_calculate_board(self):

	# parameters of board
	number_x = int(rospy.get_param('~markers_number_x', '1'))
	number_y = int(rospy.get_param('~markers_number_y', '1'))
	first = int(rospy.get_param('~first_marker', '0'))
	ids = [[i] for i in range(first,first + number_x*number_y)]
	side = float(rospy.get_param('~markers_side', '0.22'))
	dist_x = float(rospy.get_param('~markers_dist_x', '0'))
	dist_y = float(rospy.get_param('~markers_dist_y', '0'))

	# actions
	obj_points = []
	corners = [[],[],[],[]]
	max_y = side + (number_y - 1)*(side + dist_y)
	for i in range(number_y):
		for j in range(number_x):
			corners[0] = [j*(side + dist_x), max_y - i*(side + dist_y), 0]
			corners[1] = [j*(side + dist_x) + side, max_y - i*(side + dist_y), 0]
			corners[2] = [j*(side + dist_x) + side, max_y - i*(side + dist_y) - side, 0]
			corners[3] = [j*(side + dist_x), max_y - i*(side + dist_y) - side, 0]
			corners = numpy.asarray(corners, dtype=numpy.float32)
			obj_points.append(corners.copy())

	ids = numpy.asarray(ids)

	return obj_points,ids

def rps_create_board(self):

	# create board
	obj_points,ids = self.calculate_board()
	self.board = cv2.aruco.Board_create(obj_points, self.dict, ids)

	# create image of board
	self.board_img = cv2.aruco.drawPlanarBoard(self.board, (1000,1000))

def rps_correct_cm(self):
	vector = self.camera_matrix['data']
	matrix = [[],[],[]]
	for i in range(3):
		for j in range(3):
			matrix[i].append(vector[3*i + j])
	self.camera_matrix = numpy.asarray(matrix, dtype=numpy.float64)

#--- main class ---#

class rps_class:

	detect = rps_detect
	create_board = rps_create_board
	calculate_board = rps_calculate_board
	correct_cm = rps_correct_cm

	get_pose_msg = rps_get_pose_msg
	calculate_and_publish = rps_calculate_and_publish

	def __init__(self):

		rospy.loginfo('initializing rps_pose...')

		self.dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		self.param = cv2.aruco.DetectorParameters_create()
		self.board = None

		self.bridge = CvBridge()
		self.raw_img = None
		self.img = None
		self.board_img = None

		self.create_board()

		cinfo_path = rospy.get_param('~cinfo_path', '~/catkin_ws/src/clever/clever/camera_info/fisheye_cam_320.yaml')
		with open(cinfo_path) as cinfo:
			load = yaml.load(cinfo)
			self.camera_matrix = load.get('camera_matrix')
			self.correct_cm()

			self.dist_coeffs = load.get('distortion_coefficients')
			self.dist_coeffs = numpy.asarray(self.dist_coeffs['data'], dtype=numpy.float64)

		self.markers_detected = 0
		self.rvec = None
		self.tvec = None
		
		self.pose = None
		self.orient = None
		self.fail_cnt = None

		self.raw_img_sub = rospy.Subscriber('/main_camera/image_raw/throttled', Image, self.detect)

		self.img_pub = rospy.Publisher('~/RPS/debug', Image, queue_size=1)
		self.board_img_pub = rospy.Publisher('~/RPS/board_image', Image, queue_size=1)

		self.pose_pub = rospy.Publisher('~/RPS/pose', PoseStamped, queue_size=1) 

		rospy.loginfo('rps_pose inited')

#--- actions ---#

rospy.init_node('rps_pose')
rps = rps_class()

while not rospy.is_shutdown():
	rospy.sleep(0.1)

