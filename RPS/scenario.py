#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from clever.srv import *
from std_srvs.srv import Trigger

navigate = rospy.ServiceProxy("navigate",Navigate)
land = rospy.ServiceProxy("land",Trigger)

def action():
	navigate(x=0.5, y=0.5, z=0.8, frame_id='rps_map', auto_arm=True)


