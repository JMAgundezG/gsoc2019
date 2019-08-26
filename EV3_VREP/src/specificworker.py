# -*- coding: utf-8 -*-
#
# Copyright (C) 2019 by JOSÉ MANUEL AGÚNDEZ GARCÍA
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time

from PySide import QtGui, QtCore
from genericworker import *
from EV3_LEGO_controller import EV3Controller
# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

import math

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.handler = EV3Controller("158.49.227.95", 19998, debug=True)
		self.timer.timeout.connect(self.compute)
		self.Period = 2000
		self.timer.start(self.Period)

	def setParams(self, params):
		#try:
		#	self.innermodel = InnerModel(params["InnerModelPath"])
		#except:
		#	traceback.print_exc()
		#	print "Error reading config params"
		return True

	@QtCore.Slot()
	def compute(self):
		print 'SpecificWorker.compute...'
		self.setSpeedBase(0, math.pi)
		return True


	#
	# correctOdometer
	#
	def correctOdometer(self, x, z, alpha):
		"""Reallocate the robot in the map
		
		Arguments:
			x {[int]} -- [X axis position of the robot]
			z {[int]} -- [Z axis position of the robot]
			alpha {[int]} -- [Angle of orientation]
		"""
		self.handler.move_robot(x, z, alpha)
	


	#
	# getBasePose
	#
	def getBasePose(self):
		x, z, alpha = self.handler.get_base_pose()
		return [x, z, alpha]

	#
	# resetOdometer
	#
	def resetOdometer(self):
		
		pass


	#
	# setOdometer
	#
	def setOdometer(self, state):
		#
		#implementCODE
		#
		pass


	#
	# getBaseState
	#
	def getBaseState(self):
		#
		#implementCODE
		#
		state = RoboCompGenericBase.TBaseState()
		return state


	#
	# setOdometerPose
	#
	def setOdometerPose(self, x, z, alpha):
		#
		#implementCODE
		#
		pass


	#
	# stopBase
	#
	def stopBase(self):
		self.handler.set_speed_left(0)
		self.handler.set_speed_right(0)
		pass


	#
	# setSpeedBase
	#
	def setSpeedBase(self, adv, rot):
		self.handler.set_global_speed(adv, rot)
		
	
	def set_speed_left(self, vel):
		self.handler.set_speed_left(vel)
	
	def set_speed_right(self, vel):
		self.handler.set_speed_right(vel)

	def get_light_sensors(self):
		return self.handler.get_light_sensors()
