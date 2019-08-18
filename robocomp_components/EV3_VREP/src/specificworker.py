#
# Copyright (C) 2019 by YOUR NAME HERE
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

class SpecificWorker(GenericWorker):
	def __init__(self, proxy_map):
		super(SpecificWorker, self).__init__(proxy_map)
		self.handler = EV3Controller("158.49.227.10", 19998)
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
		print(self.getBasePose())
		#computeCODE
		#try:
		#	self.differentialrobot_proxy.setSpeedBase(100, 0)
		#except Ice.Exception, e:
		#	traceback.print_exc()
		#	print e

		# The API of python-innermodel is not exactly the same as the C++ version
		# self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
		# z = librobocomp_qmat.QVec(3,0)
		# r = self.innermodel.transform("rgbd", z, "laser")
		# r.printvector("d")
		# print r[0], r[1], r[2]

		return True


	#
	# correctOdometer
	#
	def correctOdometer(self, x, z, alpha):
		#
		#implementCODE
		#
		pass


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
		#
		#implementCODE
		#
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
		#
		#implementCODE
		#
		pass


	#
	# setSpeedBase
	#
	def setSpeedBase(self, adv, rot):
		#
		#implementCODE
		#
		pass

