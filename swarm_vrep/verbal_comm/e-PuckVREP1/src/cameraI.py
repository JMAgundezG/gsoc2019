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

import sys, os, Ice

ROBOCOMP = ''
try:
	ROBOCOMP = os.environ['ROBOCOMP']
except:
	print '$ROBOCOMP environment variable not set, using the default value /opt/robocomp'
	ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
	print 'ROBOCOMP environment variable not set! Exiting.'
	sys.exit()

additionalPathStr = ''
icePaths = []
try:
	icePaths.append('/opt/robocomp/interfaces')
	SLICE_PATH = os.environ['SLICE_PATH'].split(':')
	for p in SLICE_PATH:
		icePaths.append(p)
		additionalPathStr += ' -I' + p + ' '
except:
	print 'SLICE_PATH environment variable was not exported. Using only the default paths'
	pass

ice_DifferentialRobot = False
for p in icePaths:
	print 'Trying', p, 'to load DifferentialRobot.ice'
	if os.path.isfile(p+'/DifferentialRobot.ice'):
		print 'Using', p, 'to load DifferentialRobot.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"DifferentialRobot.ice"
		Ice.loadSlice(wholeStr)
		ice_DifferentialRobot = True
		break
if not ice_DifferentialRobot:
	print 'Couldn\'t load DifferentialRobot'
	sys.exit(-1)
from RoboCompDifferentialRobot import *
ice_Laser = False
for p in icePaths:
	print 'Trying', p, 'to load Laser.ice'
	if os.path.isfile(p+'/Laser.ice'):
		print 'Using', p, 'to load Laser.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"Laser.ice"
		Ice.loadSlice(wholeStr)
		ice_Laser = True
		break
if not ice_Laser:
	print 'Couldn\'t load Laser'
	sys.exit(-1)
from RoboCompLaser import *
ice_Camera = False
for p in icePaths:
	print 'Trying', p, 'to load Camera.ice'
	if os.path.isfile(p+'/Camera.ice'):
		print 'Using', p, 'to load Camera.ice'
		preStr = "-I/opt/robocomp/interfaces/ -I"+ROBOCOMP+"/interfaces/ " + additionalPathStr + " --all "+p+'/'
		wholeStr = preStr+"Camera.ice"
		Ice.loadSlice(wholeStr)
		ice_Camera = True
		break
if not ice_Camera:
	print 'Couldn\'t load Camera'
	sys.exit(-1)
from RoboCompCamera import *

class CameraI(Camera):
	def __init__(self, worker):
		self.worker = worker

	def getRGBPackedImage(self, cam, c):
		return self.worker.getRGBPackedImage(cam)
	def getYRGBImage(self, cam, c):
		return self.worker.getYRGBImage(cam)
	def getYLogPolarImage(self, cam, c):
		return self.worker.getYLogPolarImage(cam)
	def getYUVImage(self, cam, c):
		return self.worker.getYUVImage(cam)
	def getCamParams(self, c):
		return self.worker.getCamParams()
	def getYImageCR(self, cam, div, c):
		return self.worker.getYImageCR(cam, div)
	def setInnerImage(self, roi, c):
		return self.worker.setInnerImage(roi)
	def getYImage(self, cam, c):
		return self.worker.getYImage(cam)
