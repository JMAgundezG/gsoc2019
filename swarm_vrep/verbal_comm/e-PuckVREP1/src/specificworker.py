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

import sys, os, traceback, time, math, random, imutils, cv2
import numpy as np

from PySide import QtGui, QtCore
from genericworker import *

from epuck_controller import EPuckController

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        """Constructor of the specificWorker
        
        Arguments:
            GenericWorker {[type]} -- [description]
            proxy_map {[type]} -- [description]
        """

        # This dict is a state machine
        # Key : The state name
        # Value : The state method  
        self.__STATE_MACHINE = {
            'BUG': self.bug_action,
            'EATING': self.eating_action,
            'SLEEPING': self.sleeping_action,
            'FREE': self.free_action,
            'ROTATING': self.rotating_action,
            'SEARCHING': self.searching_action
        }

        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

        # The handler of the EPuck in VREP
        self.handler = EPuckController("127.0.0.1", 20000, "#1")
        # The initial state
        self.state = 'EATING'
        
        # The objects that we need to control in the scene
        self.handler.set_new_objects(['COMEDERO'])

        # Starts with the differential robot without any movement
        self.setSpeedBase(0, 0)

        # Auxiliary count
        self.count = 0

        self.eating = False

        self.max_eating = 2

        self.max_eating_rounds = 50

        self.waiting_to_eat = False

        self.eating_count = 0
        
        self.handler.set_swarm_data("0")

        self.handler.lights_off()


    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        
        print self.getBasePose()[-1]
        print self.state
        self.__STATE_MACHINE[self.state]()
        return True


    def wait_to_eat(self):
        self.waiting_to_eat = True
        self.start_free_action()


    def sleeping_action(self):
        """
        The action of the SLEEPING state
        """
        self.stopBase()

    def free_action(self):
        """ 
        The action of the FREE state
        """
        if self.count % 10 == 0:
                self.start_searching_action()
        else:
            data = self.handler.get_prox2()
            self.count += 1
            if self.obstacled_no_petition(data):
                self.setSpeedBase(0, -.2)
            elif random.randrange(20) == 0:
                self.setSpeedBase(0, -.5)
            else:
                self.setSpeedBase(10, 0)   

    def start_bug_action(self):
        """
        Sets the bug state
        """
        self.previous_state = self.state
        self.state = 'BUG'

    def start_eating_action(self):
        """
        Sets the eating state
        """
        self.count = 0
        self.eating_count = 0
        self.state = 'EATING'
        self.eating = False
        self.waiting_to_eat = False

    def start_free_action(self):
        """
        Sets the free action
        """
        self.count = 0
        self.state = 'FREE'
        self.eating = False
        self.handler.lights_off()

    def rotating_action(self):
        data = self.handler.get_prox2()
        if self.front_occupied(data):
            self.setSpeedBase(0, -.5)
        else:
            self.state = 'BUG'
    
    def front_occupied(self, data):
        return any([v['detectionState'] for k,v in data.iteritems() if k in [2, 3, 4, 5]])
        # for i in d.values():
        #     if i['detectionState']:
        #         return True
        # return False

   
    def start_searching_action(self):
        self.previous_state = self.state
        self.state = 'SEARCHING'
        self.searching_move = 0
        self.found = []
        self.starting_angle = self.getBasePose()[-1]
        self.searching_action()

    def searching_action(self):
        
        x, y, alpha = self.getBasePose()
        
        print "SEARCHING", alpha
        print "MOVE", self.searching_move 

        # Posible position of eating zone
        angle_rot = -.3
        if self.searching_move == 0:
            self.setSpeedBase(0, angle_rot)
            if alpha < 0:
                self.searching_move += 1
        elif self.searching_move == 1:
            self.setSpeedBase(0, angle_rot)
            if alpha < -1.4:
                self.searching_move += 1
        elif self.searching_move == 2:
            self.setSpeedBase(0, angle_rot)
            if alpha > 0:
                self.searching_move += 1
        elif self.searching_move == 3:
            self.setSpeedBase(0, angle_rot)
            if alpha > 1.4:
                self.searching_move += 1
        self.found.extend(self.find())
        self.found = list(set(self.found))
        if (len(self.found) > 1):
            self.start_free_action()
        else:
            if self.searching_move == 4:
                self.start_eating_action()
        
    def find(self):
        image = self.handler.get_camera_image()['data']['image']
        
        colors = []
        if self.__any_pink(image):
            colors.append('pink')
        if self.__any_blue(image):
            colors.append('blue')
        if self.__any_purple(image):
            colors.append('purple')
        return colors
            
        return [0]


    def __any_pink(self, im):
        pink_lower=np.array([0,76.5,230],np.uint8)
        pink_upper=np.array([51,128,255],np.uint8)
        frame = imutils.resize(im, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV) 
        mask = cv2.inRange(hsv, pink_lower, pink_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return np.count_nonzero(mask) > 50
    
    
    def __any_blue(self, im):
        blue_lower=np.array([100,150,0],np.uint8)
        blue_upper=np.array([140,255,255],np.uint8)
        frame = imutils.resize(im, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV) 
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        return np.count_nonzero(mask) > 50

    def __any_purple(self, im):
        purple_lower=np.array([130,220,100],np.uint8)
        purple_upper=np.array([170,255,200],np.uint8)
        frame = imutils.resize(im, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV) 
        mask = cv2.inRange(hsv, purple_lower, purple_upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.waitKey(1)
        return np.count_nonzero(mask) > 20

    def bug_action(self):
        data = self.handler.get_prox2()
        if self.front_occupied(data):
            self.state = 'ROTATING'
        elif self.left_occupied(data):
            self.setSpeedBase(100, 0)
        else:
            self.state = self.previous_state
 

    def left_occupied(self, data):
        """Returns if the left side of the robot
        
        Arguments:
            data {[dict]} -- [self.handler.get_prox2()]
        
        Returns:
            [Boolean]
        """
        return any([v['detectionState'] for k,v in data.iteritems() if k in [2, 1, 8]])

    def obstacled(self):
        """
        Finds a obstacle using the proximity sensors
        """
        data = self.handler.get_prox2()
        data = {k: v for k,v in data.iteritems() if k < 6 and k > 1}
        for i in data.values():
			if i['detectionState']:
				return True
        return False
		
    def behind_occupied_no_petition(self, data):
        """
        Returns if the back of the robot is occupied
        """
        return any([v['detectionState'] for k,v in data.iteritems() if k in [7, 8]]) 


    def obstacled_no_petition(self, data):
        """
        Exactly as self.obstacled() but without petiton
        Arguments:
            data {[Dict]} -- [self.handler.get_prox2()]
        """
        d = {k: v for k,v in data.iteritems() if k < 6 and k > 1}
        for i in d.values():
			if i['detectionState']:
				return True
        return False

    def eating_action(self):
        """
        The actions of eating state
        """
        data = self.handler.get_prox2()
        # EATING
        if self.in_eating_zone_no_petition(data) or self.eating:
            if self.eating_count == 0:
                suff = int(self.handler.suffix[-1]) + 2 if len(self.handler.suffix) > 0 else 1
                self.handler.lights_on(suff) 
            print self.eating_count
            self.stopBase()
            self.eating_count += 1
            if self.eating_count > self.max_eating_rounds:
                self.start_free_action()
        
        # OBSTACLED
        elif self.obstacled_no_petition(data):
            print "OBSTACLED"
            self.start_bug_action()
        else:
            print self.count
            if self.count == 8:
                if self.start_searching_action():
                    self.wait_to_eat()
            else:
                print "GOING TO FEEDER"    
                x, y, alpha = self.getBasePose()
                # Posible position of eating zone
                fin_x, fin_y = x, 2
                rel_x, rel_y = 0, 2
                angle_rot = alpha
                mod = np.linalg.norm((rel_x, rel_y), ord=2)
                linear_speed = 100 * (1 / (1 + (math.e**(-mod + 3 / 2)) *
                                        (math.e**(-1.5 * angle_rot**2))))
                self.setSpeedBase(linear_speed, angle_rot)
                self.count += 1
        
    def in_eating_zone(self):
        """
        Search if you are in the objetive position (Feeder)
        """
        data = self.handler.get_prox2()
        for i in data.values():
            if i['detectionState']:
                det_obj = i['detectedObjectHandle']
                if det_obj in self.handler.objects:
                    if self.handler.objects[det_obj] == 'COMEDERO':
                        self.eating = True
                        return True
        return False

    def in_eating_zone_no_petition(self, data):
        """ 
        Exactly as self.in_eating_zone but without petitions
        
        Arguments:
        data {[Dict]} -- [self.handler.get_prox2()]
        
        Returns:
            [Boolean]
        """
        for i in data.values():
            if i['detectionState']:
                det_obj = i['detectedObjectHandle']
                if det_obj in self.handler.objects:
                    if self.handler.objects[det_obj] == 'COMEDERO':
                        self.eating = True
                        return True
        return False
    

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
    # correctOdometer
    #
    def correctOdometer(self, x, z, alpha):
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
    # stopBase
    #
    def stopBase(self):
        self.handler.stop_base()

    #
    # setSpeedBase
    #
    def setSpeedBase(self, adv, rot):
        self.handler.diff_vel(adv, rot)
    #----------------------------------------------------------------------
    #Autogenerated code, i'm not going to implement it
    #----------------------------------------------------------------------
    #
    # setOdometerPose
    #
    def setOdometerPose(self, x, z, alpha):
        #
        #implementCODE
        #
        pass


    #
    # getRGBPackedImage
    #
    def getRGBPackedImage(self, cam):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # getYRGBImage
    #
    def getYRGBImage(self, cam):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # getYLogPolarImage
    #
    def getYLogPolarImage(self, cam):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # getYUVImage
    #
    def getYUVImage(self, cam):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # getCamParams
    #
    def getCamParams(self):
        ret = TCamParams()
        #
        #implementCODE
        #
        return ret

    #
    # getYImageCR
    #
    def getYImageCR(self, cam, div):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # setInnerImage
    #
    def setInnerImage(self, roi):
        #
        #implementCODE
        #
        pass

    #
    # getYImage
    #
    def getYImage(self, cam):
        #
        #implementCODE
        #
        roi = imgType()
        hState = RoboCompCommonHead.THeadState()
        bState = RoboCompGenericBase.TBaseState()
        return [roi, hState, bState]

    #
    # getLaserData
    #
    def getLaserData(self):
        ret = TLaserData()
        #
        #implementCODE
        #
        return ret

    #
    # getLaserConfData
    #
    def getLaserConfData(self):
        ret = LaserConfData()
        #
        #implementCODE
        #
        return ret

    #
    # getLaserAndBStateData
    #
    def getLaserAndBStateData(self):
        ret = TLaserData()
        #
        #implementCODE
        #
        bState = RoboCompGenericBase.TBaseState()
        return [ret, bState]
