# -*- coding: utf-8 -*-

import sys
import vrep
from toolkit import *
from vrep_client_controller import VRepClientController
import numpy as np
import math
__package__ = "EV3_ROBOCOMP_LEGO"

__version__ = "1.0.1"
__author__ = "José Manuel Agúndez García"
__license__ = "GPL"


class EV3Controller(VRepClientController):
    

    __COMPONENTS = {
        'robot': 'LEGO_EV3',
        'camera': 'Camara',
        'camera_bumper': 'Camara_bumper',
        'camera_sonar': 'Camara_sonar',
        'sensor_color_LR': 'Sensor_Color_LR',
        'sensor_color_RC': 'Sensor_Color_RC',
        'sonar': 'Sonar',
        'bumper': 'Bumper',
        'motor_b': 'Motor_B',
        'motor_c': 'Motor_C',
        'slider': 'Slider_SF',
        'giroscope': 'Giroscopio'
    }

    def __init__(self, host, port, suffix="", debug=False):
        """
        EV3Controller constructor

        Arguments:
            host {[string]} -- ip host
            port {[int]} -- VREP port
        
        Keyword Arguments:
            suffix {str} -- [Suffix for multiple robots] (default: {""})
            debug {bool} -- [Debug messages activation] (default: {False})
        """
        VRepClientController.__init__(self, host, port)
        self.suffix = suffix
        self.debug = debug
        self.components = {}
        self.handle_objects()
        self.left_vel = None
        self.right_vel = None
        self.position = [None, None, None]
        self.dist_between_wheels = 1.0
        self.radius = 0.5

    def handle_objects(self):
        """
        Connects each component to the simulator
        """
        for i, j in EV3Controller.__COMPONENTS.iteritems():
            self.components[i] = {'name': j + self.suffix, 'id': None}

        for i in self.components.keys():
            res, comp_id = vrep.simxGetObjectHandle(
                self.client_id, self.components[i]['name'],
                vrep.simx_opmode_oneshot_wait)
            if res == 0:
                self.components[i]['id'] = comp_id
            elif res != 0 and self.debug:
                err_print(prefix="HANDLE OBJECTS:" +
                          self.components[i]['name'] + " ",
                          message=parse_error(res))


    def set_speed_left(self, speed):
        """Sets the speed of the left motor
        
        Arguments:
            speed {[int]} -- [Linear speed of the left motor, it should in [-1000, 1000]]
        
        Returns:
            res[int] -- [result of the vrep library method]
            err_list[list[string]] -- list of errors
        """
        if speed > 1000:
            speed = 1000
        elif speed < -1000:
            speed = -1000
        res = 0
        err_list = []
        if self.left_vel != speed:
            res = -1
            if self.components['motor_c']['id'] != None:
                res = vrep.simxSetJointTargetVelocity(
                    self.client_id, self.components['motor_c']['id'], speed,
                    vrep.simx_opmode_buffer)
            err_list = parse_error(res)
            if res != 0 and self.debug:
                err_print(prefix="SET SPEED LEFT: ", message=err_list)
        return res, err_list

    def set_speed_right(self, speed):
        """Sets the speed of the right motor
        
        Arguments:
            speed {[int]} -- [Linear speed of the right motor, it should in [-1000, 1000]]
        
        Returns:
            res[int] -- [result of the vrep library method]
            err_list[list[string]] -- list of errors
        """
        if speed > 1000:
            speed = 1000
        elif speed < -1000:
            speed = -1000
        err_list = []
        res = 0
        if self.right_vel != speed:
            res = -1
            if self.components['motor_b']['id'] != None:
                res = vrep.simxSetJointTargetVelocity(
                    self.client_id, self.components['motor_b']['id'], speed,
                    vrep.simx_opmode_blocking)
                err_list = parse_error(res)
                if res != 0 and self.debug:
                    err_print(prefix="SET SPEED RIGHT: ", message=err_list)
        return res, err_list


    def get_light_sensors(self):
        """ Light sensors data getter
        
        Returns:
            [list] -- [list of light sensors data]
        """
        res_list = []
        for i in list(filter(lambda x: 'sensor_color' in x[0] ,self.components.items())):
            res, _, prox = vrep.simxReadVisionSensor(self.client_id, i[1]['id'], vrep.simx_opmode_blocking)
            if res != vrep.simx_return_ok:
                err_print(prefix="GET LIGHT SENSORS: ", message=err_list)
            res_list.append(prox[0][11:14])
        return res_list

    def get_camera_image(self):
        data = {'res' : -1,
                'err_list': ["camera isn't connected to client"],
                'data': {'image': np.zeros((128, 128, 3)), 'resolution': (128, 128, 3)}}
        if self.components['camera']['id'] != None:
            res, resolution, image = vrep.simxGetVisionSensorImage(self.client_id, self.components['camera']['id'], 0, vrep.simx_opmode_streaming)
            data['res'] = res
            data['err_list'] = err_list = parse_error(res)
            print resolution
            if res not in (0, 1) and self.debug:
                err_print(prefix='CAMERA', message=err_list)
            elif len(resolution) > 1:
                resolution = (resolution[0], resolution[1], 3)
                data['data']['resolution'] = resolution 
                mat_image = np.array(image, dtype=np.uint8)
                mat_image.resize([resolution[0], resolution[1], 3])

                data['data']['image'] = mat_image[::-1]
        return data 

                
    def get_base_pose(self):
        res, pos = vrep.simxGetObjectPosition(self.client_id, self.components['robot']['id'], -1, vrep.simx_opmode_blocking)
        if res != 0:
            err_print("GET BASE POSE", parse_error(res))
            raise Exception("ERROR IN GET BASE POSE")
        else:
            res, ang = vrep.simxGetObjectOrientation(self.client_id, self.components['robot']['id'], -1, vrep.simx_opmode_blocking)
            if res != 0:
                err_print("GET BASE POSE", parse_error(res))
                raise Exception("ERROR IN GET BASE POSE")
            else: 
                ang = ang[-1]
                return pos[0], pos[1], ang

    def get_base_pose_full(self):
        res, pos = vrep.simxGetObjectPosition(self.client_id, self.components['robot']['id'], -1, vrep.simx_opmode_blocking)
        if res != 0:
            err_print("GET BASE POSE", parse_error(res))
            raise Exception("ERROR IN GET BASE POSE")
        else:
            res, ang = vrep.simxGetObjectOrientation(self.client_id, self.components['robot']['id'], -1, vrep.simx_opmode_blocking)
            if res != 0:
                err_print("GET BASE POSE", parse_error(res))
                raise Exception("ERROR IN GET BASE POSE")
            else: 
                ang = ang[-1]
                return pos, ang
    

    def move_robot(self, x, z, alpha):
        y = self.get_base_pose_full()[0][2]
        res = vrep.simxSetObjectPosition(self.client_id, self.components['robot']['id'], -1, (x, y, z), vrep.simx_opmode_blocking)
        if res != 1:
            err_print("MOVE ROBOT SET POSITION", parse_error(res))
        
        # res = vrep.simxSetObjectOrientation(self.client_id, self.components['robot']['id'], -1, (alpha, 0, 0), vrep.simx_opmode_oneshot)
        # if res != 1:
        #     err_print("MOVE ROBOT SET ORIENTATION", parse_error(res))

    def set_global_speed(self, adv, rot):
        """set the robot speed using an advance and rotation speed
        
        Arguments:
            adv {[type]} -- [description]
            rot {[type]} -- [description]
        """

        # control of the input variables
        if adv > 100:
            adv = 100
        elif adv < -100:
            adv = -100
        if rot > math.pi:
            rot = math.pi
        elif rot < -math.pi:
            rot = -math.pi

        spd_left = (adv - (self.dist_between_wheels / 2) * rot) / self.radius
        spd_right = (adv + (self.dist_between_wheels / 2) * rot) / self.radius
        
        print("SPEED:", spd_left, spd_right)
        self.set_speed_left(spd_left)
        self.set_speed_right(spd_right)

    
        

        

