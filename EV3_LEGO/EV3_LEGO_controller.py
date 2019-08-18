# -*- coding: utf-8 -*-

import sys
sys.path.append('../vrep_api')
sys.path.append('../toolkit')
import vrep
from toolkit import *
from vrep_client_controller import VRepClientController
import numpy as np
__package__ = "ePuck"

__version__ = "1.0.1"
__author__ = "José Manuel Agúndez García"
__license__ = "GPL"


class EV3Controller(VRepClientController):

    __COMPONENTS = {
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

    def __init__(self, host, port, suffix=""):
        VRepClientController.__init__(self, host, port)
        self.suffix = suffix
        self.components = {}
        self.handle_objects()
        self.left_vel = None
        self.right_vel = None

    def handle_objects(self):
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
                    vrep.simx_opmode_streaming)
            err_list = parse_error(res)
            if res != 0 and self.debug:
                err_print(prefix="SET SPEED LEFT: ", message=err_list)
        return res, err_list

    def set_speed_right(self, speed):
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
                    vrep.simx_opmode_streaming)
                err_list = parse_error(res)
                if res != 0 and self.debug:
                    err_print(prefix="SET SPEED RIGHT: ", message=err_list)
        return res, err_list

    def get_prox_sensors(self):

        res, data = vrep.simxGetStringSignal(self.client_id, 'EPUCK_PROXSENS' + self.suffix,
                                             vrep.simx_opmode_oneshot_wait)
        err_list = parse_error(res)
        if res != 0 and self.debug:
            err_print(prefix="SET SPEED LEFT: ", message=err_list)
        data = vrep.simxUnpackFloats(data)
        self.prox_sensors = data
        return {'res': res, 'err_list': err_list, 'data': data}

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

                