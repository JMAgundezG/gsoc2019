# -*- coding: utf-8 -*-

import sys
import vrep
from toolkit import *
from vrep_client_controller import VRepClientController
import numpy as np
__package__ = "ePuck"

__version__ = "1.0.1"
__author__ = "José Manuel Agúndez García"
__license__ = "GPL"


class EPuckController(VRepClientController):

    __COMPONENTS = {
        'robot': 'ePuck',
        'base': 'ePuck_base',
        'ledLight': 'ePuck_ledLight',
        'leftJoint': 'ePuck_leftJoint',
        'leftWheel': 'ePuck_leftWheel',
        'leftWheelPart': 'ePuck_leftWheelPart',
        'rightJoint': 'ePuck_rightJoint',
        'rightWheel': 'ePuck_rightWheel',
        'rightWheelPart': 'ePuck_rightWheelPart',
        'camera': 'ePuck_camera',
        'lightSensor': 'ePuck_lightSensor',
        'proxSensor1': 'ePuck_proxSensor1',
        'proxSensor2': 'ePuck_proxSensor2',
        'proxSensor3': 'ePuck_proxSensor3',
        'proxSensor4': 'ePuck_proxSensor4',
        'proxSensor5': 'ePuck_proxSensor5',
        'proxSensor6': 'ePuck_proxSensor6',
        'proxSensor7': 'ePuck_proxSensor7',
        'proxSensor8': 'ePuck_proxSensor8',
        'speaker': 'ePuck_speaker'
    }

    def __init__(self, host, port, suffix=""):
        VRepClientController.__init__(self, host, port)
        self.suffix = suffix
        self.components = {}
        self.handle_objects()
        self.left_vel = 0
        self.right_vel = 0
        self.led_lights = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0}
        self.prox_sensors = []
        self.robot_wheel_radius = 20.5
        self.robot_wheel_base_length = 52
        self.objects = {}

    def handle_objects(self):
        for i, j in EPuckController.__COMPONENTS.iteritems():
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

    def set_new_objects(self, objects):
        for i in objects:
            res, comp_id = vrep.simxGetObjectHandle(
                self.client_id, i, vrep.simx_opmode_oneshot_wait)
            if res == 0:
                self.objects[comp_id] = i
    
    def stop_base(self):
        err_list = []
        self.left_vel = -9999
        self.right_vel = -9999
        self.set_speed_left(0)
        self.set_speed_right(0) 
                
    def set_speed_left(self, speed):
        if speed > 179.9:
            speed = 179.9
        elif speed < -179.9:
            speed = -179.9
        res = 0
        err_list = []
        if self.left_vel != speed:
            res = -1
            if self.components['leftJoint']['id'] != None:
                res = vrep.simxSetJointTargetVelocity(
                    self.client_id, self.components['leftJoint']['id'], speed,
                    vrep.simx_opmode_streaming)
            err_list = parse_error(res)
            if res != 0:
                if self.debug:
                    err_print(prefix="SET SPEED LEFT: ", message=err_list)
            else:
                self.left_vel = speed
        return res, err_list

    def set_speed_right(self, speed):
        if speed > 179.9:
            speed = 179.9
        elif speed < -179.9:
            speed = -179.9
        err_list = []
        res = 0
        if self.right_vel != speed:
            res = -1
            if self.components['rightJoint']['id'] != None:
                res = vrep.simxSetJointTargetVelocity(
                    self.client_id, self.components['rightJoint']['id'], speed,
                    vrep.simx_opmode_streaming)
                err_list = parse_error(res)
                if res != 0: 
                    if self.debug:
                        err_print(prefix="SET SPEED RIGHT: ", message=err_list)
                else:
                    self.vel_right = speed
        return res, err_list

    def set_led_light(self, led_number, intensity=0):
        if led_number > 7 or led_number < 0:
            if self.debug:
                err_print(prefix="SET LED LIGHT",
                          message=["LED NUMBER SHOULD BE BETWEEN 0 AND 7"])
        elif intensity not in (0, 1):
            pass
        else:
            self.led_lights[led_number] = intensity
            pack = vrep.simxPackInts([led_number, intensity])
            vrep.simxSetStringSignal(self.client_id, 'EPUCK_LED' + self.suffix,
                                     pack, vrep.simx_opmode_oneshot_wait)

    def get_prox_sensors(self):

        res, data = vrep.simxGetStringSignal(self.client_id,
                                             'EPUCK_PROXSENS' + self.suffix,
                                             vrep.simx_opmode_oneshot_wait)
        err_list = parse_error(res)
        if res != 0 and self.debug:
            err_print(prefix="SET SPEED LEFT: ", message=err_list)
        data = vrep.simxUnpackFloats(data)
        self.prox_sensors = data
        return {'res': res, 'err_list': err_list, 'data': data}

    def get_prox2(self):
        data = {}
        for k, v in enumerate(
                list(
                    filter(lambda x: 'proxSensor' in x[0],
                           self.components.items())), 1):
            res, detectionState, detectedPoint, detectedObjectHandle, _ = vrep.simxReadProximitySensor(
                self.client_id, self.components['proxSensor' + str(k)]['id'],
                vrep.simx_opmode_oneshot_wait)

            ind_data = {
                'res': res,
                'detectionState': detectionState,
                'detectedPoint': detectedPoint,
                'detectedObjectHandle': detectedObjectHandle
            }
            data[k - 1] = ind_data
        return data

    def get_camera_image(self):
        data = {
            'res': -1,
            'err_list': ["camera isn't connected to client"],
            'data': {
                'image': np.zeros((128, 128, 3)),
                'resolution': (128, 128, 3)
            }
        }
        if self.components['camera']['id'] != None:
            res, resolution, image = vrep.simxGetVisionSensorImage(
                self.client_id, self.components['camera']['id'], 0,
                vrep.simx_opmode_streaming)
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

    def get_light_sensors(self):
        """ Light sensors data getter
            Returns:
            [list] -- [list of light sensors data]
        """
        res_list = []
        for i in list(
                filter(lambda x: 'ePuck_lightSensor' in x[0],
                       self.components.items())):
            res, _, prox = vrep.simxReadVisionSensor(self.client_id,
                                                     i[1]['id'],
                                                     vrep.simx_opmode_blocking)
            if res != vrep.simx_return_ok:
                err_print(prefix="GET LIGHT SENSORS: ", message=err_list)
                res_list.append(prox[0][11:14])
        return res_list

    def _to_differential_vel(self, vel, omega):

        R = self.robot_wheel_radius
        L = self.robot_wheel_base_length

        vel_left = ((2.0 * vel) - (omega * L)) / (2.0 * R)
        vel_right = ((2.0 * vel) + (omega * L)) / (2.0 * R)

        return vel_left, vel_right

    def diff_vel(self, v, omega):
        vel_left, vel_right = self._to_differential_vel(v, omega)
        max_ = max([vel_left, vel_right, 179.9])
        vel_left = (vel_left / max_) * 179.9
        vel_right = (vel_right / max_) * 179.9
        print "VEL_LEFT: ", vel_left
        print "VEL_RIGHT: ", vel_right
        self.set_speed_left(vel_left)
        self.set_speed_right(vel_right)

    def get_base_pose(self):
        res, pos = vrep.simxGetObjectPosition(self.client_id,
                                              self.components['robot']['id'],
                                              -1, vrep.simx_opmode_blocking)
        if res != 0:
            err_print("GET BASE POSE", parse_error(res))
            raise Exception("ERROR IN GET BASE POSE")
        else:
            res, ang = vrep.simxGetObjectOrientation(
                self.client_id, self.components['robot']['id'], -1,
                vrep.simx_opmode_blocking)
            if res != 0:
                err_print("GET BASE POSE", parse_error(res))
                raise Exception("ERROR IN GET BASE POSE")
            else:
                ang = ang[1]
                return pos[0], pos[1], ang

    def set_base_orientation(self, omega):
        res, ang = vrep.simxGetObjectOrientation(
                self.client_id, self.components['robot']['id'], -1,
                vrep.simx_opmode_blocking)
        if res != 0:
            err_print("GET BASE POSE", parse_error(res))
            raise Exception("ERROR IN GET BASE POSE")
        print "ANG", ang
        ang[1] = omega
        res, pos = vrep.simxSetObjectOrientation(self.client_id,
                                              self.components['robot']['id'],
                                              -1, ang, vrep.simx_opmode_blocking)
        if res != 0:
            err_print("SET BASE ORI", parse_error(res))
            raise Exception("ERROR IN SET BASE ORIENTATION")
       
    def get_swarm_data(self):
        signal_name = "signal"
        res, signal_val = vrep.simxGetStringSignal(self.client_id, signal_name, vrep.simx_opmode_blocking)
        if res <= 8:
            return signal_val if signal_val != '' else '0'
        else:
            err_print("GET SWARM DATA", parse_error(res))
            raise Exception("ERROR IN GET SWARM DATA")
    

    def set_swarm_data(self, val):
        signal_name = "signal"

        res = vrep.simxSetStringSignal(self.client_id, signal_name, val, vrep.simx_opmode_blocking)
        if res != 0:
            err_print("SET WARM DATA", parse_error(res))
            raise Exception("ERROR IN SET WARM DATA")
    
    def take_position_eating(self, take):
        val = self.get_swarm_data()
        self.set_swarm_data(val, take)

    