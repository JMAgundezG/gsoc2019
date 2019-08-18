import sys

# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')

import vrep

import numpy as np
import cv2


def a(image, resolution):
    image = np.array(image)
    new_mat = np.zeros(resolution[0] * resolution[1])
    r = image[::3]
    g = image[1::3]
    b = image[2::3]
    for i in range(len(r)):
        new_mat[i] = r[i] * 0.731 + g[i] * 0.317 + b[i] * 0.136
    return np.reshape(new_mat, resolution)

if __name__ == "__main__":    

    print 'Program started'
    vrep.simxFinish(-1) # just in case, close all opened connections
    ip = "158.49.227.95"

    clientID=vrep.simxStart(ip,19997,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
        res, v0= vrep.simxGetObjectHandle(clientID,'ePuck_camera',vrep.simx_opmode_oneshot_wait)
        res, v1= vrep.simxGetObjectHandle(clientID,'PassiveVision_sensor',vrep.simx_opmode_oneshot_wait)
        res, l0 = vrep.simxGetObjectHandle(clientID,'ePuck_proxSensor4', vrep.simx_opmode_oneshot_wait)
        res, s0 = vrep.simxGetObjectHandle(clientID, 'Transceiver_antenna', vrep.simx_opmode_streaming)
        res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_streaming)
        while (vrep.simxGetConnectionId(clientID) != -1):
            #print vrep.simxGetObjectIntParameter(clientID, s0, )
            res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_streaming)
            if res == vrep.simx_return_ok:
                res = vrep.simxSetVisionSensorImage(clientID,v1,image,0,vrep.simx_opmode_oneshot)
        vrep.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print 'Program ended'
