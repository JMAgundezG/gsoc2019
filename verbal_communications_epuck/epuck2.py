import sys

# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')

import vrep

import numpy as np
import cv2
import matplotlib.pyplot as plt
import time
ip = '158.49.227.95'
port = 19998
def how_much_red(image):
    lower_red = np.array([50,50,110]) 
    upper_red = np.array([255,255,255])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  
    mask = cv2.inRange(hsv, lower_red, upper_red) 
    res = cv2.bitwise_and(image, image, mask= mask) 
    cv2.imshow('frame',image) 
    cv2.imshow('mask',mask) 
    cv2.imshow('res',res) 
    cv2.waitKey(5)

def vec2image(vec, resolution):
    mat_image = np.array(vec, dtype=np.uint8)
    mat_image.resize([resolution[0], resolution[1], 3])
    mat_image = cv2.flip(cv2.cvtColor(mat_image, cv2.COLOR_RGB2BGR), 0)
    return cv2.GaussianBlur(mat_image, (25,25), 0)
if __name__ == "__main__":        
    print "STARTING EPUCK2"

    clientID=vrep.simxStart(ip,port,True,True,5000,5)
    if clientID!=-1:
        print 'Connected to remote API server'
        res, v0= vrep.simxGetObjectHandle(clientID,'ePuck_camera#1',vrep.simx_opmode_oneshot_wait)
        res, v1= vrep.simxGetObjectHandle(clientID,'PassiveVision_sensor',vrep.simx_opmode_oneshot_wait)
        res, motor_left = vrep.simxGetObjectHandle(clientID, 'ePuck_leftWheel#1', vrep.simx_opmode_streaming)
        res, motor_right = vrep.simxGetObjectHandle(clientID, 'ePuck_rightWheel#1', vrep.simx_opmode_streaming)
        res = vrep.simxSetJointTargetVelocity(clientID, motor_left, 0, vrep.simx_opmode_streaming)
        res = vrep.simxSetJointTargetVelocity(clientID, motor_right, 0, vrep.simx_opmode_streaming)
        first = False
        count = 0
        while (vrep.simxGetConnectionId(clientID) != -1):
            res,resolution,image=vrep.simxGetVisionSensorImage(clientID,v0,0,vrep.simx_opmode_oneshot_wait)
            if res == vrep.simx_return_ok:    
                image_converted = vec2image(image, resolution)
                if not first:
                    last_image = image_converted
                    first = True
                else:
                    if not np.array_equal(last_image, image_converted):
                        count += 1
                        last_image = image_converted
                        if count % 2 == 0:
                            print("CATCHED LED MESSAGE", count / 2)
                cv2.imshow('image', image_converted)
                cv2.waitKey(25)
                res = vrep.simxSetVisionSensorImage(clientID,v1,image,0,vrep.simx_opmode_oneshot_wait)
                
        vrep.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print 'Program ended'