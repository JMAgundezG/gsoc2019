import sys

# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')

import vrep

if __name__ == "__main__":

    print 'Program started'

    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        empty_buff = bytearray()
        print clientID
        antenna = vrep.simxGetObjectHandle(clientID, "Transceiver_antenna",
                                           vrep.simx_opmode_oneshot_wait)
        while True:
            message = 'Hola desde antena 1'
            messageName = "MyMessage"
            
            # res, retInts, target1Pose, retStrings, retBuffer = vrep.simxCallScriptFunction(
            #     clientID, 'Transceiver', vrep.sim_scripttype_childscript,
            #     'sendData', [vrep.sim_handle_all], [], ['myMessage', message],
            #     empty_buff, vrep.simx_opmode_streaming)

            # res, retInts, target1Pose, retStrings, retBuffer = vrep.simxCallScriptFunction(
            #     clientID, 'Transceiver', vrep.sim_scripttype_customizationscript,
            #     'getObjectPose', [], [], [],
            #     empty_buff, vrep.simx_opmode_streaming)
            res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,
             'Transceiver',vrep.sim_scripttype_childscript,'sendData',[0],[],
             [messageName, str(message)],empty_buff,vrep.simx_opmode_oneshot_wait)
            print "SEND: ", res, retInts, target1Pose, retStrings, retBuffer, empty_buff
            auxiliaryConsoleHandle = -1
            res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,
             'Transceiver',vrep.sim_scripttype_childscript,'receiveData',[0],[],
             [messageName],empty_buff,vrep.simx_opmode_oneshot_wait)
            print "RECEIVE: ", res, retInts, target1Pose, retStrings, retBuffer, empty_buff
    else:
        print 'Failed connecting to remote API server'
    print 'Program ended'
