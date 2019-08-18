import sys

# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')

import vrep

if __name__ == "__main__":

    print 'Program started'
    # just in case, close all opened connections
    vrep.simxFinish(-1)

    # Starts a connection with the API, returns an ID
    clientID = vrep.simxStart('127.0.0.1', 19998, True, True, 5000, 5)
    if clientID != -1:
        print 'Connected to remote API server'
        # starts a console in V-REP
        console = vrep.simxAuxiliaryConsoleOpen(clientID, "CONSOLA", 4, 5,
                                                None, None, None, None,
                                                vrep.simx_opmode_blocking)

        # prints a hello world in the console
        print(vrep.simxAuxiliaryConsolePrint(clientID, console[1],
                                             "HELL0 W0RLD",
                                             vrep.simx_opmode_blocking))
        vrep.simxFinish(clientID)
    else:
        print 'Failed connecting to remote API server'
    print 'Program ended'
