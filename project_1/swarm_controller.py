import sys
import config
# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')

import vrep


class SwarmController:
    
    def __init__(self, ip='127.0.0.1', port=19998, console=False, mode=None):
        self.ip = ip
        self.port = port
        self.connect(ip, port)
        self.console = None if console == False else self.take_terminal()
        self.mode = vrep.simx_opmode_blocking if mode == None else mode

    def connect(self, ip, port):
        self.clientID = vrep.simxStart(ip, port, True, True, 5000, 5)
        if self.clientID == -1:
            raise Exception("[ERROR] THE CONTROLLER CANNOT CONNECT")
        else:
            # Saving all possible sensors

    def disconnect(self):
        vrep.simxFinish(self.clientID)

    def open_console(self):
        if self.console is None:
            self.console = vrep.simxAuxiliaryConsoleOpen(
                self.clientID, "CONSOLA", 4, 5, None, None, None, None,
                vrep.simx_opmode_blocking)
        else:
            print("[ERROR] CONSOLE ALREADY OPENED")

    def print_on_console(self, message):
        if self.console is not None:
            vrep.simxAuxiliaryConsolePrint(self.clientID, self.console[1],
                                           message, vrep.simx_opmode_blocking)
        else:
            print("[ERROR] THERE ISN'T ANY CONSOLES")



if __name__ == "__main__":
    swarm = SwarmController(ip=config.ip, port=config.port)
