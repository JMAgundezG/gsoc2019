import sys
sys.path.append('../vrep_api')
sys.path.append('../toolkit')
import vrep
from toolkit import *


class VRepClientController:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client_id = -1
        self.console_id = -1
        self.debug = False

        self.connect()

        if not self.is_connected():
            if self.debug:
                err_print(prefix="COMPONENT CREATION",
                          message=["CANNOT CONNECT TO REMOTE API"])
            raise Exception("CANNOT CONNECT TO REMOTE API")

    def is_debug_mode(self):
        return self.debug

    def set_debug(self, mode):
        self.debug = mode

    def is_connected(self):
        return self.client_id != -1

    def connect(self, host=None, port=None):
        self.host = self.host if host == None else host
        self.port = self.port if port == None else port

        self.client_id = vrep.simxStart(self.host, self.port, True, True, 5000,
                                        5)

    def init_terminal(self):
        err_list = []
        res, self.console_id = vrep.simxAuxiliaryConsoleOpen(
            self.client_id, "CONSOLA", 4, 5, None, None, None, None,
            vrep.simx_opmode_blocking)
        if res != 0:
            err_list = parse_error(res)
        return res, err_list

    def write_on_terminal(self, mess):
        res = vrep.simxAuxiliaryConsolePrint(self.client_id, self.console_id,
                                             mess, vrep.simx_opmode_blocking)
        return res, parse_error(res)
