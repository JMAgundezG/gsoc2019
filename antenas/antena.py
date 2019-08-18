import sys

# Takes the V-REP Remote API
sys.path.append('../vrep_api')

import vrep

_NAMES = {
    0: "Transceiver",
    1: "Transceiver#0",
    2: "Transceiver#1",
    3: "Transceiver#2",
    4: "Transceiver#3",
    5: "Transceiver#4",
    6: "Transceiver#5",
}


class Antena:
    def __init__(self, ip='127.0.0.1', port=19997, number=0, start_sending=True):
        self.ip = ip
        self.port = port
        self.sending = start_sending
        if number not in range(7):
            raise Exception(
                "[ERROR] WRONG NUMBER, IT SHOULD BE BETWEEN 0 AND 7")
        else:
            self.number = number
            self.name = _NAMES[number]
        self.connect()
        print("CONNECTED TO", self.name, self.objectID)

    def connect(self):
        self.clientID = vrep.simxStart(self.ip, self.port, True, True, 5000, 5)

        if self.clientID == -1:
            raise Exception("[ERROR] CANNOT CONNECT TO SERVER")

        self.objectID = vrep.simxGetObjectHandle(self.clientID, self.name,
                                                 vrep.simx_opmode_oneshot_wait)

    def send_data(self, messageID, message):
        empty_buff = bytearray()
        receiverID = vrep.sim_handle_all
        res, ret_ints, _, _, _ = vrep.simxCallScriptFunction(
            self.clientID, self.name, vrep.sim_scripttype_childscript,
            '_sendData', [receiverID], [], [messageID, str(message)], empty_buff,
            vrep.simx_opmode_oneshot_wait)
        return res, ret_ints
    
    def receive_data(self, messageID):
        print vrep.simxGetPingTime(self.clientID)
        empty_buff = bytearray()
        res, _, _, ret_strings, _ = vrep.simxCallScriptFunction(
                self.clientID, self.name, vrep.sim_scripttype_childscript,
                '_receiveData', [0], [], [messageID], empty_buff,
                vrep.simx_opmode_oneshot_wait)
        print res, ret_strings
        return res, ret_strings
        
        
    def start(self):
        self.init_work()
        self.turn = 0
        while True:
            self.compute()
            self.turn += 1
            
    def init_work(self):
        pass
    
    def compute(self):
        if self.sending:
            print "SENDING TO ", _NAMES[(self.number + 1) % 6], "FROM ", self.objectID[1]
            res = self.send_data("MESSAGE", "message from " + self.name + ": " + str(self.turn))
            print res
        else:
            res, ret_strings = self.receive_data("MESSAGE")
            print "RES", res, "STRINGS", type(ret_strings), ret_strings
            if res == 0:
                pass

    def disconnect(self):
        vrep.simxFinish(self.clientID)



if __name__ == "__main__":

    print 'Program starts \n\n\n'
    number_ = 0
    if len(sys.argv) > 1:
        try:
            number_ = int(sys.argv[1])
        except:
            raise ValueError("THE VALUE SHOULD BE AN INTEGER")    
    
    if number_ != 0:
        _port = 19998
        Antena(port=_port,number=number_, start_sending= ( number_ == 0 )).start()
    else:
        Antena(number=number_, start_sending= ( number_ == 0 )).start()
    print '\n\n\nProgram ended'
