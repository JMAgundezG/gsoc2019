#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#         ePuck.py
#
#         Copyright 2010 Manuel Martín Ortiz <manuel.martin@itrblabs.eu>
#                   2014 Martin Knopp <Martin.Knopp@tum.de>
#
#         This program is free software; you can redistribute it and/or modify
#         it under the terms of the GNU General Public License as published by
#         the Free Software Foundation; either version 3 of the License, or
#         (at your option) any later version.
#
#         This program is distributed in the hope that it will be useful,
#         but WITHOUT ANY WARRANTY; without even the implied warranty of
#         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#         GNU General Public License for more details.
#
#         You should have received a copy of the GNU General Public License
#         along with this program; if not, write to the Free Software
#         Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#         MA 02110-1301, USA.
#
#         -- ePuck.py --
#
#         The aim of this library is to provide an compatible interface on the
#         V-REP simulator. Thus, you can write a program that runs via bluetooth,
#         on the Overo extension and in the simulator without big changes.
#
#         This library is written in Python 2.6, and you can import it from
#         any program written in Python  (same version or later).

#         MODIFIED BY JMAGUNDEZG



import sys  # System library

sys.path.append('/home/robocomp/Development/GSoC/project_1/vrep_api')

import serial  # Used for communications
import time  # Used for image capture process
import struct  # Used for Big-Endian messages
from PIL import Image          # Used for the pictures of the camera
#import i2c # Ground sensors are directly connected to the overo's i2c bus
import vrep # Interface to the V-REP simulator
import math
import ctypes

__package__ = "ePuck"
__docformat__ = "restructuredtext"

__version__ = "1.2.2+vrep-1"
__author__ = "Martin Knopp"
__license__ = "GPL"
__company__ = "Technische Universität München"
__contact__ = ["Martin.Knopp@tum.de"]

# This dictionary have as keys the first character of the message, that
# is used to know the number of lines. If no key for the message, 1 line is assumed
DIC_MSG = {
    "v": 2,  # Version
    "\n": 23,  # Menu
    "\x0c": 2,  # Welcome
    "k": 3,  # Calibration
    "R": 2  # Reset
}

# You have to use the keys of this dictionary for indicate on "enable" function
# the sensor that you want to read
DIC_SENSORS = {
    "accelerometer" : "a",
    "selector" : "c",
    "motor_speed" : "e",
    "camera" : "i",
    "floor"  : "m",
    "proximity" : "n",
    "light" : "o",
    "motor_position" : "q",
    "microphone" : "u"
}

# You have to use the keys of this dictionary for indicate the operating
# mode of the camera
CAM_MODE = {
    "GREY_SCALE"    : 0,
    "RGB_365"        : 1,
    "YUV"            : 2,
    "LINEAR_CAM"    : 3
    }

# You can use three diferents Zoom in the camera
CAM_ZOOM = (1, 4, 8)

class ePuck():
    """
    This class represent an ePuck object
    """

    def __init__(self, host='localhost', port=19999, debug=False):
        """
        Constructor process
        
        :param    host: hostname or ip address of the V-REP remote server
        :type     host: string
        :param    port: port of the V-REP remote server,
        :type     port: int
        :param    debug: If you want more verbose information, useful for debugging
        :type     debug: Boolean

        :return: ePuck object
        """

        # Monitoring Variables
        self.messages_sent = 0
        self.messages_received = 0
        self.version = __version__
        self.debug = debug

        # Connection Attributes
        self._clientID = -1
        self._hostname = host
        self._port = port

        # Camera attributes
        self._cam_width = None
        self._cam_height = None
        self._cam_enable = False
        self._cam_zoom = None
        self._cam_mode = None
        self._cam_size = None

        # Sensors and actuators lists
        self._sensors_to_read = []
        self._actuators_to_write = []
        
        # Sensors
        self._accelerometer = (0, 0, 0)
        self._accelerometer_filtered = False
        self._selector = (0)
        self._motor_speed = (0, 0)  # left and right motor
        self._motor_position = (0, 0)  # left and right motor
        self._camera_parameters = (0, 0, 0, 0)
        self._floor_sensors = (0, 0, 0)
        self._proximity = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        self._light_sensor = (0, 0, 0, 0, 0, 0, 0, 0)
        self._microphone = (0, 0, 0)
        self._pil_image = None

        # Leds
        self._leds_status = [False] * 10

    #
    # Private methods
    #
    def _debug(self, *txt):
        """
        Show debug information and data, only works if debug information
        is enable (see "set_debug()")
        
        :param     txt: Data to be showed separated by comma
        :type    txt: Any
        """
        if self.debug:
            print >> sys.stderr, '\033[31m[ePuck]:\033[0m ', ' '.join([str(e) for e in txt])

        return 0

    def _read_image(self):
        """
        Returns an image obtained from the robot's camera. For communication
        issues you only can get 1 image per second
        
        :return: The image in PIL format
        :rtype: PIL Image
        """

        # Thanks to http://www.dailyenigma.org/e-puck-cam.shtml for
        # the code for get the image from the camera
        msg = struct.pack(">bb", -ord("I"), 0)

        try:
            n = self._send(msg)
            self._debug("Reading Image: sending " + repr(msg) + " and " + str(n) + " bytes")

            # We have to add 3 to the size, because with the image we
            # get "mode", "width" and "height"
            size = self._cam_size + 3
            img = self._recv(size)
            while len(img) != size:
                img += self._recv(size)

            # Create the PIL Image
            image = Image.frombuffer("RGB", (self._cam_width, self._cam_height),
                                     img, "raw",
                                     "BGR;16", 0, 1)

            image = image.rotate(180)
            self._pil_image = image

        except Exception, e:
            self._debug('Problem receiving an image: ', e)

    def _refresh_camera_parameters(self):
        """
        Method for refresh the camera parameters, it's called for some
        private methods
        """
        try:
            msg = self.send_and_receive("I").split(',')
        except:
            return False
        else:
            self._cam_mode, \
            self._cam_width, \
            self._cam_height, \
            self._cam_zoom, \
            self._cam_size = [int(i) for i in msg[1:6]]

            self._camera_parameters = self._cam_mode, self._cam_width, self._cam_height, self._cam_zoom

    def _write_actuators(self):
        """
        Write in the robot the actuators values. Don't use directly,
        instead use 'step()'
        """

        # We make a copy of the actuators list
        actuators = self._actuators_to_write[:]

        for m in actuators:
            if m[0] == 'L':
                # Leds
                # Sent as packed string to ensure update of the wanted LED
                test = vrep.simxPackInts([m[1]+1, m[2]])
                vrep.simxSetStringSignal(self._clientID, 'EPUCK_LED', test, vrep.simx_opmode_oneshot_wait)

            elif m[0] == 'D':
                # Set motor speed
                # maxVel = 120.0 * math.pi / 180.0
                # maxVel of ePuck firmware is 1000
                # => 120 * pi / (180*1000) = pi/1500
                velLeft = m[1] * math.pi / 1500.0
                velRight = m[2] * math.pi / 1500.0
                if velLeft > 120.0 * math.pi / 180.0:
                    velLeft = 120.0 * math.pi / 180.0
                    self._debug("velLeft too high, thresholded")
                if velLeft < -120.0 * math.pi / 180.0:
                    velLeft = -120.0 * math.pi / 180.0
                    self._debug("velLeft too high, thresholded")
                if velRight > 120.0 * math.pi / 180.0:
                    velRight = 120.0 * math.pi / 180.0
                    self._debug("velRight too high, thresholded")
                if velRight < -120.0 * math.pi / 180.0:
                    velRight = -120.0 * math.pi / 180.0
                    self._debug("velRRight too high, thresholded")
                vrep.simxSetFloatSignal(self._clientID, 'EPUCK_velLeft', velLeft, vrep.simx_opmode_oneshot)
                vrep.simxSetFloatSignal(self._clientID, 'EPUCK_velRight', velRight, vrep.simx_opmode_oneshot)
                
            elif m[0] == 'P':
                # Motor position
                self._debug('WARNING: Motor position not yet implemented!')
                
            else:
                self._debug('Unknown actuator to write')

            self._actuators_to_write.remove(m)
        return

    def _read_sensors(self):
        """
        This method is used for read the ePuck's sensors. Don't use directly,
        instead use 'step()'
        """

        # Read differents sensors
        for s in self._sensors_to_read:

            if s == 'a':
                # Accelerometer sensor in a non filtered way
                if self._accelerometer_filtered:
                    parameters = ('A', 12, '@III')

                else:
                    parameters = ('a', 6, '@HHH')

                self._debug('WARNING: Accelerometer not yet implemented!')

            elif s == 'n':
                # Proximity sensors
                res, prox = vrep.simxGetStringSignal(self._clientID, 'EPUCK_proxSens', vrep.simx_opmode_streaming)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Proximity sensors readout failed: ", res)
                else:
                    proxVals = vrep.simxUnpackFloats(prox)
                    # TODO: Find out actual needed scaling factor 
                    proxVals = [int(x * 1000) for x in proxVals]
                    self._proximity = tuple(proxVals)

            elif s == 'm':
                # Floor sensors
                res, floor1 = vrep.simxGetFloatSignal(self._clientID, 'EPUCK_mylightSens_0', vrep.simx_opmode_streaming)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Floor 1 sensor readout failed: ", res)
                res, floor2 = vrep.simxGetFloatSignal(self._clientID, 'EPUCK_mylightSens_1', vrep.simx_opmode_streaming)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Floor 2 sensor readout failed: ", res)
                res, floor3 = vrep.simxGetFloatSignal(self._clientID, 'EPUCK_mylightSens_2', vrep.simx_opmode_streaming)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Floor 3 sensor readout failed: ", res)
                # Scale returned values to mimic real robot; current factor is just guessed                    
                self._floor_sensors = (floor1*1800, floor2*1800, floor3*1800)                

            elif s == 'q':
                # Motor position sensor
                # First: Get the handles of both motor joints
                res, leftMotor = vrep.simxGetObjectHandle(self._clientID, 'ePuck_leftJoint', vrep.simx_opmode_oneshot_wait)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Unable to get handle of left motor: ", res)
                    continue
                res, rightMotor = vrep.simxGetObjectHandle(self._clientID, 'ePuck_rightJoint', vrep.simx_opmode_oneshot_wait)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Unable to get handle of right motor: ", res)
                    continue

                # Second: Get the actual motor position (in radians)
                res, leftPos = vrep.simxGetJointPosition(self._clientID, leftMotor, vrep.simx_opmode_oneshot_wait)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Readout of left motor failed: ", res)
                    continue
                res, rightPos = vrep.simxGetJointPosition(self._clientID, rightMotor, vrep.simx_opmode_streaming)
                if res != vrep.simx_return_ok:
                    self._debug("WARNING: Readout of left motor failed: ", res)
                    continue

                self._motor_position = (leftPos, rightPos)

            elif s == 'o':
                # Light sensors
                parameters = ('O', 16, '@HHHHHHHH')
                self._debug('WARNING: Light sensors not yet implemented!')

            elif s == 'u':
                # Microphone
                parameters = ('u', 6, '@HHH')
                self._debug('WARNING: Microphones not yet implemented!')

            elif s == 'e':
                # Motor Speed
                parameters = ('E', 4, '@HH')
                self._debug('WARNING: Motor speed not yet implemented!')

            elif s == 'i':
                # Do nothing for the camera, is an independent process
                pass

            else:
                self._debug('Unknow type of sensor to read')


    #
    # Public methods
    #

    def startsim(self):
        """
        Start the simulation in synchronous mode to achieve exact simulation
        independent of the framerate. To be able to do this, you have to 
        connec to the port 19997 of vrep, edit the file
        `remoteApiConnections.txt` to contain something like

            portIndex1_port           = 19997
            portIndex1_debug          = true
            portIndex1_syncSimTrigger = false

        """
        self.set_led(8,0)
        vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)
        vrep.simxSynchronous(self._clientID, True)
        self.stepsim(1)

    def stopsim(self):
        """
        Stop the simulation if running in the syncronous mode.
        """
        self.set_led(8,1)
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

    def stepsim(self, steps):
        """
        Do n-steps of simulation.

        :param steps: Number of steps you want to simulate
        :type steps: int
        """
        for i in xrange(steps):
            vrep.simxSynchronousTrigger(self._clientID)

    def connect(self):
        """
        Connect to the V-REP simulator
        
        :return: If the connection was successful
        :rtype: boolean
        """

        if self.is_connected():
            self._debug('Already connected')
            return False
        
        self._clientID = vrep.simxStart(self._hostname, self._port, True, True, 5000, 5)
        
        if self._clientID == -1:
            self._debug("Failed to connect to remote V-REP server")
            return False

        self._debug("Connected to V-REP server")

        self.reset()
        return True

    def disconnect(self):
        """
        Disconnect from ePuck robot. Same as 'close()'
        """
        self.close()

    def close(self):
        """
        Close the connection with the robot. Same as 'disconnect()'
        :return: 0 if all ok
        :rtype: int
        """
        if self.is_connected():
            # Stop the robot
            self.stop()

            # Close the socket
            vrep.simxFinish(self._clientID)
            return 0            
        else:
            return -1

    def set_debug(self, debug):
        """
        Set / unset debug information
        :param debug: True or False, as you want or not Debug information
        :type debug: Boolean
        """

        self.debug = debug

    def send_and_receive(self, msg):
        """
        Send an Ascii message to the robot and return the reply. You can
        use it, but I don't recommend, use 'enable()', 'disable()'
        and 'step()' instead 
        
        :param msg: The message you want to send
        :type msg:    String
        :return: Response of the robot
        :rtype: String
        """

        # Check the connection
        if not self.conexion_status:
            raise Exception, 'There is not connection'

        # Make sure the Message is a string
        message = str(msg)

        # Add carriage return if not
        if not message.endswith('\n'):
            message += '\n'

        # Check the lines of the waited reply
        if message[0] in DIC_MSG:
            lines = DIC_MSG[message[0]]
        else:
            lines = 1
        self._debug('Waited lines:', lines)

        # We make 5 tries before desist
        tries = 1
        while tries < 5:
            # Send the message
            bytes = self._send(message)
            self._debug('Message sent:', repr(message))
            self._debug('Bytes sent:', bytes)

            try:
                # Receive the reply. As we want to receive a line, we have to insist
                reply = ''
                while reply.count('\n') < lines:
                    reply += self._recv()
                    if message[0] == 'R':
                        # For some reason that I don't understand, if you send a reset
                        # command 'R', sometimes you recive 1 or 2 lines of 'z,Command not found\r\n'
                        # Therefore I have to remove it from the expected message: The Hello message
                        reply = reply.replace('z,Command not found\r\n', '')
                self._debug('Message received: ', reply)
                return reply.replace('\r\n', '')

            except Exception, e:
                tries += 1
                self._debug('Communication timeout, retrying')



    def save_image(self, name='ePuck.jpg'):
        """
        Save image from ePuck's camera to disk
        
        :param name: Image name, ePuck.jpg as default
        :type name: String
        
        :return: Operation result
        :rtype:  Boolean
        """

        if self._pil_image:
            return self._pil_image.save(name)
        else:
            return False

    def get_accelerometer(self):
        """
        Return Accelerometer values in (x, y, z)
        
        :return: Accelerometer values
        :rtype: Tuple
        """
        return self._accelerometer

    def get_selector(self):
        """
        Return the selector position (0-15)
        
        :return: Selector value
        :rtype: int
        """
        return self._selector

    def get_motor_speed(self):
        """
        Return the motor speed. Correct values are in the range [-1000, 1000]
        
        :return: Motor speed
        :rtype: Tuple
        """
        return self._motor_speed

    def get_camera_parameters(self):
        """
        Return the camera parameters as a tuple
        (mode, width, height, zoom)
        
        :return: Camera parameters
        :rtype: Tuple
        """
        return self._camera_parameters

    def get_floor_sensors(self):
        """
        Return the floor sensors values as (left, center, right)
        
        :return: Floor sensors values
        :rtype: Tuple
        """
        return self._floor_sensors

    def get_proximity(self):
        """
        Return the values of the 8 proximity sensors
        
        :return: Proximity sensors values
        :rtype: Tuple
        """
        return self._proximity

    def get_light_sensor(self):
        """
        Return the value of the light sensor
        
        :return: Ligth sensor value
        :rtype: Tuple
        """
        return self._light_sensor

    def get_motor_position(self):
        """
        Return the position of the left and right motor as a tuple
        
        :return: Motor position
        :rtype: Tuple
        """
        return self._motor_position

    def get_microphone(self):
        """
        Return the volume of the three microphones
        
        :return: Microphones values
        :rtype: Tuple
        """
        return self._microphone

    def is_connected(self):
        """
        Return a boolean value that indicates if we are connected with the simulator
        
        :return: If the connection to the simulator is established
        :rtype: Boolean
        """
        if self._clientID != -1:
            return True
        else:
            return False

    def get_image(self):
        """
        Return the last image captured from the ePuck's camera (after a 'step()'). 
        None if    there are not images captured. The image is an PIL object
        
        :return: Image from robot's camera
        :rtype: PIL
        """
        return self._pil_image

    def get_sercom_version(self):
        """
        :return: Return the ePuck's firmware version
        :rtype: String
        """
        return self.send_and_receive("v")

    def set_accelerometer_filtered(self, filter=False):
        """
        Set filtered way for accelerometer, False is default value
        at the robot start
        
        :param filter: True or False, as you want
        :type filter: Boolean
        """
        self._accelerometer_filtered = filter

    def disable(self, *sensors):
        """
        Sensor(s) that you want to get disable in the ePuck
        
        :param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
        :type sensors: String
        :return: Sensors enabled
        :rtype: List
        :except Exception: Some wrong happened
        """
        for sensor in sensors:
            try:
                if not DIC_SENSORS.has_key(sensor):
                    self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
                    break

                if sensor == "camera":
                    self._cam_enable = False

                if DIC_SENSORS[sensor] in self._sensors_to_read:
                    l = list(self._sensors_to_read)
                    l.remove(DIC_SENSORS[sensor])
                    self._sensors_to_read = tuple(l)
                    self._debug('Sensor "' + sensor + '" disabled')
                else:
                    self._debug('Sensor "' + sensor + '" alrady disabled')

            except Exception, e:
                self._debug('Something wrong happened to disable the sensors: ', e)

        return self.get_sensors_enabled()

    def enable(self, *sensors):
        """
        Sensor(s) that you want to get enable in the ePuck
        
        :param sensors: Name of the sensors, take a look to DIC_SENSORS. Multiple sensors can be separated by commas
        :type sensors: String
        :return: Sensors enabled
        :rtype: List
        :except Exception: Some wrong happened
        """

        # Using the * as a parameters, we get a tuple with all sensors
        for sensor in sensors:
            try:
                if not DIC_SENSORS.has_key(sensor):
                    self._debug('Sensor "' + sensor + '" not in DIC_SENSORS')
                    break

                if sensor == "camera":
                    # If the sensor is the Camera, then we refresh the
                    # camera parameters
                    if not self._cam_enable:
                        try:
                            self._refresh_camera_parameters()
                            self._cam_enable = True
                            self.timestamp = time.time()
                        except:
                            break

                if sensor == "motor_position":
                    self._debug("INFO: motor_position currently returns the actual angle of the motor joints in V-REP [-pi:pi] and is therefore not compatible with the actual robot.")

                if DIC_SENSORS[sensor] not in self._sensors_to_read:
                    l = list(self._sensors_to_read)
                    l.append(DIC_SENSORS[sensor])
                    self._sensors_to_read = tuple(l)
                    self._debug('Sensor "' + sensor + '" enabled')
                else:
                    self._debug('Sensor "' + sensor + '" alrady enabled')

            except Exception, e:
                self._debug('Something wrong happened to enable the sensors: ', e)
        return self.get_sensors_enabled()

    def get_sensors_enabled(self):
        """
        :return: Return a list of sensors thar are active
        :rtype: List
        """
        l = []
        for sensor in DIC_SENSORS:
            if DIC_SENSORS[sensor] in self._sensors_to_read:
                l.append(sensor)
        return l

    def set_motors_speed(self, l_motor, r_motor):
        """
        Set the motors speed. The MAX and MIN speed of the ePcuk is [-1000, 1000]
        
        :param l_motor: Speed of left motor
        :type l_motor: int
        :param r_motor: Speed of right motor
        :type r_motor: int
        """

        # I don't check the MAX and MIN speed because this check
        # will be made by the ePuck's firmware. Here we need speed
        # and we lose time mading recurrent chekings

        self._actuators_to_write.append(("D", int(l_motor), int(r_motor)))

        return True

    def set_motor_position(self, l_wheel, r_wheel):
        """
        Set the motor position, useful for odometry
        
        :param l_wheel: left wheel
        :type l_wheel: int
        :param r_wheel: right wheel
        :type r_wheel: int
        """

        self._actuators_to_write.append(("P", l_wheel, r_wheel))

    def set_led(self, led_number, led_value):
        """
        Turn on/off the leds
        
        :param led_number: If led_number is other than 0-7, all leds are set to the indicated value.
        :type led_number: int
        :param led_value: 
            - 0 : Off
            - 1 : On (Red)
            - 2 : Inverse, does not work for all leds
        :type led_value: int
        """

        led = abs(led_number)
        value = abs(led_value)

        if led < 11:
            if led <= 7:
                self._actuators_to_write.append(("L", led, value))
            else:
                self._actuators_to_write.append(("L", 10, value))
                for i in range(8):
                    if value == 0:
                        self._leds_status[i] = False
                    elif value == 1:
                        self._leds_status[i] = True
                if value == 0:
                    self._leds_status[led] = False
                elif value == 1:
                    self._leds_status[led] = True
                else:
                    self._leds_status[led] = not self._leds_status[led]            
            return True
        else:
            return False

    def set_body_led(self, led_value):
        """
        Turn on /off the body led
        
        :param led_value: 
            - 0 : Off
            - 1 : On (green)
            - 2 : Inverse
        :type led_value: int
        """

        value = abs(led_value)

        self._actuators_to_write.append(("L", 8, value))

        if value == 0:
                self._leds_status[8] = False
        elif value == 1:
            self._leds_status[8] = True
        else:
            self._leds_status[8] = not self._leds_status[8]

        return True

    def set_front_led(self, led_value):
        """
        Turn on /off the front led
        
        :type    led_value: int
        :param     led_value: 
            - 0 : Off
            - 1 : On (green)
            - 2 : Inverse
        """
        value = abs(led_value)

        self._actuators_to_write.append(("L", 9, value))

        if value == 0:
                self._leds_status[9] = False
        elif value == 1:
            self._leds_status[9] = True
        else:
            self._leds_status[9] = not self._leds_status[9]

        return True

    def set_sound(self, sound):
        """
        Reproduce a sound
        
        :param sound: Sound in the range [1,5]. Other for stop
        :type sound: int
        """

        self._actuators_to_write.append(("T", sound))
        return True

    def set_camera_parameters(self, mode, width, height, zoom):
        """
        Set the camera parameters
        
        :param mode: GREY_SCALE, LINEAR_CAM, RGB_365, YUM
        :type  mode: String
        :param width: Width of the camera
        :type  width: int
        :param height: Height of the camera
        :type  height: int
        :param zoom: 1, 4, 8
        :type  zoom: int
        """

        if mode in CAM_MODE:
            self._cam_mode = CAM_MODE[mode]
        else:
            self._debug(ERR_CAM_PARAMETERS, "Camera mode")
            return -1

        if int(zoom) in CAM_ZOOM:
            self._cam_zoom = zoom
        else:
            self._debug(ERR_CAM_PARAMETERS, "Camera zoom")
            return -1

        if self.conexion_status and int(width) * int(height) <= 1600:
            # 1600 are for the resolution no greater than 40x40, I have
            # detect some problems
            self._actuators_to_write.append(("J",
                                             self._cam_mode,
                                             width,
                                             height,
                                             self._cam_zoom))
            return 0

    def calibrate_proximity_sensors(self):
        """
        Calibrate proximity sensors, keep off any object in 10 cm
        
        :return: Successful operation
        :rtype: Boolean
        """

        reply = self.send_and_receive("k")
        if reply[1] == "k":
            return True
        else:
            return False

    def reset(self):
        """
        Reset the robot
        
        :return: Successful operation
        :rtype: Boolean
        :raise Exception: If there is not connection
        """
        if not self.is_connected():
            raise Exception, 'There is not connection'

        self._debug("TODO: Reset / stop robot in simulation")

        return True

    def stop(self):
        """
        Stop the motor and turn off all leds
        :return: Successful operation
        :rtype: Boolean
        :raise Exception: If there is not connection
        """

        if not self.is_connected():
            raise Exception, 'There is not connection'

        self._debug("TODO: Stop robot")
        
        return True


    def step(self):
        """
        Method to update the sensor readings and to reflect changes in 
        the actuators. Before invoking this method is not guaranteed
        the consistency of the sensors
        """

        if not self.is_connected():
            raise Exception, 'There is not connection'

        self._write_actuators()
        self._read_sensors()

        # Get an image in 1 FPS
        if self._cam_enable and time.time() - self.timestamp > 1:
            self._read_image()
            self.timestamp = time.time()
