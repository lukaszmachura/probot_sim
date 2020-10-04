import sys
import logging
import time
from ev3dev2.sensor import Sensor
from ev3dev2 import *

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')

class GyroSensor(Sensor):
    """
    LEGO EV3 gyro sensor.
    """

    VREP_GYRO_NAME = 'Giroscopio'
    # VREP_GYRO_NAME = 'GyroSensor_G'
    # VREP_GYRO_NAME = 'GyroSensor_reference_G'
    # VREP_GYRO_NAME = 'GyroSensor_VA'
    # VREP_GYRO_NAME = 'GyroSensor_reference'

    # vrep
    def __init__(self, **kwargs):
        self.kwargs = kwargs

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)
            # print("Need clientID")
            # sys.exit("No client")

        if "gyroName" in self.kwargs:
            self.VREP_GYRO_NAME = self.kwargs.get('gyroName')
        else:
            self.kwargs['gyroName'] = self.VREP_GYRO_NAME

        # object handle
        errorCode, self.gyro = vrep.simxGetObjectHandle(
            self.kwargs.get('clientID'),
            # GyroSensor.VREP_GYRO_NAME,
            self.VREP_GYRO_NAME,
            vrep.simx_opmode_oneshot_wait)

        if VERBOSE:
            print("GyroError, GyroCodeNr", errorCode, self.gyro)

        self.connected = True
        self.units = 'degrees'
        self.mode = 'GYRO-ANG'  # or 'GYRO-RATE'

    @property
    def mode(self):
        return self.__mode

    @mode.setter
    def mode(self, var):
        if var in ['GYRO_ANG', 'GYRO_RATE']:
            self.__mode = var
        else:
            self.__mode = 'GYRO_ANG'

    def __str__(self):
        return f"{self.gyro} in {self.mode} mode"

    def __repr__(self):
        return self.__str__()

    def angle(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorGyroA',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        _angle = outInts[0]
        return _angle

    def angular_velocity(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorGyroVA',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        _angular_velocity = outInts[0]
        return _angular_velocity

    def value(self):
        if self.mode == 'GYRO-RATE':
            return self.angular_velocity()
        else:  # 'GYRO-ANG':
            return self.angle()

    def velocity(self):
        return self.angular_velocity()

    def state(self):
        return self.angle(), self.angular_velocity()
