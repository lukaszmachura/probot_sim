# copyright LM
#
# file version: 0.1
# not yet specified whether we use self or kwargs (it is somehow both now)
#

"""
.. _Google Python Style Guide:
   http://google.github.io/styleguide/pyguide.html
"""

import vrep
import sys
import math
import time
import re
from time import sleep

from collections import OrderedDict

api_return_codes = {
    0: 'OK',  # 'simx_return_ok; The function executed fine',
    1: 'simx_return_novalue_flag; There is no command reply in the input buffer. This should not always be considered as an error, depending on the selected operation mode',
    2: 'simx_return_timeout_flag; The function timed out (probably the network is down or too slow)',
    4: 'simx_return_illegal_opmode_flag; The specified operation mode is not supported for the given function',
    8: 'simx_return_remote_error_flag; The function caused an error on the server side (e.g. an invalid handle was specified)',
    16: 'simx_return_split_progress_flag; The communication thread is still processing previous split command of the same type',
    32: 'simx_return_local_error_flag; The function caused an error on the client side',
    64: 'simx_return_initialize_error_flag; simxStart was not yet called'
}

CONNECT = not True
VERBOSE = not True
if VERBOSE:
    print("vrep imported")


def finish(sig=-1):
    "wrapper for vrep command"
    vrep.simxFinish(sig)


def restart_simulation():
    if ("clientID" in globals()) and VERBOSE:  # global question
        print("global ID")
    else:
        print("connecting...")
        global clientID
        clientID = connection()

    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)


def connection(connectionAddress='127.0.0.1',
                connectionPort=19999,
                waitUntilConnected=True,
                doNotReconnectOnceDisconnected=True,
                timeOutInMs=5000,
                commThreadCycleInMs=5,
                dummy=False):
    """wrapper for connection with vrep server"""
    global clientID

    if dummy:
        return 666

    finish(-1)  #wrapper for vrep.simxFinish
    clientID = vrep.simxStart(connectionAddress,
                    connectionPort,
                    waitUntilConnected,
                    doNotReconnectOnceDisconnected,
                    timeOutInMs,
                    commThreadCycleInMs)

    if VERBOSE:
        if clientID != -1:
            print('Connected to remote server')
        else:
            print('Not connected')
            sys.exit('Could not connect')

    return clientID


class SpeedValue(object):
    """
    A base class for other unit types.
    Don't use this directly; instead, see
    :class:`SpeedPercent`,
    :class:`SpeedRPS`,
    :class:`SpeedRPM`,
    :class:`SpeedDPS`, and
    :class:`SpeedDPM`.
    """

    def __eq__(self, other):
        return self.to_native_units() == other.to_native_units()

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        return self.to_native_units() < other.to_native_units()

    def __le__(self, other):
        return self.to_native_units() <= other.to_native_units()

    def __gt__(self, other):
        return self.to_native_units() > other.to_native_units()

    def __ge__(self, other):
        return self.to_native_units() >= other.to_native_units()

    def __rmul__(self, other):
        return self.__mul__(other)


class SpeedPercent(SpeedValue):
    """
    Speed as a percentage of the motor's maximum rated speed.
    """
    # max_speed = 18.326

    def __init__(self, percent):
        assert -100 <= percent <= 100,\
            f"{percent} is an invalid percentage, must be between \
             -100 and 100 (inclusive)"

        self.percent = percent

    def __str__(self):
        return str(self.percent) + "%"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
            f"{self} can only be multiplied by an int or float"
        return SpeedPercent(self.percent * other)

    def to_native_units(self, motor):
        """
        Return this SpeedPercent in native motor units
        """
        return self.percent / 100 * motor.max_speed


class SpeedNativeUnits(SpeedValue):
    """
    Speed in tacho counts per second.
    """

    def __init__(self, native_counts):
        self.native_counts = native_counts

    def __str__(self):
        return "{:.2f}".format(self.native_counts) + " counts/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
            f"{self} can only be multiplied by an int or float"
        return SpeedNativeUnits(self.native_counts * other)

    def to_native_units(self, motor=None):
        """
        Return this SpeedNativeUnits as a number
        """
        return self.native_counts


class SpeedRPS(SpeedValue):
    """
    Speed in rotations-per-second.
    """

    def __init__(self, rotations_per_second):
        self.rotations_per_second = rotations_per_second

    def __str__(self):
        return str(self.rotations_per_second) + " rot/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
            f"{self} can only be multiplied by an int or float"
        return SpeedRPS(self.rotations_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve
        desired rotations-per-second
        """
        assert abs(self.rotations_per_second) <= motor.max_rps,\
            f"invalid rotations-per-second: \
            {motor} max RPS is {motor.max_rps}, \
            {self.rotations_per_second} was requested"
        return self.rotations_per_second / motor.max_rps * motor.max_speed


class SpeedRPM(SpeedValue):
    """
    Speed in rotations-per-minute.
    """

    def __init__(self, rotations_per_minute):
        self.rotations_per_minute = rotations_per_minute

    def __str__(self):
        return str(self.rotations_per_minute) + " rot/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
        f"{self} can only be multiplied by an int or float"
        return SpeedRPM(self.rotations_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve
        desired rotations-per-minute
        """
        assert abs(self.rotations_per_minute) <= motor.max_rpm,\
            "invalid rotations-per-minute: {} max RPM is {}, {} \
            was requested".format(
            motor, motor.max_rpm, self.rotations_per_minute)
        return self.rotations_per_minute / motor.max_rpm * motor.max_speed


class SpeedDPS(SpeedValue):
    """
    Speed in degrees-per-second.
    """

    def __init__(self, degrees_per_second):
        self.degrees_per_second = degrees_per_second

    def __str__(self):
        return str(self.degrees_per_second) + " deg/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
        "{} can only be multiplied by an int or float".format(self)
        return SpeedDPS(self.degrees_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve
        desired degrees-per-second
        """
        assert abs(self.degrees_per_second) <= motor.max_dps,\
            "invalid degrees-per-second: {} max DPS is {}, {} \
            was requested".format(
            motor, motor.max_dps, self.degrees_per_second)
        return self.degrees_per_second / motor.max_dps * motor.max_speed


class SpeedDPM(SpeedValue):
    """
    Speed in degrees-per-minute.
    """

    def __init__(self, degrees_per_minute):
        self.degrees_per_minute = degrees_per_minute

    def __str__(self):
        return str(self.degrees_per_minute) + " deg/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), \
        "{} can only be multiplied by an int or float".format(self)
        return SpeedDPM(self.degrees_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve
        desired degrees-per-minute
        """
        assert abs(self.degrees_per_minute) <= motor.max_dpm,\
            "invalid degrees-per-minute: {} max DPM is {}, {} \
            was requested".format(
            motor, motor.max_dpm, self.degrees_per_minute)
        return self.degrees_per_minute / motor.max_dpm * motor.max_speed


class Wheel:
    """
    Wheel

    Args:
        radius (float)
        width (float)
        **kwargs (dict): note, that named args have precedence over kwargs

    Attributes:
        radius:

    """
    def __init__(self, radius=None, width=None, **kwargs):
        self.kwargs = kwargs
        self.radius = radius
        self.width = width
        self.circumference = None

    @property
    def radius(self):
        return self._radius

    @radius.setter
    def radius(self, var):
        assert isinstance(var, (int, float))
        self._radius = var

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, var):
        assert isinstance(var, (int, float))
        self._width = var

    @property
    def circumference(self):
        return self._circumference

    @circumference.setter
    def circumference(self, var):
        from math import pi
        self._circumference = 2 * pi * self.radius

    def __str__(self):
        if 'description' not in self.kwargs:
            ret = f"Wheel of radius {self.radius} "
            ret += f"and cicrumference {self.circumference}"
        else:
            ret = str(self.kwargs['description'])
        return ret

    def __repr__(self):
        return self.__str__()


class LegoWheel_41896c04(Wheel):
    """
    Standard Lego Wheel and Tire assembly 41896c04

    Wheel 43.2mm D. x 26mm Technic Racing Small,
    3 Pin Holes with Black Tire 56 x 28 ZR Street (41896 / 41897)
    """
    diameter = 56
    width = 28
    units = 'mm'

    def __init__(self):
        super().__init__(
            radius=__class__.diameter / 2,
            width=__class__.width,
            units=__class__.units
        )


class MotorSet(object):
    """
    The MotorSet object is a parent class for collection of Lego motors

    Args:
        motor_specs (dict):
        {
            OUTPUT_A : LargeMotor,
            OUTPUT_C : LargeMotor,
        }
        desc (str, optional): contains description of the motors

    Attributes:
        motors (ordered dict): containes motors' names and descriptions
        desc (str): contains description of the motors
    """

    def __init__(self, motor_specs, desc=None):

        self.motors = OrderedDict()
        # actual port names
        for motor_port in sorted(motor_specs.keys()):
            motor_class = motor_specs[motor_port]
            self.motors[motor_port] = motor_class(motor_port)
            self.motors[motor_port].reset()

        # left/right wheel
        for motor_port in desc:
            self.motors[motor_port] = self.motors[desc[motor_port]]
        self.desc = desc

    def __str__(self):
        if self.desc:
            if isinstance(self.desc, str):
                return self.desc
            else:
                return str(self.desc)
        else:
            return self.__class__.__name__

    def set_args(self, **kwargs):
        motors = kwargs.get('motors', self.motors.values())

        for motor in motors:
            for key in kwargs:
                if key != 'motors':
                    try:
                        setattr(motor, key, kwargs[key])
                    except AttributeError as e:
                        # log.error("%s %s cannot set %s to %s" % (self, motor,
                        #           key, kwargs[key]))
                        raise e

    def set_polarity(self, polarity, motors=None):
        raise NotImplementedError
        pass

    def _run_command(self, **kwargs):
        motors = kwargs.get('motors', self.motors.values())

        for motor in motors:
            for key in kwargs:
                if key not in ('motors', 'commands'):
                    setattr(motor, key, kwargs[key])

        for motor in motors:
            motor.command = kwargs['command']
            #log.debug("%s: %s command %s" % (self, motor, kwargs['command']))


class Motor:
    """Motor class"""
    ev3def = {
              'MOTION_TORQUE': 0.2,
              'REST_TORQUE': 0.4,
              'max_speed': 18.326,
              'MAX_SPEED': 18.326,
              'wheel': LegoWheel_41896c04(),
              }

    def __init__(self, address=None, **kwargs):
        self.kwargs = kwargs

        if 'B' in address:
            self.int_motor = 1
            self.str_motor = 'B'
            self.kwargs['int motor'] = 1
            self.kwargs['str motor'] = 'B'
            address = "Motor_B"
        elif 'C' in address:
            self.int_motor = 2
            self.str_motor = 'C'
            self.kwargs['int motor'] = 2
            self.kwargs['str motor'] = 'C'
            address = "Motor_C"
        else:
            raise ValueError('Not a TANK MODE, address single motor')

        if address is not None:
            self.kwargs['address'] = address
        self.kwargs.update(Motor.ev3def)
        self.address = address

        if 'max_speed' not in self.kwargs:
            self.kwargs['max_speed'] = Motor.ev3def['max_speed']
        self.max_speed = self.kwargs['max_speed']

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)

        # vrep
        eC, motor = vrep.simxGetObjectHandle(self.kwargs['clientID'],
                self.kwargs.get('address'), vrep.simx_opmode_oneshot_wait)
        self.motor = motor
        self.handle = motor
        self.kwargs['handle'] = motor
        self.kwargs['motor'] = motor

        if 'state' in self.kwargs:
            self.state = self.kwargs.get('state')
        else:
            self.kwargs['state'] = 'break'

    def initialize_motor(self):
        # wait (10-20ms) for robot to send back the data...
        rC = 1
        while rC != vrep.simx_error_noerror:
            rC, ini_wheel_pos = vrep.simxGetJointPosition(
                self.kwargs['clientID'],
                self.kwargs.get('motor'),
                vrep.simx_opmode_oneshot)

        return rC, ini_wheel_pos

    def full_stop(self):
        errorCode = vrep.simxSetJointForce(
            self.kwargs['clientID'],
            self.kwargs.get('motor'),
            self.kwargs['REST_TORQUE'],
            vrep.simx_opmode_blocking)
        errorCode = vrep.simxSetJointTargetVelocity(
            self.kwargs['clientID'],
            self.kwargs.get('motor'),
            0,
            vrep.simx_opmode_blocking)

    def stop(self, **kwargs):
        "..."
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'Off',
            [self.int_motor], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode

    def run(self, speed_sp):
        """
        speed (int/float): speed of motor, in [-MAX_SPEED, MAX_SPEED]
        """
        speed_sp = int(self._speed_native_units(speed_sp))
        if VERBOSE:
            print(f"speed: {speed_sp}")

        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'On',
            [self.int_motor, speed_sp], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode

    def run_forever(self, speed_sp):
        return self.run(speed_sp)

    def runforever(self, speed_sp):
        return self.run(speed_sp)

    def run_to_rel_pos(self, position_sp, speed_sp, **kwargs):
        # if VERBOSE:
        #     print(f"{__class__.__name__}.run_to_rel_pos: Partly implemented.")

        if kwargs.get('wait', False) == True:
            optmode = vrep.simx_opmode_oneshot_wait
        else:
            optmode = vrep.simx_opmode_streaming

        speed_sp = self._speed_native_units(speed_sp)

        # wait (10-20ms) for robot to send back the data...
        rC, ini_wheel_pos = self.initialize_motor()

        # an actuall motion
        iteration = 0
        wheel_pos = ini_wheel_pos
        position_sp_rad = math.radians(position_sp)
        while wheel_pos - ini_wheel_pos <= position_sp_rad:
            # and iteration < 10:
            iteration += 1

            errorCodeJTV = vrep.simxSetJointTargetVelocity(
                self.kwargs['clientID'],
                self.kwargs.get('motor'),
                speed_sp,
                optmode)
            errorCodeSJF = vrep.simxSetJointForce(
                self.kwargs['clientID'],
                self.kwargs.get('motor'),
                self.kwargs['MOTION_TORQUE'],
                optmode)
            # positions of a wheel
            errorCodeP, wheel_pos = vrep.simxGetJointPosition(
                self.kwargs['clientID'],
                self.kwargs.get('motor'),
                optmode) #vrep.simx_opmode_streaming) #simx_opmode_oneshot)

            if VERBOSE:
                print("{}. ini:{} wheel:{} pos:{} speed:{}".format(
                    iteration, ini_wheel_pos,
                    wheel_pos, position_sp_rad,
                    speed_sp))
                print("ERRORS: vel:{} pos:{} force:{} init:{}".format(
                    errorCodeJTV, errorCodeP,
                    errorCodeSJF, rC))

        # full stop
        self.full_stop()

        return wheel_pos

    def _speed_native_units(self, speed, label=None):
        # If speed is not a SpeedValue object we treat it as a percentage
        if not isinstance(speed, SpeedValue):
            assert -100 <= speed <= 100,\
                "{}{} is an invalid speed percentage, must \
                be between -100 and 100 (inclusive)".format(
                "" if label is None else (label + ": ") , speed)
            speed = SpeedPercent(speed)

        return speed.to_native_units(self)

    def __str__(self):
        if 'address' in self.kwargs:
            return "%s(%s)" % (self.__class__.__name__,
                self.kwargs.get('address'))
        else:
            return "A" + self.__class__.__name__

    def __repr__(self):
        return self.__str__()

    def reset_rotation_count(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'ResetRotationCount',
            [self.int_motor], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode


class LargeMotor(Motor):
    """Large Motor class"""
    def __init__(self, address=None, **kwargs):
        super(LargeMotor, self).__init__(address, **kwargs)

    def reset(self):
        return self.reset_rotation_count()


class MoveTank: #(LargeMotor):
    """
    Controls a pair of motors simultaneously, via individual speed setpoints
    for each motor.
    Example:
    .. code:: python
        tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
        # drive in a turn for 10 rotations of the outer motor
        tank_drive.on_for_rotations(50, 75, 10)
    """

    def __init__(self, left_motor_port, right_motor_port,
                 desc=None, motor_class=LargeMotor):
        motor_specs = {
            left_motor_port : motor_class,
            right_motor_port : motor_class,
        }
        if desc is None:
            desc = {
                'Left motor port': left_motor_port,
                'Right motor port': right_motor_port,
            }

        MotorSet.__init__(self, motor_specs, desc)

    def _unpack_speeds_to_native_units(self, left_speed, right_speed):
        left_speed = self.motors['Left motor port']._speed_native_units(
            left_speed, "left_speed")
        right_speed = self.motors['Right motor port']._speed_native_units(
            right_speed, "right_speed")

        return (
            left_speed,
            right_speed
        )

    def on_for_degrees(self,
            left_speed, right_speed, degrees,
            brake=True, block=True):

        if brake is not True:
            print(f'{__class__.__name__} break not implemented')

        if block is not True:
            print(f'{__class__.__name__} block not implemented')

        # deg to rad
        if degrees == 'full':
            degrees = 360
        if degrees == 'half':
            degrees = 180
        if degrees == 'right':
            degrees = 90
        rad = degrees * math.pi / 180

        _clientID = self.motors['Left motor port'].kwargs['clientID']
        _left_motor = self.motors['Left motor port']
        _right_motor = self.motors['Right motor port']

        left_speed, right_speed = self._unpack_speeds_to_native_units(
            left_speed, right_speed
        )

        rCL, ini_pos_L = _left_motor.initialize_motor()
        rCR, ini_pos_R = _right_motor.initialize_motor()
        if VERBOSE:
            print(ini_pos_L, ini_pos_R)

        pos_L, pos_R = ini_pos_L, ini_pos_R
        while ((abs(pos_L - ini_pos_L) <= rad)
            and (abs(pos_R - ini_pos_R) <= rad)):

            err_vL = vrep.simxSetJointTargetVelocity(
                _clientID, _left_motor.kwargs.get('motor'), left_speed,
                vrep.simx_opmode_oneshot_wait
            )
            err_vR = vrep.simxSetJointTargetVelocity(
                _clientID, _right_motor.kwargs.get('motor'), right_speed,
                vrep.simx_opmode_oneshot_wait
            )

            rC, pos_L = vrep.simxGetJointPosition(
                _clientID,
                _left_motor.kwargs.get('motor'),
                vrep.simx_opmode_streaming)
            rC, pos_R = vrep.simxGetJointPosition(
                _clientID,
                _right_motor.kwargs.get('motor'),
                vrep.simx_opmode_streaming)
            if VERBOSE:
                print(pos_L - ini_pos_L, pos_R - ini_pos_R)

        self.motors['Left motor port'].full_stop()
        self.motors['Right motor port'].full_stop()

    def on_for_rotations(self, left_speed, right_speed,
                         rotations, brake=True, block=True):
        """
        Rotate the motors at 'left_speed & right_speed' for 'rotations'. Speeds
        can be percentages or any SpeedValue implementation.
        If the left speed is not equal to the right speed (i.e., the robot will
        turn), the motor on the outside of the turn will rotate for the full
        ``rotations`` while the motor on the inside will have its requested
        distance calculated according to the expected turn.
        """
        MoveTank.on_for_degrees(self, left_speed, right_speed,
            rotations * 360, brake, block)


# -----------------------------------------------------------------------------
# Define the base class from which all other ev3dev classes are defined.

class Device(object):
    """The ev3dev device base class"""

    __slots__ = [
        '_path',
        '_device_index',
        '_attr_cache',
        'kwargs',
    ]

    # DEVICE_ROOT_PATH = '/sys/class'
    # _DEVICE_INDEX = re.compile(r'^.*(\d+)$')
    # to be implemented


class Sensor(Device):
    """
    The sensor class provides a uniform interface for using most of the
    sensors available for the EV3.
    """

    SYSTEM_CLASS_NAME = 'lego-sensor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'
    __slots__ = [
    '_address',
    '_command',
    '_commands',
    '_decimals',
    '_driver_name',
    '_mode',
    '_modes',
    '_num_values',
    '_units',
    '_value',
    '_bin_data_format',
    '_bin_data_size',
    '_bin_data',
    '_mode_scale'
    ]

    # TO BE implemented


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

    def __str__(self):
        return str(self.gyro)

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
        return self.angle()

    def velocity(self):
        return self.angular_velocity()

    def state(self):
        return self.angle(), self.angular_velocity()



# vrep simulation init
# v0.1 - clientID will be global
#
# TODO: 1. move (maybe) to a function and call here...
#       2. think of possible user specific clientID (in kwargs?)
#       3. rewrite self.kwargs['arg'] to self.kwargs.get('arg')
#       4. add __slots__ to Motor
if __name__ == "__main__" and CONNECT:
    vrep.simxFinish(-1)
    clientID = connection()
