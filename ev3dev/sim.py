# copyright LM
#
# file version: 0.1
# not yet specified whether we use self or kwargs (it is somehow both now)
#

import vrep
import sys
import math
import time
import re

from collections import OrderedDict


CONNECT = not True
VERBOSE = not True
if VERBOSE:
    print("vrep imported")


def finish(sig=-1):
    vrep.simxFinish(sig)


def restart_simulation():
    if "clientID" in globals():  # global question
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

    vrep.simxFinish(-1)
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
    A base class for other unit types. Don't use this directly; instead, see
    :class:`SpeedPercent`, :class:`SpeedRPS`, :class:`SpeedRPM`,
    :class:`SpeedDPS`, and :class:`SpeedDPM`.
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

    def __init__(self, percent):
        assert -100 <= percent <= 100,\
            "{} is an invalid percentage, must be between -100 and 100 (inclusive)".format(percent)

        self.percent = percent

    def __str__(self):
        return str(self.percent) + "%"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
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
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
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
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedRPS(self.rotations_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired rotations-per-second
        """
        assert abs(self.rotations_per_second) <= motor.max_rps,\
            "invalid rotations-per-second: {} max RPS is {}, {} was requested".format(
            motor, motor.max_rps, self.rotations_per_second)
        return self.rotations_per_second/motor.max_rps * motor.max_speed


class SpeedRPM(SpeedValue):
    """
    Speed in rotations-per-minute.
    """

    def __init__(self, rotations_per_minute):
        self.rotations_per_minute = rotations_per_minute

    def __str__(self):
        return str(self.rotations_per_minute) + " rot/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedRPM(self.rotations_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired rotations-per-minute
        """
        assert abs(self.rotations_per_minute) <= motor.max_rpm,\
            "invalid rotations-per-minute: {} max RPM is {}, {} was requested".format(
            motor, motor.max_rpm, self.rotations_per_minute)
        return self.rotations_per_minute/motor.max_rpm * motor.max_speed


class SpeedDPS(SpeedValue):
    """
    Speed in degrees-per-second.
    """

    def __init__(self, degrees_per_second):
        self.degrees_per_second = degrees_per_second

    def __str__(self):
        return str(self.degrees_per_second) + " deg/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedDPS(self.degrees_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired degrees-per-second
        """
        assert abs(self.degrees_per_second) <= motor.max_dps,\
            "invalid degrees-per-second: {} max DPS is {}, {} was requested".format(
            motor, motor.max_dps, self.degrees_per_second)
        return self.degrees_per_second/motor.max_dps * motor.max_speed


class SpeedDPM(SpeedValue):
    """
    Speed in degrees-per-minute.
    """

    def __init__(self, degrees_per_minute):
        self.degrees_per_minute = degrees_per_minute

    def __str__(self):
        return str(self.degrees_per_minute) + " deg/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedDPM(self.degrees_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired degrees-per-minute
        """
        assert abs(self.degrees_per_minute) <= motor.max_dpm,\
            "invalid degrees-per-minute: {} max DPM is {}, {} was requested".format(
            motor, motor.max_dpm, self.degrees_per_minute)
        return self.degrees_per_minute/motor.max_dpm * motor.max_speed



class MotorSet(object):

    def __init__(self, motor_specs, desc=None):
        """
        motor_specs is a dictionary such as
        {
            OUTPUT_A : LargeMotor,
            OUTPUT_C : LargeMotor,
        }
        """
        self.motors = OrderedDict()
        for motor_port in sorted(motor_specs.keys()):
            motor_class = motor_specs[motor_port]
            self.motors[motor_port] = motor_class(motor_port)
            self.motors[motor_port].reset()

        self.desc = desc

    def __str__(self):

        if self.desc:
            return self.desc
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
                        #log.error("%s %s cannot set %s to %s" % (self, motor, key, kwargs[key]))
                        raise e

    def set_polarity(self, polarity, motors=None):
        pass
        # valid_choices = (LargeMotor.POLARITY_NORMAL, LargeMotor.POLARITY_INVERSED)
        #
        # assert polarity in valid_choices,\
        #     "%s is an invalid polarity choice, must be %s" % (polarity, ', '.join(valid_choices))
        # motors = motors if motors is not None else self.motors.values()
        #
        # for motor in motors:
        #     motor.polarity = polarity

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
    ev3def = {'MOTION_TORQUE': 0.2,
              'REST_TORQUE': 0.4,
              'MAX_SPEED': 18.326,
              }

    def __init__(self, address=None, **kwargs):
        self.kwargs = kwargs

        if address is not None:
            self.kwargs['address'] = address
        self.kwargs.update(Motor.ev3def)
        self.address = address

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)
            # print("Need clientID")
            # sys.exit("No client")

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

    def run_forever(speed_sp=0):
        print("Not implemented")
        pass

        step = 1
        while self.kwargs.get('state') == 'run':
            errorCodeJTV = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                    self.kwargs.get('motor'), speed_sp, vrep.simx_opmode_streaming) #oneshot_wait)
            errorCodeSJF = vrep.simxSetJointForce(self.kwargs['clientID'],
                    self.kwargs.get('motor'), self.kwargs['MOTION_TORQUE'], vrep.simx_opmode_streaming)
            errorCodeP, wheel_pos = vrep.simxGetJointPosition(self.kwargs['clientID'],
                    self.kwargs.get('motor'), vrep.simx_opmode_oneshot)
        return 0

    def run_to_rel_pos(self, position_sp, speed_sp):
        if VERBOSE:
            print("Partly implemented.")


        # wait (10-20ms) for robot to send back the data...
        rC = 1
        while rC != vrep.simx_error_noerror:
            rC, ini_wheel_pos = vrep.simxGetJointPosition(
                self.kwargs['clientID'], self.kwargs.get('motor'),
                vrep.simx_opmode_oneshot)

        # an actuall motion
        iteration = 0
        wheel_pos = ini_wheel_pos
        position_sp_rad = math.radians(position_sp)
        while wheel_pos - ini_wheel_pos <= position_sp_rad:
            # and iteration < 10:
            iteration += 1

            errorCodeJTV = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                    self.kwargs.get('motor'), speed_sp, vrep.simx_opmode_streaming) #oneshot_wait)
            errorCodeSJF = vrep.simxSetJointForce(self.kwargs['clientID'],
                    self.kwargs.get('motor'), self.kwargs['MOTION_TORQUE'], vrep.simx_opmode_streaming) #oneshot_wait)

            # positions of a wheel
            errorCodeP, wheel_pos = vrep.simxGetJointPosition(self.kwargs['clientID'],
                    self.kwargs.get('motor'), vrep.simx_opmode_oneshot)

            if VERBOSE:
                print("{}. ini:{} wheel:{} pos:{} speed:{}".format(iteration, ini_wheel_pos, wheel_pos, position_sp_rad, speed_sp))
                print("ERRORS: vel:{} pos:{} force:{} init:{}".format(errorCodeJTV, errorCodeP, errorCodeSJF, rC))

        # full stop
        errorCode = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                self.kwargs.get('motor'), 0, vrep.simx_opmode_blocking)
        errorCode = vrep.simxSetJointForce(self.kwargs['clientID'],
                self.kwargs.get('motor'), self.kwargs['REST_TORQUE'], vrep.simx_opmode_blocking)

        return wheel_pos

    def __str__(self):
        if 'address' in self.kwargs:
            return "%s(%s)" % (self.__class__.__name__, self.kwargs.get('address'))
        else:
            return "A" + self.__class__.__name__

    def __repr__(self):
        return self.__str__()


class LargeMotor(Motor):
    """Large Motor class"""
    def __init__(self, address=None, **kwargs):
        super(LargeMotor, self).__init__(address, **kwargs)


class MoveTank(LargeMotor):
    """
    Controls a pair of motors simultaneously, via individual speed setpoints for each motor.
    Example:
    .. code:: python
        tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
        # drive in a turn for 10 rotations of the outer motor
        tank_drive.on_for_rotations(50, 75, 10)
    """

    def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=LargeMotor):
        motor_specs = {
            left_motor_port : motor_class,
            right_motor_port : motor_class,
        }

        #TODO: MotorSet.__init__(self, motor_specs, desc)
        self.left_motor = LargeMotor(left_motor_port)
        self.right_motor = LargeMotor(right_motor_port)


    def _speed_native_units(self, speed, label=None):
        # If speed is not a SpeedValue object we treat it as a percentage
        if not isinstance(speed, SpeedValue):
            assert -100 <= speed <= 100,\
                "{}{} is an invalid speed percentage, must be between -100 and 100 (inclusive)".format("" if label is None else (label + ": ") , speed)
            speed = SpeedPercent(speed)

        return speed.to_native_units(self)

    def _unpack_speeds_to_native_units(self, left_speed, right_speed):
        left_speed = self.left_motor._speed_native_units(left_speed, "left_speed")
        right_speed = self.right_motor._speed_native_units(right_speed, "right_speed")

        return (
            left_speed,
            right_speed
        )

    def on_for_degrees(self, left_speed, right_speed, degrees, brake=True, block=True):

        factor = 1
        for deg in range(degrees * factor):
            errorCode = vrep.simxPauseCommunication(clientID, not True)
            if VERBOSE:
                print("PauseComm Przed:", errorCode)
            self.left_motor.run_to_rel_pos(1 / factor, left_speed)
            self.right_motor.run_to_rel_pos(1 / factor, right_speed)
            errorCode = vrep.simxPauseCommunication(clientID, False)
            if VERBOSE:
                print("PauseComm Po:", errorCode)

    def a(self):
        if VERBOSE:
            print("Partly implemented.")


        # wait (10-20ms) for robot to send back the data...
        rC = 1
        while rC != vrep.simx_error_noerror:
            rC, ini_wheel_pos = vrep.simxGetJointPosition(
                self.kwargs['clientID'], self.kwargs.get('motor'),
                vrep.simx_opmode_oneshot)

        # an actuall motion
        iteration = 0
        wheel_pos = ini_wheel_pos
        position_sp_rad = math.radians(position_sp)
        while wheel_pos - ini_wheel_pos <= position_sp_rad:
            # and iteration < 10:
            iteration += 1

            errorCodeJTV = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                    self.kwargs.get('motor'), speed_sp, vrep.simx_opmode_streaming) #oneshot_wait)
            errorCodeSJF = vrep.simxSetJointForce(self.kwargs['clientID'],
                    self.kwargs.get('motor'), self.kwargs['MOTION_TORQUE'], vrep.simx_opmode_streaming) #oneshot_wait)

            # positions of a wheel
            errorCodeP, wheel_pos = vrep.simxGetJointPosition(self.kwargs['clientID'],
                    self.kwargs.get('motor'), vrep.simx_opmode_oneshot)

            if VERBOSE:
                print("{}. ini:{} wheel:{} pos:{} speed:{}".format(iteration, ini_wheel_pos, wheel_pos, position_sp_rad, speed_sp))
                print("ERRORS: vel:{} pos:{} force:{} init:{}".format(errorCodeJTV, errorCodeP, errorCodeSJF, rC))

        # full stop
        errorCode = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                self.kwargs.get('motor'), 0, vrep.simx_opmode_blocking)
        errorCode = vrep.simxSetJointForce(self.kwargs['clientID'],
                self.kwargs.get('motor'), self.kwargs['REST_TORQUE'], vrep.simx_opmode_blocking)

        return wheel_pos

    def on_for_rotations(self, left_speed, right_speed, rotations, brake=True, block=True):
        """
        Rotate the motors at 'left_speed & right_speed' for 'rotations'. Speeds
        can be percentages or any SpeedValue implementation.
        If the left speed is not equal to the right speed (i.e., the robot will
        turn), the motor on the outside of the turn will rotate for the full
        ``rotations`` while the motor on the inside will have its requested
        distance calculated according to the expected turn.
        """
        MoveTank.on_for_degrees(self, left_speed, right_speed, rotations * 360, brake, block)


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

    DEVICE_ROOT_PATH = '/sys/class'

    _DEVICE_INDEX = re.compile(r'^.*(\d+)$')

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
    # VREP_GYRO_NAME = 'GyroSensor_reference_G'
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

        errorCode, self.gyro = vrep.simxGetObjectHandle(self.kwargs.get('clientID'),
                    GyroSensor.VREP_GYRO_NAME,
                    vrep.simx_opmode_oneshot_wait)

        if VERBOSE:
            print("GyroError, GyroCodeNr", errorCode, self.gyro)

    def __str__(self):
        return str(self.gyro)

    def value(self):
        rC = 1
        # while rC != vrep.simx_error_noerror:
        if 1:
            rC, signalValue = vrep.simxGetFloatSignal(
                self.kwargs.get('clientID'),
                GyroSensor.VREP_GYRO_NAME,
                vrep.simx_opmode_buffer
            )

            # this works only in Ceppelia
            # matrix = vrep.simxGetObjectMatrix(
            #     self.gyro,
            #     -1,
            #     vrep.simxServiceCall
            # )

        # return vrep.simxgetObjectMatrix(self.gyro, -1)
        return rC, signalValue  #, matrix

    # oldTransformationMatrix=sim.getObjectMatrix(ref,-1)


# Gyro


# vrep simulation init
# v0.1 - clientID will be global
#
# TODO: 1. move (maybe) to a function and call here...
#       2. think of possible user specific clientID (in kwargs?)
#       3. rewrite self.kwargs['arg'] to self.kwargs.get('arg')
if CONNECT:
    vrep.simxFinish(-1)
    clientID = connection()
