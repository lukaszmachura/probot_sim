# copyright LM
#
# file version: 0.1
# not yet specified whether we use self or kwargs (it is somehow both now)
#

import vrep
import sys
import math

from collections import OrderedDict


CONNECT = not True
VERBOSE = True
if VERBOSE:
    print("vrep imported")


def finish(sig=-1):
    vrep.simxFinish(sig)


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

    def run_to_rel_pos(self, position_sp, speed_sp):
        if VERBOSE:
            print("Partly implemented.")

        rC, ini_wheel_pos = vrep.simxGetJointPosition(
            self.kwargs['clientID'], self.kwargs.get('motor'),
            vrep.simx_opmode_streaming)

        wheel_pos = ini_wheel_pos
        while wheel_pos - ini_wheel_pos < math.radians(position_sp):

            # wait for both motors
            # vrep.simxPauseCommunication(self.kwargs['clientID'], True)

            errorCode = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                    self.kwargs['motor'], speed_sp, vrep.simx_opmode_streaming)
            errorCode = vrep.simxSetJointForce(self.kwargs['clientID'],
                    self.kwargs['motor'], self.kwargs['MOTION_TORQUE'], vrep.simx_opmode_streaming)
            # positions of a wheel
            errorCode, wheel_pos = vrep.simxGetJointPosition(self.kwargs['clientID'],
                    self.kwargs['motor'], vrep.simx_opmode_streaming)

            # vrep.simxPauseCommunication(self.kwargs['clientID'], False)

        # full stop
        errorCode = vrep.simxSetJointTargetVelocity(self.kwargs['clientID'],
                self.kwargs['motor'], 0, vrep.simx_opmode_blocking)
        errorCode = vrep.simxSetJointForce(self.kwargs['clientID'],
                self.kwargs['motor'], self.kwargs['REST_TORQUE'], vrep.simx_opmode_blocking)

        # vrep.simxFinish(self.kwargs['clientID'])

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


class MoveTank:
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
        self.left_motor = self.motors[left_motor_port]
        self.right_motor = self.motors[right_motor_port]
        self.max_speed = self.left_motor.max_speed

        # color sensor used by follow_line()
        self.cs = None


# vrep simulation init
# v0.1 - clientID will be global
#
# TODO: 1. move (maybe) to a function and call here...
#       2. think of possible user specific clientID (in kwargs?)
#       3. rewrite self.kwargs['arg'] to self.kwargs.get('arg')
if CONNECT:
    vrep.simxFinish(-1)
    clientID = connection()
