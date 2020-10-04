# -----------------------------------------------------------------------------
# Copyright (c) 2020 Lukasz Machura <lukasz.machura@us.edu.pl>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------

import sys
import math
import select
import time
import _thread

# python3 uses collections
# micropython uses ucollections
try:
    from collections import OrderedDict
except ImportError:
    from ucollections import OrderedDict

from logging import getLogger
from os.path import abspath
from ev3dev2 import get_current_platform, Device, list_device_names, DeviceNotDefined, ThreadNotRunning
from ev3dev2.stopwatch import StopWatch

# OUTPUT ports have platform specific values that we must import
platform = get_current_platform()

if platform == 'sim':
    from ev3dev2._platform.sim import OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D  # noqa: F401

else:
    raise Exception("Unsupported platform '%s'" % platform)

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')

log = getLogger(__name__)

# The number of milliseconds we wait for the state of a motor to
# update to 'running' in the "on_for_XYZ" methods of the Motor class
WAIT_RUNNING_TIMEOUT = 100


class SpeedInvalid(ValueError):
    pass


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
    def __init__(self, percent, desc=None, max_speed=900):
        if percent < -max_speed or percent > max_speed:
            raise SpeedInvalid("invalid percentage {}, must be between -{} and {} (inclusive)".format(percent, max_speed, max_speed))
        self.percent = percent
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + str(self.percent) + "%"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedPercent(self.percent * other)

    def to_native_units(self, motor):
        """
        Return this SpeedPercent in native motor units
        """
        return self.percent / max_speed * motor.max_speed


class SpeedNativeUnits(SpeedValue):
    """
    Speed in tacho counts per second.
    """
    def __init__(self, native_counts, desc=None):
        self.native_counts = native_counts
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + "{:.2f}".format(self.native_counts) + " counts/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedNativeUnits(self.native_counts * other)

    def to_native_units(self, motor=None):
        """
        Return this SpeedNativeUnits as a number
        """
        if self.native_counts > motor.max_speed:
            raise SpeedInvalid("invalid native-units: {} max speed {}, {} was requested".format(
                motor, motor.max_speed, self.native_counts))
        return self.native_counts


class SpeedRPS(SpeedValue):
    """
    Speed in rotations-per-second.
    """
    def __init__(self, rotations_per_second, desc=None):
        self.rotations_per_second = rotations_per_second
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + str(self.rotations_per_second) + " rot/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedRPS(self.rotations_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired rotations-per-second
        """
        if abs(self.rotations_per_second) > motor.max_rps:
            raise SpeedInvalid("invalid rotations-per-second: {} max RPS is {}, {} was requested".format(
                motor, motor.max_rps, self.rotations_per_second))
        return self.rotations_per_second / motor.max_rps * motor.max_speed


class SpeedRPM(SpeedValue):
    """
    Speed in rotations-per-minute.
    """
    def __init__(self, rotations_per_minute, desc=None):
        self.rotations_per_minute = rotations_per_minute
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + str(self.rotations_per_minute) + " rot/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedRPM(self.rotations_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired rotations-per-minute
        """
        if abs(self.rotations_per_minute) > motor.max_rpm:
            raise SpeedInvalid("invalid rotations-per-minute: {} max RPM is {}, {} was requested".format(
                motor, motor.max_rpm, self.rotations_per_minute))
        return self.rotations_per_minute / motor.max_rpm * motor.max_speed


class SpeedDPS(SpeedValue):
    """
    Speed in degrees-per-second.
    """
    def __init__(self, degrees_per_second, desc=None):
        self.degrees_per_second = degrees_per_second
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + str(self.degrees_per_second) + " deg/sec"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedDPS(self.degrees_per_second * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired degrees-per-second
        """
        if abs(self.degrees_per_second) > motor.max_dps:
            raise SpeedInvalid("invalid degrees-per-second: {} max DPS is {}, {} was requested".format(
                motor, motor.max_dps, self.degrees_per_second))
        return self.degrees_per_second / motor.max_dps * motor.max_speed


class SpeedDPM(SpeedValue):
    """
    Speed in degrees-per-minute.
    """
    def __init__(self, degrees_per_minute, desc=None):
        self.degrees_per_minute = degrees_per_minute
        self.desc = desc

    def __str__(self):
        return "{} ".format(self.desc) if self.desc else "" + str(self.degrees_per_minute) + " deg/min"

    def __mul__(self, other):
        assert isinstance(other, (float, int)), "{} can only be multiplied by an int or float".format(self)
        return SpeedDPM(self.degrees_per_minute * other)

    def to_native_units(self, motor):
        """
        Return the native speed measurement required to achieve desired degrees-per-minute
        """
        if abs(self.degrees_per_minute) > motor.max_dpm:
            raise SpeedInvalid("invalid degrees-per-minute: {} max DPM is {}, {} was requested".format(
                motor, motor.max_dpm, self.degrees_per_minute))
        return self.degrees_per_minute / motor.max_dpm * motor.max_speed


def speed_to_speedvalue(speed, desc=None):
    """
    If ``speed`` is not a ``SpeedValue`` object, treat it as a percentage.
    Returns a ``SpeedValue`` object.
    """
    if isinstance(speed, SpeedValue):
        return speed
    else:
        return SpeedPercent(speed, desc)


class Motor(Device):
    """
    The motor class provides a uniform interface for using motors with
    positional and directional feedback such as the EV3 and NXT motors.
    This feedback allows for precise control of the motors. This is the
    most common type of motor, so we just call it ``motor``.
    """

    SYSTEM_CLASS_NAME = 'tacho-motor'
    SYSTEM_DEVICE_NAME_CONVENTION = '*'

    __slots__ = [
        '_address',
        '_command',
        '_commands',
        '_count_per_rot',
        '_count_per_m',
        '_driver_name',
        '_duty_cycle',
        '_duty_cycle_sp',
        '_full_travel_count',
        '_polarity',
        '_position',
        '_position_p',
        '_position_i',
        '_position_d',
        '_position_sp',
        '_max_speed',
        '_speed',
        '_speed_sp',
        '_ramp_up_sp',
        '_ramp_down_sp',
        '_speed_p',
        '_speed_i',
        '_speed_d',
        '_state',
        '_stop_action',
        '_stop_actions',
        '_time_sp',
        '_poll',
        'max_rps',
        'max_rpm',
        'max_dps',
        'max_dpm',
    ]

    #: Run the motor until another command is sent.
    COMMAND_RUN_FOREVER = 'run-forever'

    #: Run to an absolute position specified by ``position_sp`` and then
    #: stop using the action specified in ``stop_action``.
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'

    #: Run to a position relative to the current ``position`` value.
    #: The new position will be current ``position`` + ``position_sp``.
    #: When the new position is reached, the motor will stop using
    #: the action specified by ``stop_action``.
    COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'

    #: Run the motor for the amount of time specified in ``time_sp``
    #: and then stop the motor using the action specified by ``stop_action``.
    COMMAND_RUN_TIMED = 'run-timed'

    #: Run the motor at the duty cycle specified by ``duty_cycle_sp``.
    #: Unlike other run commands, changing ``duty_cycle_sp`` while running *will*
    #: take effect immediately.
    COMMAND_RUN_DIRECT = 'run-direct'

    #: Stop any of the run commands before they are complete using the
    #: action specified by ``stop_action``.
    COMMAND_STOP = 'stop'

    #: Reset all of the motor parameter attributes to their default value.
    #: This will also have the effect of stopping the motor.
    COMMAND_RESET = 'reset'

    #: Sets the normal polarity of the rotary encoder.
    ENCODER_POLARITY_NORMAL = 'normal'

    #: Sets the inversed polarity of the rotary encoder.
    ENCODER_POLARITY_INVERSED = 'inversed'

    #: With ``normal`` polarity, a positive duty cycle will
    #: cause the motor to rotate clockwise.
    POLARITY_NORMAL = 'normal'

    #: With ``inversed`` polarity, a positive duty cycle will
    #: cause the motor to rotate counter-clockwise.
    POLARITY_INVERSED = 'inversed'

    #: Power is being sent to the motor.
    STATE_RUNNING = 'running'

    #: The motor is ramping up or down and has not yet reached a constant output level.
    STATE_RAMPING = 'ramping'

    #: The motor is not turning, but rather attempting to hold a fixed position.
    STATE_HOLDING = 'holding'

    #: The motor is turning, but cannot reach its ``speed_sp``.
    STATE_OVERLOADED = 'overloaded'

    #: The motor is not turning when it should be.
    STATE_STALLED = 'stalled'

    #: Power will be removed from the motor and it will freely coast to a stop.
    STOP_ACTION_COAST = 'coast'

    #: Power will be removed from the motor and a passive electrical load will
    #: be placed on the motor. This is usually done by shorting the motor terminals
    #: together. This load will absorb the energy from the rotation of the motors and
    #: cause the motor to stop more quickly than coasting.
    STOP_ACTION_BRAKE = 'brake'

    #: Does not remove power from the motor. Instead it actively try to hold the motor
    #: at the current position. If an external force tries to turn the motor, the motor
    #: will ``push back`` to maintain its position.
    STOP_ACTION_HOLD = 'hold'

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        if platform in ('brickpi', 'brickpi3') and type(self).__name__ != 'Motor' and not isinstance(self, LargeMotor):
            raise Exception("{} is unaware of different motor types, use LargeMotor instead".format(platform))

        if address is not None:
            kwargs['address'] = address
        super(Motor, self).__init__(self.SYSTEM_CLASS_NAME, name_pattern, name_exact, **kwargs)

        self._address = None
        self._command = None
        self._commands = None
        self._count_per_rot = None
        self._count_per_m = None
        self._driver_name = None
        self._duty_cycle = None
        self._duty_cycle_sp = None
        self._full_travel_count = None
        self._polarity = None
        self._position = None
        self._position_p = None
        self._position_i = None
        self._position_d = None
        self._position_sp = None
        self._max_speed = None
        self._speed = None
        self._speed_sp = None
        self._ramp_up_sp = None
        self._ramp_down_sp = None
        self._speed_p = None
        self._speed_i = None
        self._speed_d = None
        self._state = None
        self._stop_action = None
        self._stop_actions = None
        self._time_sp = None
        self._poll = None
        self.max_rps = float(self.max_speed / self.count_per_rot)
        self.max_rpm = self.max_rps * 60
        self.max_dps = self.max_rps * 360
        self.max_dpm = self.max_rpm * 360

    @property
    def address(self):
        """
        Returns the name of the port that this motor is connected to.
        """
        self._address, value = self.get_attr_string(self._address, 'address')
        return value

    @property
    def command(self):
        """
        Sends a command to the motor controller. See ``commands`` for a list of
        possible values.
        """
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self._command = self.set_attr_string(self._command, 'command', value)

    @property
    def commands(self):
        """
        Returns a list of commands that are supported by the motor
        controller. Possible values are ``run-forever``, ``run-to-abs-pos``, ``run-to-rel-pos``,
        ``run-timed``, ``run-direct``, ``stop`` and ``reset``. Not all commands may be supported.

        - ``run-forever`` will cause the motor to run until another command is sent.
        - ``run-to-abs-pos`` will run to an absolute position specified by ``position_sp``
          and then stop using the action specified in ``stop_action``.
        - ``run-to-rel-pos`` will run to a position relative to the current ``position`` value.
          The new position will be current ``position`` + ``position_sp``. When the new
          position is reached, the motor will stop using the action specified by ``stop_action``.
        - ``run-timed`` will run the motor for the amount of time specified in ``time_sp``
          and then stop the motor using the action specified by ``stop_action``.
        - ``run-direct`` will run the motor at the duty cycle specified by ``duty_cycle_sp``.
          Unlike other run commands, changing ``duty_cycle_sp`` while running *will*
          take effect immediately.
        - ``stop`` will stop any of the run commands before they are complete using the
          action specified by ``stop_action``.
        - ``reset`` will reset all of the motor parameter attributes to their default value.
          This will also have the effect of stopping the motor.
        """
        (self._commands, value) = self.get_cached_attr_set(self._commands, 'commands')
        return value

    @property
    def count_per_rot(self):
        """
        Returns the number of tacho counts in one rotation of the motor. Tacho counts
        are used by the position and speed attributes, so you can use this value
        to convert rotations or degrees to tacho counts. (rotation motors only)
        """
        (self._count_per_rot, value) = self.get_cached_attr_int(self._count_per_rot, 'count_per_rot')
        return value

    @property
    def count_per_m(self):
        """
        Returns the number of tacho counts in one meter of travel of the motor. Tacho
        counts are used by the position and speed attributes, so you can use this
        value to convert from distance to tacho counts. (linear motors only)
        """
        (self._count_per_m, value) = self.get_cached_attr_int(self._count_per_m, 'count_per_m')
        return value

    @property
    def driver_name(self):
        """
        Returns the name of the driver that provides this tacho motor device.
        """
        (self._driver_name, value) = self.get_cached_attr_string(self._driver_name, 'driver_name')
        return value

    @property
    def duty_cycle(self):
        """
        Returns the current duty cycle of the motor. Units are percent. Values
        are -100 to 100.
        """
        self._duty_cycle, value = self.get_attr_int(self._duty_cycle, 'duty_cycle')
        return value

    @property
    def duty_cycle_sp(self):
        """
        Writing sets the duty cycle setpoint. Reading returns the current value.
        Units are in percent. Valid values are -100 to 100. A negative value causes
        the motor to rotate in reverse.
        """
        self._duty_cycle_sp, value = self.get_attr_int(self._duty_cycle_sp, 'duty_cycle_sp')
        return value

    @duty_cycle_sp.setter
    def duty_cycle_sp(self, value):
        self._duty_cycle_sp = self.set_attr_int(self._duty_cycle_sp, 'duty_cycle_sp', value)

    @property
    def full_travel_count(self):
        """
        Returns the number of tacho counts in the full travel of the motor. When
        combined with the ``count_per_m`` atribute, you can use this value to
        calculate the maximum travel distance of the motor. (linear motors only)
        """
        (self._full_travel_count, value) = self.get_cached_attr_int(self._full_travel_count, 'full_travel_count')
        return value

    @property
    def polarity(self):
        """
        Sets the polarity of the motor. With ``normal`` polarity, a positive duty
        cycle will cause the motor to rotate clockwise. With ``inversed`` polarity,
        a positive duty cycle will cause the motor to rotate counter-clockwise.
        Valid values are ``normal`` and ``inversed``.
        """
        self._polarity, value = self.get_attr_string(self._polarity, 'polarity')
        return value

    @polarity.setter
    def polarity(self, value):
        self._polarity = self.set_attr_string(self._polarity, 'polarity', value)

    @property
    def position(self):
        """
        Returns the current position of the motor in pulses of the rotary
        encoder. When the motor rotates clockwise, the position will increase.
        Likewise, rotating counter-clockwise causes the position to decrease.
        Writing will set the position to that value.
        """
        self._position, value = self.get_attr_int(self._position, 'position')
        return value

    @position.setter
    def position(self, value):
        self._position = self.set_attr_int(self._position, 'position', value)

    @property
    def position_p(self):
        """
        The proportional constant for the position PID.
        """
        self._position_p, value = self.get_attr_int(self._position_p, 'hold_pid/Kp')
        return value

    @position_p.setter
    def position_p(self, value):
        self._position_p = self.set_attr_int(self._position_p, 'hold_pid/Kp', value)

    @property
    def position_i(self):
        """
        The integral constant for the position PID.
        """
        self._position_i, value = self.get_attr_int(self._position_i, 'hold_pid/Ki')
        return value

    @position_i.setter
    def position_i(self, value):
        self._position_i = self.set_attr_int(self._position_i, 'hold_pid/Ki', value)

    @property
    def position_d(self):
        """
        The derivative constant for the position PID.
        """
        self._position_d, value = self.get_attr_int(self._position_d, 'hold_pid/Kd')
        return value

    @position_d.setter
    def position_d(self, value):
        self._position_d = self.set_attr_int(self._position_d, 'hold_pid/Kd', value)

    @property
    def position_sp(self):
        """
        Writing specifies the target position for the ``run-to-abs-pos`` and ``run-to-rel-pos``
        commands. Reading returns the current value. Units are in tacho counts. You
        can use the value returned by ``count_per_rot`` to convert tacho counts to/from
        rotations or degrees.
        """
        self._position_sp, value = self.get_attr_int(self._position_sp, 'position_sp')
        return value

    @position_sp.setter
    def position_sp(self, value):
        self._position_sp = self.set_attr_int(self._position_sp, 'position_sp', value)

    @property
    def max_speed(self):
        """
        Returns the maximum value that is accepted by the ``speed_sp`` attribute. This
        may be slightly different than the maximum speed that a particular motor can
        reach - it's the maximum theoretical speed.
        """
        (self._max_speed, value) = self.get_cached_attr_int(self._max_speed, 'max_speed')
        return value

    @property
    def speed(self):
        """
        Returns the current motor speed in tacho counts per second. Note, this is
        not necessarily degrees (although it is for LEGO motors). Use the ``count_per_rot``
        attribute to convert this value to RPM or deg/sec.
        """
        self._speed, value = self.get_attr_int(self._speed, 'speed')
        return value

    @property
    def speed_sp(self):
        """
        Writing sets the target speed in tacho counts per second used for all ``run-*``
        commands except ``run-direct``. Reading returns the current value. A negative
        value causes the motor to rotate in reverse with the exception of ``run-to-*-pos``
        commands where the sign is ignored. Use the ``count_per_rot`` attribute to convert
        RPM or deg/sec to tacho counts per second. Use the ``count_per_m`` attribute to
        convert m/s to tacho counts per second.
        """
        self._speed_sp, value = self.get_attr_int(self._speed_sp, 'speed_sp')
        return value

    @speed_sp.setter
    def speed_sp(self, value):
        self._speed_sp = self.set_attr_int(self._speed_sp, 'speed_sp', value)

    @property
    def ramp_up_sp(self):
        """
        Writing sets the ramp up setpoint. Reading returns the current value. Units
        are in milliseconds and must be positive. When set to a non-zero value, the
        motor speed will increase from 0 to 100% of ``max_speed`` over the span of this
        setpoint. The actual ramp time is the ratio of the difference between the
        ``speed_sp`` and the current ``speed`` and max_speed multiplied by ``ramp_up_sp``.
        """
        self._ramp_up_sp, value = self.get_attr_int(self._ramp_up_sp, 'ramp_up_sp')
        return value

    @ramp_up_sp.setter
    def ramp_up_sp(self, value):
        self._ramp_up_sp = self.set_attr_int(self._ramp_up_sp, 'ramp_up_sp', value)

    @property
    def ramp_down_sp(self):
        """
        Writing sets the ramp down setpoint. Reading returns the current value. Units
        are in milliseconds and must be positive. When set to a non-zero value, the
        motor speed will decrease from 0 to 100% of ``max_speed`` over the span of this
        setpoint. The actual ramp time is the ratio of the difference between the
        ``speed_sp`` and the current ``speed`` and max_speed multiplied by ``ramp_down_sp``.
        """
        self._ramp_down_sp, value = self.get_attr_int(self._ramp_down_sp, 'ramp_down_sp')
        return value

    @ramp_down_sp.setter
    def ramp_down_sp(self, value):
        self._ramp_down_sp = self.set_attr_int(self._ramp_down_sp, 'ramp_down_sp', value)

    @property
    def speed_p(self):
        """
        The proportional constant for the speed regulation PID.
        """
        self._speed_p, value = self.get_attr_int(self._speed_p, 'speed_pid/Kp')
        return value

    @speed_p.setter
    def speed_p(self, value):
        self._speed_p = self.set_attr_int(self._speed_p, 'speed_pid/Kp', value)

    @property
    def speed_i(self):
        """
        The integral constant for the speed regulation PID.
        """
        self._speed_i, value = self.get_attr_int(self._speed_i, 'speed_pid/Ki')
        return value

    @speed_i.setter
    def speed_i(self, value):
        self._speed_i = self.set_attr_int(self._speed_i, 'speed_pid/Ki', value)

    @property
    def speed_d(self):
        """
        The derivative constant for the speed regulation PID.
        """
        self._speed_d, value = self.get_attr_int(self._speed_d, 'speed_pid/Kd')
        return value

    @speed_d.setter
    def speed_d(self, value):
        self._speed_d = self.set_attr_int(self._speed_d, 'speed_pid/Kd', value)

    @property
    def state(self):
        """
        Reading returns a list of state flags. Possible flags are
        ``running``, ``ramping``, ``holding``, ``overloaded`` and ``stalled``.
        """
        self._state, value = self.get_attr_set(self._state, 'state')
        return value

    @property
    def stop_action(self):
        """
        Reading returns the current stop action. Writing sets the stop action.
        The value determines the motors behavior when ``command`` is set to ``stop``.
        Also, it determines the motors behavior when a run command completes. See
        ``stop_actions`` for a list of possible values.
        """
        self._stop_action, value = self.get_attr_string(self._stop_action, 'stop_action')
        return value

    @stop_action.setter
    def stop_action(self, value):
        self._stop_action = self.set_attr_string(self._stop_action, 'stop_action', value)

    @property
    def stop_actions(self):
        """
        Returns a list of stop actions supported by the motor controller.
        Possible values are ``coast``, ``brake`` and ``hold``. ``coast`` means that power will
        be removed from the motor and it will freely coast to a stop. ``brake`` means
        that power will be removed from the motor and a passive electrical load will
        be placed on the motor. This is usually done by shorting the motor terminals
        together. This load will absorb the energy from the rotation of the motors and
        cause the motor to stop more quickly than coasting. ``hold`` does not remove
        power from the motor. Instead it actively tries to hold the motor at the current
        position. If an external force tries to turn the motor, the motor will 'push
        back' to maintain its position.
        """
        (self._stop_actions, value) = self.get_cached_attr_set(self._stop_actions, 'stop_actions')
        return value

    @property
    def time_sp(self):
        """
        Writing specifies the amount of time the motor will run when using the
        ``run-timed`` command. Reading returns the current value. Units are in
        milliseconds.
        """
        self._time_sp, value = self.get_attr_int(self._time_sp, 'time_sp')
        return value

    @time_sp.setter
    def time_sp(self, value):
        self._time_sp = self.set_attr_int(self._time_sp, 'time_sp', value)

    def run_forever(self, **kwargs):
        """
        Run the motor until another command is sent.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_FOREVER

    def run_to_abs_pos(self, **kwargs):
        """
        Run to an absolute position specified by ``position_sp`` and then
        stop using the action specified in ``stop_action``.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TO_ABS_POS

    def run_to_rel_pos(self, **kwargs):
        """
        Run to a position relative to the current ``position`` value.
        The new position will be current ``position`` + ``position_sp``.
        When the new position is reached, the motor will stop using
        the action specified by ``stop_action``.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TO_REL_POS

    def run_timed(self, **kwargs):
        """
        Run the motor for the amount of time specified in ``time_sp``
        and then stop the motor using the action specified by ``stop_action``.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_TIMED

    def run_direct(self, **kwargs):
        """
        Run the motor at the duty cycle specified by ``duty_cycle_sp``.
        Unlike other run commands, changing ``duty_cycle_sp`` while running *will*
        take effect immediately.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RUN_DIRECT

    def stop(self, **kwargs):
        """
        Stop any of the run commands before they are complete using the
        action specified by ``stop_action``.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_STOP

    def reset(self, **kwargs):
        """
        Reset all of the motor parameter attributes to their default value.
        This will also have the effect of stopping the motor.
        """
        for key in kwargs:
            setattr(self, key, kwargs[key])
        self.command = self.COMMAND_RESET

    @property
    def is_running(self):
        """
        Power is being sent to the motor.
        """
        return self.STATE_RUNNING in self.state

    @property
    def is_ramping(self):
        """
        The motor is ramping up or down and has not yet reached a constant output level.
        """
        return self.STATE_RAMPING in self.state

    @property
    def is_holding(self):
        """
        The motor is not turning, but rather attempting to hold a fixed position.
        """
        return self.STATE_HOLDING in self.state

    @property
    def is_overloaded(self):
        """
        The motor is turning, but cannot reach its ``speed_sp``.
        """
        return self.STATE_OVERLOADED in self.state

    @property
    def is_stalled(self):
        """
        The motor is not turning when it should be.
        """
        return self.STATE_STALLED in self.state

    def wait(self, cond, timeout=None):
        """
        Blocks until ``cond(self.state)`` is ``True``.  The condition is
        checked when there is an I/O event related to the ``state`` attribute.
        Exits early when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.
        """

        tic = time.time()

        if self._poll is None:
            if self._state is None:
                self._state = self._attribute_file_open('state')
            self._poll = select.poll()
            self._poll.register(self._state, select.POLLPRI)

        # Set poll timeout to something small. For more details, see
        # https://github.com/ev3dev/ev3dev-lang-python/issues/583
        if timeout:
            poll_tm = min(timeout, 100)
        else:
            poll_tm = 100

        while True:
            # This check is now done every poll_tm even if poll has nothing to report:
            if cond(self.state):
                return True

            self._poll.poll(poll_tm)

            if timeout is not None and time.time() >= tic + timeout / 1000:
                # Final check when user timeout is reached
                return cond(self.state)

    def wait_until_not_moving(self, timeout=None):
        """
        Blocks until ``running`` is not in ``self.state`` or ``stalled`` is in
        ``self.state``.  The condition is checked when there is an I/O event
        related to the ``state`` attribute.  Exits early when ``timeout``
        (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_until_not_moving()
        """
        return self.wait(lambda state: self.STATE_RUNNING not in state or self.STATE_STALLED in state, timeout)

    def wait_until(self, s, timeout=None):
        """
        Blocks until ``s`` is in ``self.state``.  The condition is checked when
        there is an I/O event related to the ``state`` attribute.  Exits early
        when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_until('stalled')
        """
        return self.wait(lambda state: s in state, timeout)

    def wait_while(self, s, timeout=None):
        """
        Blocks until ``s`` is not in ``self.state``.  The condition is checked
        when there is an I/O event related to the ``state`` attribute.  Exits
        early when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_while('running')
        """
        return self.wait(lambda state: s not in state, timeout)

    def _speed_native_units(self, speed, label=None):
        speed = speed_to_speedvalue(speed, label)
        return speed.to_native_units(self)

    def _set_rel_position_degrees_and_speed_sp(self, degrees, speed):
        degrees = degrees if speed >= 0 else -degrees
        speed = abs(speed)

        position_delta = int(round((degrees * self.count_per_rot) / 360))
        speed_sp = int(round(speed))

        self.position_sp = position_delta
        self.speed_sp = speed_sp

    def _set_brake(self, brake):
        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

    def on_for_rotations(self, speed, rotations, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``rotations``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        speed_sp = self._speed_native_units(speed)
        self._set_rel_position_degrees_and_speed_sp(rotations * 360, speed_sp)
        self._set_brake(brake)
        self.run_to_rel_pos()

        if block:
            self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on_for_degrees(self, speed, degrees, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``degrees``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        speed_sp = self._speed_native_units(speed)
        self._set_rel_position_degrees_and_speed_sp(degrees, speed_sp)
        self._set_brake(brake)
        self.run_to_rel_pos()

        if block:
            self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on_to_position(self, speed, position, brake=True, block=True):
        """
        Rotate the motor at ``speed`` to ``position``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        speed = self._speed_native_units(speed)
        self.speed_sp = int(round(speed))
        self.position_sp = position
        self._set_brake(brake)
        self.run_to_abs_pos()

        if block:
            self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on_for_seconds(self, speed, seconds, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``seconds``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """

        if seconds < 0:
            raise ValueError("seconds is negative ({})".format(seconds))

        speed = self._speed_native_units(speed)
        self.speed_sp = int(round(speed))
        self.time_sp = int(seconds * 1000)
        self._set_brake(brake)
        self.run_timed()

        if block:
            self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on(self, speed, brake=True, block=False):
        """
        Rotate the motor at ``speed`` for forever

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.

        Note that ``block`` is False by default, this is different from the
        other ``on_for_XYZ`` methods.
        """
        speed = self._speed_native_units(speed)
        self.speed_sp = int(round(speed))
        self._set_brake(brake)
        self.run_forever()

        if block:
            self.wait_until('running', timeout=WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def off(self, brake=True):
        self._set_brake(brake)
        self.stop()

    @property
    def rotations(self):
        return float(self.position / self.count_per_rot)

    @property
    def degrees(self):
        return self.rotations * 360


class LargeMotor(Motor):
    """
    EV3/NXT large servo motor.

    Same as :class:`Motor`, except it will only successfully initialize if it finds a "large" motor.
    """

    SYSTEM_CLASS_NAME = Motor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = '*'
    __slots__ = []

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):

        super(LargeMotor, self).__init__(address,
                                         name_pattern,
                                         name_exact,
                                         driver_name=['lego-ev3-l-motor', 'lego-nxt-motor'],
                                         **kwargs)

# follow gyro angle classes
class FollowGyroAngleErrorTooFast(Exception):
    """
    Raised when a gyro following robot has been asked to follow
    an angle at an unrealistic speed
    """
    pass


# line follower classes
class LineFollowErrorLostLine(Exception):
    """
    Raised when a line following robot has lost the line
    """
    pass


class LineFollowErrorTooFast(Exception):
    """
    Raised when a line following robot has been asked to follow
    a line at an unrealistic speed
    """
    pass


# line follower functions
def follow_for_forever(tank):
    """
    ``tank``: the MoveTank object that is following a line
    """
    return True


def follow_for_ms(tank, ms):
    """
    ``tank``: the MoveTank object that is following a line
    ``ms`` : the number of milliseconds to follow the line
    """
    if not hasattr(tank, 'stopwatch') or tank.stopwatch is None:
        tank.stopwatch = StopWatch()
        tank.stopwatch.start()

    if tank.stopwatch.value_ms >= ms:
        tank.stopwatch = None
        return False
    else:
        return True


##########
### LM ###
##########

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

    def on(self, speed):
        """
        motor (int): motor number
            1: 'B' or 'left'
            2: 'C' or 'right'
            3: both
        speed (int/float): speed of motor, in [-MAX_SPEED, MAX_SPEED]
        """
        speed = SpeedPercent(speed)

        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'On',
            [3, int(speed)], [speed], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode
