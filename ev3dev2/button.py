# -----------------------------------------------------------------------------
# Copyright (c) 2015 Ralph Hempel <rhempel@hempeldesigngroup.com>
# Copyright (c) 2015 Anton Vanhoucke <antonvh@gmail.com>
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
# Copyright (c) 2015 Eric Pascual <eric@pobot.org>
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
import vrep
from ev3dev2 import *
from ._platform.sim import BUTTONS_FILENAME, EVDEV_DEVICE_NAME
from .stopwatch import StopWatch
import time


class Button(object):
    """
    EV3 Buttons
    """

    _buttons = {
        'up': {
            'name': BUTTONS_FILENAME,
            'value': 5
        },
        'down': {
            'name': BUTTONS_FILENAME,
            'value': 1
        },
        'left': {
            'name': BUTTONS_FILENAME,
            'value': 3
        },
        'right': {
            'name': BUTTONS_FILENAME,
            'value': 2
        },
        'enter': {
            'name': BUTTONS_FILENAME,
            'value': 4
        },
        'backspace': {
            'name': BUTTONS_FILENAME,
            'value': 6
        },
    }
    evdev_device_name = 'ev3sim'

    on_up = None
    on_down = None
    on_left = None
    on_right = None
    on_enter = None
    on_backspace = None

    def __init__(self, desc='Button', **kwargs):
        self.kwargs = kwargs

        self.platform = "ev3sim"
        self.desc = desc

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)

        self.connected = True
        self.buttons_pressed = 0

    def __str__(self):
        return self.desc

    @property
    def get_pressed_button_number(self):
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'ButtonPressed',
            [], [], [], bytearray(),
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        self.buttons_pressed = outInts[0]
        return outInts[0]

    def any(self):
        """
        Checks if any button is pressed.
        """
        return self.get_pressed_button_number != 0

    def _wait(self, wait_for_button_press, wait_for_button_release, timeout_ms):
        stopwatch = StopWatch()
        stopwatch.start()
        pressed = False

    def wait_for_pressed(self, buttons, timeout_ms=None):
        """
        Wait for ``buttons`` to be pressed down.
        """
        # return self._wait(buttons, [], timeout_ms)
        if isinstance(buttons, str):
            buttons = __class__._buttons[buttons]['value']

        while self.get_pressed_button_number != buttons:
            time.sleep(0.1)
            
        return True

    def wait_for_released(self, buttons, timeout_ms=None):
        """
        Wait for ``buttons`` to be released.
        """
        return self._wait([], buttons, timeout_ms)

    def wait_for_bump(self, buttons, timeout_ms=None):
        """
        Wait for ``buttons`` to be pressed down and then released.
        Both actions must happen within ``timeout_ms``.
        """
        stopwatch = StopWatch()
        stopwatch.start()

        if self.wait_for_pressed(buttons, timeout_ms):
            if timeout_ms is not None:
                timeout_ms -= stopwatch.value_ms
            return self.wait_for_released(buttons, timeout_ms)

        return False

    @property
    def up(self):
        """
        Check if ``up`` button is pressed.
        """
        return 'up' in self.buttons_pressed

    @property
    def down(self):
        """
        Check if ``down`` button is pressed.
        """
        return 'down' in self.buttons_pressed

    @property
    def left(self):
        """
        Check if ``left`` button is pressed.
        """
        return 'left' in self.buttons_pressed

    @property
    def right(self):
        """
        Check if ``right`` button is pressed.
        """
        return 'right' in self.buttons_pressed

    @property
    def enter(self):
        """
        Check if ``enter`` button is pressed.
        """
        return 'enter' in self.buttons_pressed

    @property
    def backspace(self):
        """
        Check if ``backspace`` button is pressed.
        """
        return 'backspace' in self.buttons_pressed

    # @property
    # def buttons_pressed(self):
    #     """
    #     Returns list of names of pressed buttons.
    #     One button in fact, as you cannot press 2 on display.
    #     """
    #
    #     return self.buttons_pressed
