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

import sys
import os
import stat
import time
import vrep
from collections import OrderedDict
from ev3dev2.stopwatch import StopWatch
from time import sleep
from ev3dev2 import *
from ev3dev2._platform.sim import LEDS, LED_GROUPS, LED_COLORS, LED_DEFAULT_COLOR

# if sys.version_info < (3, 4):
#     raise SystemError('Must be using Python 3.4 or higher')

# Import the LED settings, this is platform specific
# platform = get_current_platform()

class Led(Device):
    pass


class Leds(Led):
    def __init__(self, desc='Leds', **kwargs):
        self.kwargs = kwargs

        self.leds = LEDS
        self.led_groups = LED_GROUPS
        self.led_colors = LED_COLORS
        self.animate_thread_id = None
        self.animate_thread_stop = False

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
        # print('ID', clientID, self.clientID, self.kwargs['clientID'])

    def __str__(self):
        return self.__class__.__name__

    def set_color(self, group, color, pct=1):
        """
        Sets brightness of LEDs in the given group to the values specified in
        color tuple. When percentage is specified, brightness of each LED is
        reduced proportionally.

        Args::

            group (str): 'LEFT', 'RIGHT', 'BOTH'

            color (str): 'RED', 'GREEN', 'AMBER', 'GREY'
            color (tuple/list): rgb

        Example::

            my_leds = Leds()
            my_leds.set_color('LEFT', 'AMBER')
            my_leds.set_color('LEFT', (0.89, 0.54, 0.23))

        """
        # If this is a platform without LEDs there is nothing to do
        if not self.leds:
            # print('not self.leds')
            return

        color_tuple = self.led_colors['GREY']
        if isinstance(color, str):
            color = color.upper()
            assert color in self.led_colors, \
                "%s is an invalid LED color, valid choices are %s" % \
                (color, ', '.join(self.led_colors.keys()))
            color_tuple = self.led_colors[color]

        elif isinstance(color, (list, tuple)):
            assert all([0 <= i <= 1 for i in color]), \
                f"provide tuple with numbers between 0 and 1"
            color_tuple = color

        else:
            raise ValueError("color should be str or rgb tuple/list")

        group = group.upper()
        assert group in self.led_groups, \
            "%s is an invalid LED group, valid choices are %s" % \
            (group, ', '.join(self.led_groups.keys()))

        if 'L' in group:
            fun = 'StatusLightLeft'
        elif 'R' in group:
            fun = 'StatusLightRight'
        elif 'B' in group:  # that is silly sim hack
            fun = 'StatusLight'
        else:
            raise ValueError

        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            fun,
            [0], color_tuple, [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return returnCode

    def set(self, group, **kwargs):
        pass

    def all_off(self):
        """
        Turn all LEDs off
        """
        self.set_color('BOTH', 'GREY')

    def reset(self):
        """
        Put all LEDs back to their default color
        """

        if not self.leds:
            return

        # self.animate_stop()

        for group in self.led_groups:
            self.set_color(group, LED_DEFAULT_COLOR)

    def animate_stop(self):
        """
        Signal the current animation thread to exit and wait for it to exit
        """
        pass

    def animate_police_lights(self,
                              color1,
                              color2,
                              group1='LEFT',
                              group2='RIGHT',
                              sleeptime=0.5,
                              duration=5,
                              block=True):
        """
        Cycle the ``group1`` and ``group2`` LEDs between ``color1`` and ``color2``
        to give the effect of police lights.  Alternate the ``group1`` and ``group2``
        LEDs every ``sleeptime`` seconds.

        Animate for ``duration`` seconds.  If ``duration`` is None animate for forever.

        Example:

        .. code-block:: python

            from ev3dev2.led import Leds
            leds = Leds()
            leds.animate_police_lights('RED', 'GREEN', sleeptime=0.75, duration=10)
        """
        pass

    def animate_flash(self, color, groups=('LEFT', 'RIGHT'), sleeptime=0.5, duration=5, block=True):
        """
        Turn all LEDs in ``groups`` off/on to ``color`` every ``sleeptime`` seconds

        Animate for ``duration`` seconds.  If ``duration`` is None animate for forever.

        Example:

        .. code-block:: python

            from ev3dev2.led import Leds
            leds = Leds()
            leds.animate_flash('AMBER', sleeptime=0.75, duration=10)
        """
        pass

    def animate_cycle(self, colors, groups=('LEFT', 'RIGHT'), sleeptime=0.5, duration=5, block=True):
        """
        Cycle ``groups`` LEDs through ``colors``. Do this in a loop where
        we display each color for ``sleeptime`` seconds.

        Animate for ``duration`` seconds.  If ``duration`` is None animate for forever.

        Example:

        .. code-block:: python

            from ev3dev2.led import Leds
            leds = Leds()
            leds.animate_cycle(('RED', 'GREEN', 'AMBER'))
        """
        pass

    def animate_rainbow(self, group1='LEFT', group2='RIGHT', increment_by=0.1, sleeptime=0.1, duration=5, block=True):
        """
        Gradually fade from one color to the next

        Animate for ``duration`` seconds.  If ``duration`` is None animate for forever.

        Example:

        .. code-block:: python

            from ev3dev2.led import Leds
            leds = Leds()
            leds.animate_rainbow()
        """
        pass
