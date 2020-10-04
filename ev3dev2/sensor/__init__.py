# -----------------------------------------------------------------------------
# Copyright (c) 2020
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
from os.path import abspath
from struct import unpack
from ev3dev2 import get_current_platform, Device

# INPUT ports have platform specific values that we must import
platform = get_current_platform()

if platform == 'sim':
    from ev3dev2._platform.sim import INPUT_1, INPUT_2, INPUT_3, INPUT_4  #
else:
    raise Exception("Unsupported platform '%s'" % platform)

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')


class Sensor(Device):
    """
    The sensor class provides a uniform interface for using most of the
    sensors available for the EV3.
    """

    SYSTEM_CLASS_NAME = 'lego-sensor'
    SYSTEM_DEVICE_NAME_CONVENTION = 'sensor*'
    __slots__ = [
        '_address', '_command', '_commands', '_decimals', '_driver_name', '_mode', '_modes', '_num_values', '_units',
        '_value', '_bin_data_format', '_bin_data_size', '_bin_data', '_mode_scale'
    ]
