# ------------------------------------------------------------------------------
# Copyright (c) 2020 LM
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
# FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# -----------------------------------------------------------------------------
"""
An assortment of classes modeling specific features of the EV3 brick.
"""

from collections import OrderedDict

OUTPUT_A = 'Motor_A'
OUTPUT_B = 'Motor_B'
OUTPUT_C = 'Motor_C'
OUTPUT_D = 'Motor_D'

# OUTPUT_A = 'outA'
# OUTPUT_B = 'outB'
# OUTPUT_C = 'outC'
# OUTPUT_D = 'outD'

INPUT_1 = 'in1'
INPUT_2 = 'in2'
INPUT_3 = 'in3'
INPUT_4 = 'in4'

BUTTONS_FILENAME = None
EVDEV_DEVICE_NAME = None

LEDS = 'StatusLight'
LED_GROUPS = {'LEFT': 'L', 'RIGHT': 'R', 'BOTH': 'B'}
NAMED_COLORS = {'GREEN': (0, 1, 0),
                'AMBER': (0.89, 0.54, 0.23),
                'RED': (1, 0, 0),
                'GREY': (0.85, 0.85, 0.85)}
LED_COLORS = {#'Default': NAMED_COLORS['GREY'],
              'GREEN': NAMED_COLORS['GREEN'],
              'AMBER': NAMED_COLORS['AMBER'],
              'RED': NAMED_COLORS['RED'],
              'GREY': NAMED_COLORS['GREY']}
LED_DEFAULT_COLOR = 'GREY'
