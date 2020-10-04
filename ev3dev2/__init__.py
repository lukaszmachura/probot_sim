# -----------------------------------------------------------------------------
# Copyright (c) 2015 Ralph Hempel <rhempel@hempeldesigngroup.com>
# Copyright (c) 2015 Anton Vanhoucke <antonvh@gmail.com>
# Copyright (c) 2015 Denis Demidov <dennis.demidov@gmail.com>
# Copyright (c) 2015 Eric Pascual <eric@pobot.org>
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
import sys
import os
import io
import fnmatch
import re
import stat
import errno
from os.path import abspath

try:
    # if we are in a released build, there will be an auto-generated "version"
    # module
    from .version import __version__
except ImportError:
    __version__ = "<unknown>"

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')

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


clientID = connection()

def get_current_platform():
    return 'sim'


# -----------------------------------------------------------------------------
def list_device_names(class_path, name_pattern, **kwargs):
    pass


def library_load_warning_message(library_name, dependent_class):
    return 'Import warning: Failed to import "{}". {} will be unusable!'.format(library_name, dependent_class)


class DeviceNotFound(Exception):
    pass


class DeviceNotDefined(Exception):
    pass


class ThreadNotRunning(Exception):
    pass


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


def list_devices(class_name, name_pattern, **kwargs):
    pass
