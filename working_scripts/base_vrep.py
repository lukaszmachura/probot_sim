import vrep
import sys
import math
import time

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

VERBOSE = True

connectionAddress = '127.0.0.1'
connectionPort = 19999
waitUntilConnected = True
doNotReconnectOnceDisconnected = True
timeOutInMs = 5000
commThreadCycleInMs = 5

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

# vrep.simxSynchronous(clientID, 1)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

handles = {}
for name in [
    'Motor_B', 'Rueda_C_resp', 'Rueda_C',
    'Motor_C','Rueda_B_resp', 'Rueda_B',
    'Slider_SF', 'Slider_Resp', 'Slider',
    'Giroscopio', 'GyroSensor_G', 'GyroSensor_reference_G',
    'GyroSensor_VA', 'GyroSensor_reference',
    'Bumper', 'Pulsador_Resp', 'Pulsador',
    'Sonar',
    'V_LEGO_EV3', 'Functiones',
    'Sensor_Color_LR', 'Sensor_Color_RC',
    'Camara', 'Camara_bumper', 'Camara_sonar',
    ]:

    eC, var = vrep.simxGetObjectHandle(clientID,
        name,
        vrep.simx_opmode_blocking)  # simx_opmode_oneshot_wait)
    handles[name] = var
    if VERBOSE and False:
        print(name, api_return_codes.get(eC, eC), var)


###
# below functions should help bulid around sensors

def write_line_on_display(line_number, text):
    assert 0 < line_number < 9
    fun = 'TextOut'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [line_number], [], [text], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )

    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def clear_screen():
    fun = 'ClearScreen'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )

    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def get_rotation(motor):
    "..."
    if motor == 'B':
        fun = 'MotorRotationCountB'
    elif motor == 'C':
        fun = 'MotorRotationCountC'
    else:
        raise ValueError('motor = C or B')

    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return outInts[0]


def reset_rotation_count(motor):
    """
    motor: motor number
        1: 'B' or 'left'
        2: 'C' or 'right'
    """
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'ResetRotationCount',
        [motor], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def run(motor, speed):
    """
    motor (int): motor number
        1: 'B' or 'left'
        2: 'C' or 'right'
        3: both
    speed (int/float): speed of motor, in [-MAX_SPEED, MAX_SPEED]
    """
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'On',
        [motor, int(speed)], [speed], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def stop(motor):
    "..."
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'Off',
        [motor], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def read_sonar_sensor():
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        # dev,
        vrep.sim_scripttype_childscript,
        # handle,
        'SensorSonar',
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait #buffer #locking
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return outFloats[0]


def display_text(message):
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'remoteApiCommandServer',
        vrep.sim_scripttype_childscript,
        'displayText_function',
        [], [], [message], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    if outStrings[0] == 'message was displayed':
        return vrep.simx_return_ok
    else:
        return vrep.simx_return_remote_error_flag


def reset_gyro():
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'ResetGyroA',
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode


def gyro_sensor():
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
    angle = outInts[0]

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
    angular_velocity = outInts[0]

    return angle, angular_velocity


def get_light():
    'returns intensity'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'SensorLight',
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return outInts[0]


def get_color():
    'returns red, green, blue, depth values'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        'SensorColor',
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return outFloats


def get_color_light():
    return get_color() + [get_light()]
