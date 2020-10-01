import vrep
import sys
import math

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
