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


def get_ini_pos(h):
    # Do sth with Motor B
    rC = 1
    while rC != vrep.simx_error_noerror:
        rC, pos = vrep.simxGetJointPosition(
                            clientID,
                            handles[h],
                            vrep.simx_opmode_oneshot)
    return rC, pos


def on_for_degrees(left_speed, right_speed, degrees):
    if degrees == 'full':
        degrees = 360

    rad = degrees * math.pi / 180

    rC, ini_wheel_pos_B = get_ini_pos('Motor_B')
    rC, ini_wheel_pos_C = get_ini_pos('Motor_C')
    print(ini_wheel_pos_B, ini_wheel_pos_C)
    print('--------------------------------')

    errorCodeSJF = vrep.simxSetJointForce(
        clientID, handles['Motor_B'], 10.2,
        vrep.simx_opmode_streaming)
    errorCodeSJF = vrep.simxSetJointForce(
        clientID, handles['Motor_C'], 10.2,
        vrep.simx_opmode_streaming)

    pos_B, pos_C = ini_wheel_pos_B, ini_wheel_pos_C

    while ((abs(pos_B - ini_wheel_pos_B) <= rad)
        and (abs(pos_C - ini_wheel_pos_C) <= rad)):

        vrep.simxSetJointTargetVelocity(
            clientID, handles['Motor_B'], left_speed,
            vrep.simx_opmode_oneshot_wait
        )
        vrep.simxSetJointTargetVelocity(
            clientID, handles['Motor_C'], right_speed,
            vrep.simx_opmode_oneshot_wait
        )

        rC, pos_B = vrep.simxGetJointPosition(
                            clientID,
                            handles['Motor_B'],
                            vrep.simx_opmode_streaming)
        rC, pos_C = vrep.simxGetJointPosition(
                            clientID,
                            handles['Motor_C'],
                            vrep.simx_opmode_streaming)
        print(pos_B - ini_wheel_pos_B, pos_C - ini_wheel_pos_C)

    errorCodeSJF = vrep.simxSetJointForce(
        clientID, handles['Motor_B'], 0.4,
        vrep.simx_opmode_streaming)
    errorCodeSJF = vrep.simxSetJointForce(
        clientID, handles['Motor_C'], 0.4,
        vrep.simx_opmode_streaming)

    vrep.simxSetJointTargetVelocity(
        clientID, handles['Motor_B'], 0,
        vrep.simx_opmode_oneshot_wait
    )
    vrep.simxSetJointTargetVelocity(
        clientID, handles['Motor_C'], 0,
        vrep.simx_opmode_oneshot_wait
    )

    return pos_B, pos_C





speed = 1
for step in range(4):
    on_for_degrees(speed, speed, 360)
    on_for_degrees(speed, -speed, 180)

# stop
# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
