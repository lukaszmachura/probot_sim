from base_vrep import *


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
