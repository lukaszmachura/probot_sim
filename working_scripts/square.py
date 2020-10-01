import vrep
import sys
import time
import math
#import ev3dev.sim as ev3


def stop_motors(ev3cfg):
    # stop motors
    clientID = ev3cfg['clientID']
    motorB, motorC = ev3cfg['motorL'], ev3cfg['motorR']

    errorCode = vrep.simxSetJointTargetVelocity(clientID,
            motorB, 0,
            #vrep.simx_opmode_streaming)
            #vrep.simx_opmode_oneshot_wait)
            vrep.simx_opmode_blocking)
    errorCode = vrep.simxSetJointForce(clientID,
            motorB, ev3cfg['REST_TORQUE'], vrep.simx_opmode_blocking)

    errorCode = vrep.simxSetJointTargetVelocity(clientID,
            motorC, 0,
            #vrep.simx_opmode_streaming)
            #vrep.simx_opmode_oneshot_wait)
            vrep.simx_opmode_blocking)
    errorCode = vrep.simxSetJointForce(clientID,
            motorC, ev3cfg['REST_TORQUE'], vrep.simx_opmode_blocking)


def run_to_rel_pos(position_sp, speed_sp_left, speed_sp_right, ev3cfg):
    """v0.3, emulate motor.run_to_rel_pos"""

    clientID = ev3cfg['clientID']
    motorB, motorC = ev3cfg['motorL'], ev3cfg['motorR']
    lw_vel, rw_vel = speed_sp_left, speed_sp_right

    # initial positions of wheels
    # wait for both motors
    vrep.simxPauseCommunication(clientID, True)
    rC, ini_left_wheel_pos = vrep.simxGetJointPosition(clientID, motorB,
        vrep.simx_opmode_streaming)
    rC, ini_right_wheel_pos = vrep.simxGetJointPosition(clientID, motorC,
        vrep.simx_opmode_streaming)
    vrep.simxPauseCommunication(clientID, False)


    left_wheel_pos = ini_left_wheel_pos
    right_wheel_pos = ini_right_wheel_pos
    counter_forward = 0
#    while (math.degrees(left_wheel_pos - ini_left_wheel_pos) < FULL_ANGLE
#           and
#           math.degrees(right_wheel_pos - ini_right_wheel_pos) < FULL_ANGLE):
    while (left_wheel_pos - ini_left_wheel_pos < math.radians(position_sp)
           and
           right_wheel_pos - ini_right_wheel_pos < math.radians(position_sp)):

        # wait for both motors
        vrep.simxPauseCommunication(clientID, True)

        # left motor
        errorCode = vrep.simxSetJointTargetVelocity(clientID,
                motorB, lw_vel, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointForce(clientID,
                motorB, ev3cfg['MOTION_TORQUE'], vrep.simx_opmode_streaming)
        # positions of a wheel
        rC, left_wheel_pos = vrep.simxGetJointPosition(clientID,
                motorB, vrep.simx_opmode_streaming)

        # right motor
        errorCode = vrep.simxSetJointTargetVelocity(clientID,
                motorC, rw_vel, vrep.simx_opmode_streaming)
        errorCode = vrep.simxSetJointForce(clientID,
                motorC, ev3cfg['MOTION_TORQUE'], vrep.simx_opmode_streaming)
        # positions of a wheel
        rC, right_wheel_pos = vrep.simxGetJointPosition(clientID,
                motorC, vrep.simx_opmode_streaming)

        vrep.simxPauseCommunication(clientID, False)
    return rC


# some constants
FULL_ANGLE = 360
FULL_RAD = 2 * math.pi

vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote server')
else:
    print('Not connected')
    sys.exit('Could not connect')

# left motor handle
eC, motorB = vrep.simxGetObjectHandle(clientID,
        "Motor_B", vrep.simx_opmode_oneshot_wait)
# roght motor handle
eC, motorC = vrep.simxGetObjectHandle(clientID,
        "Motor_C", vrep.simx_opmode_oneshot_wait)
eC, bumper_handle = vrep.simxGetObjectHandle(clientID,
        "Bumper", vrep.simx_opmode_oneshot_wait)
eC, sonar_handle = vrep.simxGetObjectHandle(clientID,
        "Sonar", vrep.simx_opmode_oneshot_wait)
eC, senor_color_LR_handle = vrep.simxGetObjectHandle(clientID,
        "Sensor_Color_LR", vrep.simx_opmode_oneshot_wait)
eC, senor_color_RC_handle = vrep.simxGetObjectHandle(clientID,
        "Sensor_Color_RC", vrep.simx_opmode_oneshot_wait)

ev3cfg = {'clientID': clientID,
          'motorL': motorB,
          'motorR': motorC,
          'MOTION_TORQUE': 0.2,
          'REST_TORQUE': 0.4,
          'MAX_SPEED': 18.326
          }

time.sleep(1)

# 4 steps for square
for steps in range(4):
    run_to_rel_pos(360, 1, 1, ev3cfg)
    run_to_rel_pos(180, 1, -1, ev3cfg)
    stop_motors(ev3cfg)
