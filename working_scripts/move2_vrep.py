from base_vrep import *

# motor = 3  # 1 - B, 2 - C, 3 - both

speed = 1
for i in [1, 2, 3, 4]:
    run(3, speed)
    time.sleep(1)
    stop(3)

    reset_gyro()
    while gyro_sensor()[0] < 90:
        run(1, speed)
    stop(1)



#
stop(3)

# on = not True
# if on:
#     run(motor, speed)
# else:
#     stop(motor)


# print(out)
# stop
# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
