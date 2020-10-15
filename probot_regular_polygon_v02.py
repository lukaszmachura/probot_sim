'''

'''

import ev3dev.sim as ev3

# initialize tank mode
tank = ev3.MoveTank("Motor_B", "Motor_C")

# number of sides
N = 5

# left and right motor's speed
left_speed = right_speed = 5

# an actual movement
for side in range(N):
    print(f"side {side + 1}")

    # forward
    tank.on_for_rotations(left_speed, right_speed, 1)

    # turn right
    rotate = 180 * 4 / N
    tank.on_for_degrees(left_speed, -right_speed, rotate)
