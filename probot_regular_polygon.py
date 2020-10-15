import ev3dev.sim as ev3

# initialize tank mode
tank = ev3.MoveTank("Motor_B", "Motor_C")

# number of sides
N = 3

# left and right motor's speed
left_speed = right_speed = 5

# an actual movement
for side in range(N):
    print(f"side {side + 1}")

    # forward
    rotate = 360
    tank.on_for_degrees(left_speed, right_speed, rotate)

    # turn right
    rotate = 360 * 4 / N
    tank.on_for_degrees(left_speed, 0, rotate)
