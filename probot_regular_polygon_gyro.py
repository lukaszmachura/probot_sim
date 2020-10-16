# Create a program to make the robot move straight and then turn 90 degrees.
# Import of necessary libraries
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import GyroSensor

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

# Plug a gyro sensor into any sensor port
gy = GyroSensor()
# Set mode of gyro sensor
gy.mode = 'GYRO-ANG'

for step in range(4):
    # read the value from gyro sensor
    start_angle = gy.value()
    angle = start_angle
    print(angle, start_angle)

    # move 2 full wheel rotations forward
    tank_pair.on_for_rotations(20, 20, 2)

    # do the loop while condition is true
    while abs(angle - start_angle) < 90 :
        angle = gy.value()
        # you can see the results on the screen
        print(angle, abs(angle - start_angle))
        tank_pair.on(-10, 10)

tank_pair.off()
