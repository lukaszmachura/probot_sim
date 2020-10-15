# Create a program to make the robot move straight and then turn 90 degrees.
# Import of necessary libraries
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import UltrasonicSensor, GyroSensor
from time import sleep

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
u = UltrasonicSensor()
gy = GyroSensor()


loc = u.distance_centimeters
print(loc)

vl, vr = 20, -20
for i in range(2):
    tank_pair.on(20, 20)
    while loc > 10:
        loc = u.distance_centimeters
        print(loc)
    tank_pair.on(-20, -20)

    while loc < 60:
        loc = u.distance_centimeters
        print(loc)

    start_angle = gy.value()
    angle = start_angle
    vl, vr = vr, vl
    while abs(angle - start_angle) < 360 :
        angle = gy.value()
        print(angle, abs(angle - start_angle))
        tank_pair.on(vl, vr)

tank_pair.off()
