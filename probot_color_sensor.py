# Create a program to make the robot move straight and then turn 90 degrees.
# Import of necessary libraries
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import LightSensor
from time import sleep

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
light = LightSensor()
# color.mode = 'COL-REFLECT'

tank_pair.on(20, 20)

intensity = light.reflected_light_intensity
print(intensity)

while intensity > 20:
    intensity = light.reflected_light_intensity
    print(intensity)

tank_pair.off()
