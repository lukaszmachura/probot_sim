###################################
### load probot_empty.ttt scene ###
###################################
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import LightSensor, GyroSensor
from time import sleep, time

tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)

light = LightSensor()
intensity = light.reflected_light_intensity
print(intensity, end=', ')

g = GyroSensor()
angle = g.value()
print(angle)

# tank_pair.on(20, -20)
tank_pair.on_for_rotations(20, 20, 1)
tank_pair.on_for_rotations(20, -20, 1)

t = time()

while time() - t < 1:
    intensity = light.reflected_light_intensity
    print(intensity, end=', ')
    angle = g.value()
    print(angle)

tank_pair.off()
