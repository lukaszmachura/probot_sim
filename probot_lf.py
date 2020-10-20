##########################################
### load probot_color_sensor.ttt scene ###
##########################################
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import LightSensor
from time import sleep
from ev3dev2.button import Button

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
light = LightSensor()
# color.mode = 'COL-REFLECT'
button = Button()

tank_pair.on(20, 20)

intensity = light.reflected_light_intensity
max = intensity
min = intensity
while intensity > 15:
    intensity = light.reflected_light_intensity
    if intensity > max:
        max = intensity
    if intensity < min:
        min = intensity
tank_pair.off()
middle=0.5*(max - min)

while not button.any():
  intensity = light.reflected_light_intensity
  if intensity>middle:
    tank_pair.on(15,5)
  else:
    tank_pair.on(5,15)

tank_pair.off()
