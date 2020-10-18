###################################
### load probot_empty.ttt scene ###
###################################
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import ColorSensor
from time import sleep

color = ColorSensor()
color.mode = 'COL-REFLECT'

print('\nBefore the calibration')
print('MAX:', color.red_max, color.green_max, color.blue_max)
print('read:', color.read_color())
print('raw (0 - 300):', color.raw)
print('rgb (0-255):', color.rgb)
print('hls:', color.hls)

color.calibrate_white()
print('\nAfter the calibration')
print('MAX:', color.red_max, color.green_max, color.blue_max)
print('read:', color.read_color())
print('raw (0 - 300):', color.raw)
print('rgb (0-255):', color.rgb)
print('hls:', color.hls)
