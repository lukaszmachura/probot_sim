#######################################
### load probot_detection.ttt scene ###
#######################################

from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import InfraredSensor
from time import sleep

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
ir = InfraredSensor()
# color.mode = 'COL-REFLECT'


loc = ir.proximity
print(loc)

tank_pair.on(20, 20)
while loc > 10:
    loc = ir.proximity
    print(loc)

tank_pair.off()
