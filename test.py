###################################
### load probot_empty.ttt scene ###
###################################
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C

tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
tank_pair.on_for_rotations(-20, -20, 1)
