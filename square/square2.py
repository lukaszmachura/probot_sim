import time
from ev3sim import *
import vrep
import sys
import math

time.sleep(1)

# 4 steps for square
for steps in range(4):
    run_to_rel_pos(360, 1, 1, ev3cfg)
    run_to_rel_pos(180, 1, -1, ev3cfg)
    stop_motors(ev3cfg)
