from base_vrep import *

display_text("Stop 10 cm before the wall")
run(3, 1)
loc = 1000
while loc > 10:
    loc = read_sonar_sensor()
    print(loc)
stop(3)
