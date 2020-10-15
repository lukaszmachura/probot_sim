from base_vrep import *

# display_text("Stop 10 cm before the wall")

run(3, 2)
loc = read_sonar_sensor()

for i in range(2):
    while loc > 10:
        loc = read_sonar_sensor()
        print(loc)
    run(3, -2)

    while loc < 330:
        loc = read_sonar_sensor()
        print(loc)
    run(3, 2)

stop(3)
