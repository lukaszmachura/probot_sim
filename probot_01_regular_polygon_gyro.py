# Import of necessary libraries
import ev3dev.sim as ev3
from time import sleep

# enable for some sim output
ev3.VERBOSE = True

#Connect two large motors to port B and port C
b = ev3.LargeMotor('outB')
c = ev3.LargeMotor('outC')

#Plug a gyro sensor into any sensor port
gy = ev3.GyroSensor()

#Test gyro sensor.  if the connection is false trigger an error.
assert gy.connected, "Connect a single gyro sensor"

#Set mode of gyro sensor
#MODE_GYRO_CAL = 'GYRO-CAL'
gy.mode = 'GYRO-ANG'
units = gy.units

# read the value from gyro sensor
start_angle = gy.value()
print(f"start_angle: {start_angle} {units}")
angle = 0

# do the loop while condition is true
# i.e. turn the left or right (it is depend you have got b or c)

while angle - start_angle < 90:
    angle = gy.value()
    # you are able to see the results on the screen
    print(str(angle) + " " + units)
    b.runforever(speed_sp=10)
    # c.runforever(speed_sp=10)

# stop the robot
b.stop(stop_action="brake")
# #c.stop(stop_action=”brake”)
