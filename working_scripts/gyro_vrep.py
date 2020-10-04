from base_vrep import *


reset_gyro()
out = gyro_sensor()
print(out)
run(1, 2)
while out[0] < 360:
    out = gyro_sensor()
    print(out)
stop(1)
