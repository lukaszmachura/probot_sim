from base_vrep import *


sensor = "Sensor_Color_LR"
handle = handles[sensor]
print(sensor, handle)

data = vrep.simxGetVisionSensorImage(
    clientID,
    handle, 0,
    vrep.simx_opmode_buffer)

print(api_return_codes[data[0]])

if data[0] == vrep.simx_return_ok:
    imageAcquisitionTime = vrep.simxGetLastCmdTime(clientID)
    print(imageAcquisitionTime)
