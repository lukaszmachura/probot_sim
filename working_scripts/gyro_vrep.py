from base_vrep import *

dev = 'Giroscopio'
dev = 'GyroSensor_G'
dev = 'GyroSensor_reference_G'
# dev = 'GyroSensor_VA'
gyro = handles[dev]
print(dev, 'is', gyro)

# sim.getIntegerSignal('gyroZ_angle')

emptyBuff = bytearray()
out = vrep.simxCallScriptFunction(
    clientID,
    # 'Funciones',
    dev,
    # vrep.sim_scripttype_childscript, 
    gyro,
    'SensorGyroA',
    [], [], [], emptyBuff,
    vrep.simx_opmode_buffer #locking
)
returnCode, outInts, outFloats, outStrings, outBuffer = out

print(out)
# stop
# vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)


# if clientID !=- 1:
#     print ('Connected to remote API server')
#
#     # 1. First send a command to display a specific message in a dialog box:
#     emptyBuff = bytearray()
#     res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(
#         clientID,
#         'Gyroscopio', #'remoteApiCommandServer',
#         vrep.sim_scripttype_childscript,
#         '', #'SensorGyroG', #'displayText_function',
#         [],[],[],
#         emptyBuff,
#         vrep.simx_opmode_blocking)
#     if res==vrep.simx_return_ok:
#         print ('Return string: ',retStrings[0]) # display the reply from V-REP (in this case, just a string)
#     else:
#         print ('Remote function call failed')

# dev = 'GyroSensor_reference_G'
# ref = handles[dev]
# o = vrep.simxGetObjectMatrix(ref, -1)
