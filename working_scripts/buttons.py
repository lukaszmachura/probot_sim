from base_vrep import *
from time import time


t = time()
while time() - t < 10:

    fun = 'ButtonPressed'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [], [], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    returnCode, outInts, outFloats, outStrings, outBuffer = out
    print(outInts[0])
