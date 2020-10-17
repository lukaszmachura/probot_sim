from base_vrep import *
from time import sleep


def write_line_on_display(line_number, text):
    assert 0 < line_number < 9
    fun = 'TextOut'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [line_number], [], [text], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )

    returnCode, outInts, outFloats, outStrings, outBuffer = out
    return returnCode

def clear_screen():
    fun = 'ClearScreen'
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
    return returnCode

for line in range(1, 9):
    write_line_on_display(line, f"w" * 19)
#
# sleep(1)
#
# clear_screen()

# write_line_on_display(1, "x" * 30)
