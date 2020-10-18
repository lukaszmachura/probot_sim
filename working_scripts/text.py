from base_vrep import *
from time import sleep




# for line in range(1, 9):
#     write_line_on_display(line, f"{line}" * 19)

clear_screen()

# for status in range(5):
#     write_line_on_display(status + 1, f"status {status}")
#
#     fun = 'StatusLightLeft'
#     emptyBuff = bytearray()
#     out = vrep.simxCallScriptFunction(
#         clientID,
#         'Funciones',
#         vrep.sim_scripttype_childscript,
#         fun,
#         [status], [], [], emptyBuff,
#         vrep.simx_opmode_oneshot_wait
#     )
#
#     fun = 'StatusLightRight'
#     emptyBuff = bytearray()
#     out = vrep.simxCallScriptFunction(
#         clientID,
#         'Funciones',
#         vrep.sim_scripttype_childscript,
#         fun,
#         [(status) % 4], [], [], emptyBuff,
#         vrep.simx_opmode_oneshot_wait
#     )
#
#     returnCode, outInts, outFloats, outStrings, outBuffer = out
#
#     sleep(.51)

for i in range(10):
    print(i)
    fun = 'StatusLight'
    emptyBuff = bytearray()
    out = vrep.simxCallScriptFunction(
        clientID,
        'Funciones',
        vrep.sim_scripttype_childscript,
        fun,
        [i % 4], [1 - i * 0.1, i * 0.1, 0], [], emptyBuff,
        vrep.simx_opmode_oneshot_wait
    )
    sleep(0.2)
    # if i % 2:
    #     sleep(0.1)
    # else:
    #     sleep(0.5)
