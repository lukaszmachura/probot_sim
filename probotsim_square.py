import ev3dev.sim as ev3

ev3.VERBOSE = not True
# ev3.restart_simulation()

tank = ev3.MoveTank("Motor_B", "Motor_C")
if ev3.VERBOSE:
    print(f"Tank: {tank}")

mB = ev3.LargeMotor("Motor_B")
if ev3.VERBOSE:
    print(f"LargeMotor: {mB}")


tank.on_for_degrees(3, -3, 360)


# mB.run_to_rel_pos(position_sp=360, speed_sp=3, stop_action='break')

# for i in [1, 2, 3, 4]:
#     print(f"step {i}")
#     # forward
#     tank.on_for_degrees(13, 13, 10)
#
#     # turn
#     mB.run_to_rel_pos(13, 90)
#
#     print("Gyro Sensor", g.value())


# mC = ev3.LargeMotor("Motor_C")
# mB.run_to_rel_pos(280, 13)
# mC.run_to_rel_pos(280, 13)

# ev3.restart_simulation()
