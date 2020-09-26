import ev3dev.sim as ev3

# ev3.restart_simulation()

tank = ev3.MoveTank("Motor_B", "Motor_C")
# print(f"Gyro Sensor ID: {tank}")

mB = ev3.LargeMotor("Motor_B")
print(f"LargeMotor: {mB}")

g = ev3.GyroSensor()#gyroName="GyroSensor_reference_G")
print(f"Gyro Sensor ID: {g}")

tank.on_for_degrees(43, -3, 33)

# mB.run_to_rel_pos(position_sp=360, speed_sp=100, stop_action='break')

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
# mB.run_to_rel_pos(280, 1)
# mC.run_to_rel_pos(280, 1)

# ev3.restart_simulation()
