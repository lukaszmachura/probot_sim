import ev3dev.sim as ev3




tank = ev3.MoveTank("Motor_B", "Motor_C")
mB = ev3.LargeMotor("Motor_B")

g = ev3.GyroSensor()
print(g)

for i in [1, 2, 3, 4]:
    # forward
    tank.on_for_degrees(1, 1, 5)

    # turn
    mB.run_to_rel_pos(5, 1)

    print(g.value())


# mC = ev3.LargeMotor("Motor_C")
# mB.run_to_rel_pos(280, 1)
# mC.run_to_rel_pos(280, 1)

# ev3.restart_simulation()
