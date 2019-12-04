import ev3dev.sim as ev3



mB = ev3.LargeMotor("Motor_B")
mC = ev3.LargeMotor("Motor_C")
mB.run_to_rel_pos(280, 1)
mC.run_to_rel_pos(280, 1)

# ev3.restart_simulation()

# tank = ev3.MoveTank("Motor_B", "Motor_C")
# tank.on_for_degrees(1, 1, 280)
