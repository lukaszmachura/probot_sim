import ev3dev.sim as ev3

mB = ev3.LargeMotor("Motor_B")
mB.run_to_rel_pos(360, 1)
mB.run_to_rel_pos(100, 1)
