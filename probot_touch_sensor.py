#######################################
### load probot_detection.ttt scene ###
#######################################
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import GyroSensor, TouchSensor
from ev3dev2.display import Display
from ev3dev2.led import Leds

# Connect two large motors to port B and port C
tank_pair = MoveTank(OUTPUT_B, OUTPUT_C)
gy = GyroSensor()
ts = TouchSensor()
d = Display()
led = Leds()


start_angle = gy.value()
angle = start_angle

led.set_color('BOTH', (0, 0, 1))
while abs(angle - start_angle) < 180 :
    angle = gy.value()
    # you can see the results on the screen
    print(angle, abs(angle - start_angle))
    tank_pair.on(-10, 10)

led.set_color('BOTH', 'GREEN')
while not ts.is_pressed:
    angle = gy.value()
    # you can see the results on the screen
    print(angle, abs(angle - start_angle), ts.is_pressed)
    d.text_grid("Touch Sensor pressed: " + str(ts.is_pressed))
    tank_pair.on(-20, -20)

led.set_color('BOTH', 'RED')
tank_pair.off()
