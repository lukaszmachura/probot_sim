###################################
### load probot_empty.ttt scene ###
###################################
from ev3dev2.led import Leds
from ev3dev2.display import Display
from time import sleep

leds = Leds()
display = Display()

leds.all_off()
display.clear()
sleep(2)

display.text_grid("POLICE!!!", y=3, x=9)

for i in range(10):
    if i % 2:
        leds.set_color('RIGHT', (0, 0, 1))
        leds.set_color('LEFT', (1, 0, 0))
    else:
        leds.set_color('RIGHT', (1, 0, 0))
        leds.set_color('LEFT', (0, 0, 1))
    sleep(0.3)
