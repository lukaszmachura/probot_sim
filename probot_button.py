from ev3dev2.button import Button
from ev3dev2.led import Leds
from ev3dev2.display import Display
from time import sleep, time

button = Button()
leds = Leds()
display = Display()

leds.all_off()
display.clear()

t = time()
while time() - t < 1:
    display.text_grid(f"Button pressed: {button.any()}, Button number: {button.get_pressed_button_number}")
    print(button.any(), button.get_pressed_button_number)


if button.wait_for_pressed(6):
    print("6 pressed")

if button.wait_for_pressed('up'):
    print('up pressed')
