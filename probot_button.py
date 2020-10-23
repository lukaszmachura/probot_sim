from ev3dev2.button import Button
from ev3dev2.led import Leds
from ev3dev2.display import Display
from time import sleep, time

buttons = {
    'up': 5,
    'down': 1,
    'left': 3,
    'right': 2,
    'enter': 4,
    'backspace': 6
    }

button = Button()
leds = Leds()
display = Display()

leds.all_off()
display.clear()

t = time()
while time() - t < 1:
    display.text_grid(f"Button pressed: {button.any()}, Button number: {button.get_pressed_button_number}")
    print(button.any(), button.get_pressed_button_number)

# works with numbers
if button.wait_for_pressed(6):
    print("6 pressed")
    display.text_grid("6 pressed")
    leds.set_color('RIGHT', (0, 0, 1))

# and names
if button.wait_for_pressed('up'):
    print('up pressed')
    display.text_grid("up pressed")
