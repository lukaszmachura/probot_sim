from ev3dev2.display import Display
from time import sleep

disp = Display()

disp.text_grid("0123456789" * 20)
sleep(1)

disp.text_grid("""Picture yourself in a boat on a river;With tangerine trees and marmalade skies;Somebody calls you, you answer quite slowly;A girl with kaleidoscope eyes""")
