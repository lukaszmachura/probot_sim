from base_vrep import *
import time

# display_text("Stop at the black line")
run(3, -2)
time.sleep(2)
run(3, 2)

out = get_color_light()
r, g, b, depth, intensity = out
while intensity > 20:
    out = get_color_light()
    r, g, b, depth, intensity = out
    print(out)

stop(3)
