from base_vrep import *


display_text("Stop at the black line")

run(3, 2)

out = get_color_light()
r, g, b, depth, intensity = out
while intensity > 20:
    out = get_color_light()
    r, g, b, depth, intensity = out

stop(3)
