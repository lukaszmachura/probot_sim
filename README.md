# Lego ev3 simulator

![p:ROBOT logo](probot_logo.png)

This is a part of the [p]:ROBOT edu project run by the University of Silesia and Foundation for the development of the Silesian Centre for Education and Interdisciplinary Research in Chorzów EDU-RES. For detailed informations please visit us at [the official project website](https://probot.smcebi.edu.pl).


How to start with the simulator
1. Get latest vrep software (not CoppeliaSim). Go for latest V-REP 3.6.2 version. During download, use the "V-REP PRO EDU" file for your operating system. This [link](http://www.coppeliarobotics.com/previousVersions) will get you to the desired download page.

*Note: if you will use one of the scenes provided in this repo, you can skip point 2*

2. Get Lego EV3 device on [github](https://github.com/albmardom/EV-R3P/tree/master/Modelo%20V-REP) and copy it to [vrep dir]/models/robots/mobile/ directory. Alternatively you can use [this link](https://drive.google.com/drive/folders/10v8IzB-qGFPkwCi_i2PbRgh9YLKMt747) to get the robot model.

3. Copy scenes/probot_square.ttt scene to
[vrep dir]/scenes

*Note: if you will use this repo, you can skip point 4*

4. Copy [vrep dir]/programming/remoteApiBindings/python
vrep.py
vrepConst.py
to main probot_sim directory (or any other where you are going to code)

5. Copy
[vrep dir]/programming/remoteApiBindings/lib/lib
to main probot_sim directory (or any other where you are going to code).
Note that you need specific remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)

6. Run vrep.

7. Open scene (File->Open scene)
* probot_color_sensor.ttt
* probot_empty.ttt
* probot_detection.ttt

8. Run your Python code. You can start from one of the provided examples:
* **empty scene**
  * probot_button.py
  * probot_display.py
  * probot_leds.py
  * probot_move_tank.py
  * probot_regular_polygon.py
  * probot_regular_polygon_gyro.py
  * probot_regular_polygon_ev3dev.py
* **wall detection**
  * probot_ir_proximity_sensor.py
  * probot_touch_sensor.py
  * probot_ultrasonic_sensor.py
* **color**
  * probot_color_calibrate_white.py
  * probot_color_sensor.py
