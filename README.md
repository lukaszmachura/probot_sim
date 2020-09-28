# Lego ev3 simulator

![p:ROBOT logo](probot_logo.png)

This is a part of the [p]:ROBOT edu project run by the University of Silesia and Foundation for the development of the Silesian Centre for Education and Interdisciplinary Research in Chorzów EDU-RES. For detailed informations please visit us at [the official project website](https://probot.smcebi.edu.pl).


How to start with the simulator
1. Get latest vrep software (not CoppeliaSim). Go for latest V-REP 3.6.2 version. During download, use the "V-REP PRO EDU" file for your operating system. This [link](http://www.coppeliarobotics.com/previousVersions) will get you to the desired download page.

2. Get Lego EV3 device on [github](https://github.com/albmardom/EV-R3P/tree/master/Modelo%20V-REP) and copy it to [vrep dir]/models/robots/mobile/ directory. Alternatively you can use or from [this link](https://drive.google.com/drive/folders/10v8IzB-qGFPkwCi_i2PbRgh9YLKMt747) to get the robot model.

3. Copy scenes/probot_square.ttt scene to
[vrep dir]/scenes

4. Copy [vrep dir]/programming/remoteApiBindings/python
vrep.py
vrepConst.py
to main probot_sim directory (or any other where you are going to code)

5. Copy
[vrep dir]/programming/remoteApiBindings/lib/lib
to main probot_sim directory (or any other where you are going to code).
Note that you need specific remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)

6. Run vrep.

7. Open "probot_square.ttt" scene (File->Open scene).

8. Run your Python code. You can start from one of the provided examples.
