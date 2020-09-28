# probot_sim
[Probot Simulator](https://probot.smcebi.edu.pl)

How to start
1. Get latest vrep software (not CoppeliaSim). Go for latest V-REP 3.6.2 version.
http://www.coppeliarobotics.com/previousVersions

2. Get Lego EV3 device
https://github.com/albmardom/EV-R3P/tree/master/Modelo%20V-REP
and copy it to [vrep dir]/models/robots/mobile/ directory

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

7. Open "probot_square.ttt" scene.

8. Run your code.
