# Turn on Robot
1. Push the lever on the wall up to turn on the power supply.
2. Turn the knob 90 deg clockwise on the control box to turn on the robot.
3. Release all E-stop buttons (on the teaching pendent and control box).
4. Turn the key on the control box to AUTO.
Turn the knob on the teaching pendent to left.

# To Run Handover Demo
1. Calibrate the camera if needed.
    a. Place the camera at a good position (human and the tag in the view).
    b. Go to `./Calibration/`.
    c. Run `test_kinect.m`. An image will be saved as calib.jpg.
    d. Open VS Code and run `calib.py`. An image will be saved as `calib_detection.jpg`. Verify the tag is successfully detected on the image. If not, change a better camera position and repeat.
2. Turn on SpeedGoat.
3. On the teaching pendant, press 
    a. `shift`+`reset`.
    b. `FCTN`->`ABORT (ALL)`.
    c. Select STMOTION.
4. Press the green button on the robot control box. 
5. Modify the parameters in `./demo.m` if necessary. 
6. Run `./demo.m`.

# To Run Control Demo
1. Go to `./Real-time-Jerk-bounded-Position-Controller/`.
2. Turn on SpeedGoat.
3. On the teaching pendant, press 
    a. `shift`+`reset`.
    b. `FCTN`->`ABORT (ALL)`.
    c. Select STMOTION.
4. Press the green button on the robot control box. 
5. Run `./Real-time-Jerk-bounded-Position-Controller/Fanuc_controller_test.m`.