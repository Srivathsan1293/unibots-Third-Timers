Software architecture:

1. Phone handles position and long range ball detection.
2. Rasberry Pi 4 handles close range ball detection (0-20cm) and FSM communicating to both phone and ESP's
3. Encoder.ino runs on Drive_esp which handles wasd instrucitons using PID with IMU+encoder fusion.
4. Launcher.ino control launch sequence

The phone used is a pixel 7. 
The onboard TPU is used to accelerate ML inference to detect balls even 2m away.
Using focal length data and real world position the system determines robots position to a couple cm. 
Using exact position the position of balls is also estimated and shown on a map with the robot always moving towards the closest ball.

In order to maximise on day performance all variables can be adjusted quickly using a settings menu.

