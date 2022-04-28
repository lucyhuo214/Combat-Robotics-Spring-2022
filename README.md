# Combat-Robotics-Spring-2022
This repository includes the files for the Combat Robotics independent study at Duke University under the mentorship of Dr. Tyler Bletsch. Contributors to this project are Lucy Huo, Richard Kim, and Emily Shao.

# Project Overview
The task for this project was to build a combat robot according to the SPARC Robot Construction Specifications in the antweight class. The final project was a combat robot with a holonomic drive system. The robot uses omni wheels and an inertial measurement unit (IMU) sensor to achieve this task, correcting errors in misalignment based on IMU sensor feedback without the use of encoders on the wheels.

The primary components of the combat robot design are the three-omni-wheel kiwi drive system and the implementation of a BNO055 accelerometer to enable closed loop control, stabilizing the drive system. The software control for the robot is implemented using an Arduino Nano microcontroller.
