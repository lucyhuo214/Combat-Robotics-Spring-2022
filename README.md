# Combat-Robotics-Spring-2022
This repository includes the files for the Combat Robotics independent study at Duke University under the mentorship of Dr. Tyler Bletsch. Contributors to this project are Lucy Huo, Richard Kim, and Emily Shao.

# Project Overview
The task for this project was to build a combat robot according to the SPARC Robot Construction Specifications in the antweight class. The final project was a combat robot with a holonomic drive system. The robot uses omni wheels and an inertial measurement unit (IMU) sensor to achieve this task, correcting errors in misalignment based on IMU sensor feedback without the use of encoders on the wheels.

The primary components of the combat robot design are the three-omni-wheel kiwi drive system and the implementation of a BNO055 accelerometer to enable closed loop control, stabilizing the drive system. The software control for the robot is implemented using an Arduino Nano microcontroller.

# Holonomic Drive System
The final combat robot moves using a holonomic drive system. Holonomic drive is a drive system in which the robot’s degrees of freedom of movement are equal to the robot’s degrees of control. The combat robot in this project can move freely in the X- and Y-directions without turning, successfully implementing a holonomic drive system.

The robot uses a kiwi drive system to accomplish this, which is a form of holonomic drive that uses three omni wheels placed 120 degrees apart. Omni wheels have small rollers that are perpendicular to the direction of rotation for a wheel, allowing the wheel to roll sideways along with going forward and backwards. Omni wheels are also able to move laterally,  giving the robot full range of motion in any direction.

The CombatRoboticsHolonomic.ino script includes the robot's implementation of a holonomic drive system using kiwi drive.

# PID Closed Loop Control System
The greatest challenge that arose while implementing kiwi drive was that the robot would veer off in undesired directions due to various internal or external factors. These might include slight variations in motor performance or uneven friction within the combat arena. In most kiwi drive systems, encoders are placed on each of the three wheels to correct misalignment. However, this robot used feedback from an IMU sensor to detect and correct these misalignments without the use of any encoders.

The self-correcting control is implemented using the BNO055 accelerometer sensor. The accelerometer is mounted to the base of the robot chassis, allowing it to remain fixed to the robot and accurately gather orientation data while the robot is in motion.

The self-oriented control system is implemented using a form of closed loop control, in which portions of the output signal are used as input to the system to reduce errors or achieve system stability. For this project, the output was the yaw value from the BNO055, which was then used as an input to the kiwi drive calculations to ensure a more stable holonomic drive system.

The robot utilizes PID control, also known as Proportional, Integral, and Derivative control, to achieve this function. The proportional control adjusts the output in proportion to the error measured. The derivative control limits the overshoot of the proportional adjustment, helping the robot converge on its goal orientation faster. Finally, the integral is normally used to reduce static error in a system, but was not implemented in this project because it was negligible while in combat.

The CombatRoboticsSelfOrienting.ino script is a proof-of-concept demonstrating the ability of the robot to use PID closed loop control. The script enables the robot to always face a set absolute orientation, self correcting towards this setpoint using applications of PID control theory.

Finally, the CombatRoboticsFinalCode.ino script includes the final iteration of software control for the combat robot. It moderates the robot's holonomic drive system using PID control, while still allowing the robot driver to move and rotate in all directions in the XY-plane as desired.
