# DiffDriveRobotMotorControllerCode

This repository is in correspondence with the autonomous differential drive robot repository.

This contains the Arduino C++ code to be uploaded to the Arduino Nano 33 IoT to control the two DC motors actuating the differential-drive robot.

Specifically, the C++ code implements microROS libraries, which allows microcontrollers to communicate with the ROS2 ecosystem within a robotics system. It receives commands from the hardware interface from the Raspberry Pi to read wheel quadrature encoder values and to write motor velocity commands.
