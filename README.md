
## 4 Degree of Freedom Arm (4-DOF Arm)

A vision-guided robotic picking system controlled by natural language. Depth data from the Orbbec Gemini 2 
camera is processed into a 3D point cloud, the GPT model (GPT 5.2) selects the intended target from the scene. 
The IK solver then converts the object's 3D position into joint angles, which are sent over USB serial to 
the Arduino and executed by the servos via a PCA9685 PWM driver for the joint servos and a Serial Bus Servo 
Driver Board for the gripper servo.

## Hardware
- Orbbec Gemini 2 RGB‑D camera
- Arduino Mega
- PCA9685 PWM driver
- Serial bus servo driver (gripper)
- 5 servos + external power