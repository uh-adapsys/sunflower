Sunflower Package
===

Provides the high-level entry points for starting the sunflower robot.

##Configuration

Contains the ```{VERSION}.xacro``` files that describe the physical configuration of the robot

##Launching

```{version}.launch``` base, lights, dynamixel drivers and sunflower controller

```dashboard.launch``` Simple dashboard with battery stats and enable/diable motor command

```teleop.launch``` Keyboard teleoperation

```teleop_joy.launch``` Joystick teleoperation (must be run on the computer that the joystick is attached to, connects to js0)

```tf.launch``` Starts the joint state publisher based off the configuration from the ```{VERSION}.xacro``` file
