Sunflower
=========

Sofware stack and hardware configurations for the University of Hertfordshire Sunflower robot

Getting up and running:
  1) Stacks
    Sunflower relies on the p2os stack for basic platform functionality, this can be installed by running the following
	sudo apt-get install ros-{VERSION}-p2os

    The laser scanners rely on the laser-driver stack, installed via
	sudo apt-get install ros-{VERSION}-laser-drivers

    Additionally, the robot model uses the hokoyo laser model from the pr2 common stack
	sudo apt-get install ros-{VERSION}-pr2-common

  2) Hardware configuration
    The base and dynamixel drivers use a custom port mapping, (p2dxBase and dynamixel).  This mapping is accomplished with udev rules, an example of which can be found in the hardware configs folder.  This usb.rules file needs to be placed into the /udev/rules.d/ folder.  The hardware ids may need to be reconfigured if hardware changes.  Use 'sudo udevadm info --query=all --name=ttyUSB0' to query connected hardware.  Full instructions for this can be found here: http://smstools3.kekekasvi.com/topic.php?id=482.

    Port configuration for the base can be found in sf_base/launch/platform.launch and sf_dynamixel/launch/manager.launch, but should generally not be modified.  Use udev rules instead.

  3) Teleoperation
    Joystick teleoperation is configured for a Logitech ATK3 joystick, and the button mappings would need to be changed for a different controller.
	Sunflower uses the X and W values as it does not have an omni-directional base.  The ros Joy tutorial has a great deal of information regarding connecting a joystick.
        Both the run and deadman buttons must be held in order to teleoperate the robot.
	In order for teleoperation to connect to the joystick all users must have rw access to the port 'sudo chmod a+rw /dev/input/js0'

Launching:
  roslaunch sf_robot {version}.launch  - Core launcher, base, lights, dynamixel drivers and sunflower controller
  roslaunch sf_robot dashboard.launch - Simple dashboard with battery stats and enable/diable motor command
  roslaunch sf_navigation start.launch - Navigation 
  roslaunch sf_robot teleop.launch - Keyboard teleoperation
  roslaunch sf_robot teleop_joy.launch - Joystick teleoperation (run on pc with joystick attached, connects to js0)
  
