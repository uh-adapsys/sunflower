Sunflower Dynamixel Package
===

ROS Stack for controlling the dynamixel servos on Sunflower

Currently, dynamixels account for:
* Head
* Neck
* Tray

Future Plans:
* Arms

##Configuration

The ```config``` directory contains the yaml description file for each robot named ```{version}.yaml```.  This details the configuration settings for each servo.  The min and max positions will be used by the controller to trim any commands sent to a valid position.  For example, if the max is 500 and the max sent is 600, then the motor will only recieve 500.  The init value will be mapped to the 0 point by the controller.  All commands are accepted in radians and automatically converted to ticks.

The ```config/*joint_configurations.yaml``` files give named positions for known joints, in radians.
