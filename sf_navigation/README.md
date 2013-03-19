Sunflower Controller Package
===

Provides a mid-level, unified control system for the Sunflower robot.
Creates an ActionServer on /sf_controller and acts as an interface with the low level control packages.

##Configuration

A description of these files can be found: http://www.ros.org/wiki/navigation/Tutorials/RobotSetup

####Notes
__costmap_common_params.yaml__: observation source->sensor_frame __must__ match the frame defined in the laser scanner launch file.  For Sunflower this is ```sf_base hokoyo.launch``` and defined as ```base_laser_front_link```

__base_local_planner_params.yaml__: 

__global_costmap_params.yaml__: robot_base_frame must match the frame_id from the robot description (```sf_robot/config/{VERSION}.xacro```), usually base_link.

__local_costmap_params.yaml__: robot_base_frame must match the frame_id from the robot description (```sf_robot/config/{VERSION}.xacro```), usually base_link.

__amcl.launch__: base_frame_id must match the frame_id from the robot description (```sf_robot/config/{VERSION}.xacro```), usually base_link.

##Launching

The ```start.launch``` launches the ```map_server```, ```amcl``` and ```move_base``` nodes.
