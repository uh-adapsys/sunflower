Sunflower Controller Package
===

Provides a mid-level, unified control system for the Sunflower robot.
Creates an ActionServer on /sf_controller and acts as an interface with the low level control packages.

##Action Description
__component__: The named joint to control (```head```, ```tray```, matched to the ```sf_base/*joint_configurations.yaml```)
__action__: ```move``` or ```init```
__namedPosition__: Position to be resolved with data from ```sf_base/*joint_configurations.yaml``` _takes prescidence over jointPositions_
__jointPoisitions__: Raw joint data (in radians) to use.  Length must match ```joint_names``` from the ```sf_base/*joint_configurations.yaml``` file.

__result__: Always 0

##Launching

The ```start.launch``` launches the controller.
