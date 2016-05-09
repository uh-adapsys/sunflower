Sunflower Controller Messages Package
===

Messages for the sf_controller package

##Action Description
__component__: The named joint to control (```head```, ```tray```, matched to the ```sf_base/*joint_configurations.yaml```)

__action__: ```move``` or ```init```

__namedPosition__: Position to be resolved with data from ```sf_base/*joint_configurations.yaml``` _takes prescidence over jointPositions_

__jointPoisitions__: List of joint positions (in radians) to use.  Length must match ```joint_names``` from the ```sf_base/*joint_configurations.yaml``` file.


__result__: Always 0
