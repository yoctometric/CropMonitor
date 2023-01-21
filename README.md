# CropMonitor
Repository of code written for the CVAR crop monitoring project

## Usage  
To simulate the drone, you need to install px4 sitl by following the instructions here: https://docs.px4.io/main/en/simulation/gazebo.html  
Once fully installed, you can run ```make px4_sitl gazebo_iris_opt_flow__yosemite``` to start gazebo. This will load the yosemite world with a quadcopter in the middle, which the scripts in this repository can then pilot.  
Now that gazebo is running, run ```python simflight.py``` to start a mission. 
