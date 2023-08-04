# dtn_sim

This package contains all necessary files for the gazebo simulation to generate a ROS data set.  
Additional scripts are used for the evaluation of the `dtn_proxy`.

Contents:

- [launch/](launch/): launch files
- [map/](map/): map files for the `Nav2` library
- [models/](models/): models for the virtual world / turtlebot
- [param/](param/): parameter files for the `Nav2` library
- [rivz/](rviz/): rviz configuration files
- [scripts/](scripts/)
  - [corePosNode.py](scripts/corePosNode.py): Publishes CORE positions in ROS
  - [extractRobotMovement.py](scripts/extractRobotMovement.py): Saves a robot's position in 0.2s steps to a file
  - [imagePublisher.py](scripts/imagePublisher.py): Publishes image files to a ROS topic
  - [parseMap.py](scripts/parseMap.py): Parses a gazebo world file and saves the position of spheres to a file (used to create waypoints)
  - [playRosbag.py](scripts/playRosbag.py): Rosbag player, that replaces timestamps with the current RosTime
  - [recordTopicDelay.py](scripts/recordTopicDelay.py): Records the delay of a ROS message from header timestamp
  - [rewriteBag.py](scripts/rewriteBag.py): Replaces DiagnosticStatus msg with DiagnosticArray msg in a ROS bag
  - [sendWaypoints.py](scripts/sendWaypoints.py): Sends waypoints created with `parseMap.py` to the `follow_waypoints` service of the `control_interface` node
- [urdf/](urdf/): URDF files for the robot
- [worlds/](worlds/): virtual worlds for gazebo

## Setup

Dependencies required to run the simulation:  

- Install gazebo
  - `sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins`
- Install Turtlebot3 dependencies
  - `sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs`

Dependencies for `playRosbag.py`:

- Install Rosbag2 mcap plugin
  - `sudo apt install ros-humble-rosbag2-storage-mcap`

## Usage

- Select the TurtleBot model at the top of the [launch file](launch/complete.launch.py)
- Run the simulation: `ros2 launch dtn_sim complete.launch.py`
