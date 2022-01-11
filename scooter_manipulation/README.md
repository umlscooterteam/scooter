# Scooter manipulation

This package is responsible for manipulation.

## Driver installation
### Install MoveIt
```sudo apt install ros-galactic-moveit```

### Install UR5 driver
[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver]()

We recommend installing this in a separate workspace as per the instructions in the driver's README.

For some reason, I ended up with a build error that was solved by removing the overlaid ```control_msgs``` package. If
this happens to you, simply delete it from your catkin workspace and it will fall back to the ```control_msgs``` package
already installed on your system.

You may also be missing some dependencies for some reason that can be installed as follows:

```
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers 
```