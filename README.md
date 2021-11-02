# PozyxToRos
A ROS publisher that publish the coordinates from a Pozyx tag connected with USB. This publisher will output in the form a [Vector3](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html)
message.

## Installation 
1. Clone the repository into the src folder of your ROS workspace. 
2. Run `catkin_make` at the workspace root.

## Example usage

How to run the publisher:
```bash
rosrun pozyx_to_ros PozyxToRos.py
```

Echoing output from the publisher: 
```bash
rostopic echo /pozyx
````
