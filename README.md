# Pozyx to ROS
A ROS publisher that publish the coordinates from a Pozyx tag connected with USB. This publisher will output in the form a [Vector3](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html)
message.

This package does only contain functionality for getting the coordinates from one tag. It does not contain the full functionality of ROS.
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

## Configuration
The package can be configured with the [PozyxConfig.yaml](https://github.com/mathiasmellemstuen/pozyx_to_ros/blob/main/config/PozyxConfig.yaml) file. This configuration essentially only holds information of the pozyx tags. 

Example configuration: 
```yaml
anchors: 
  - anchor1:
    id: 0x682c
    flag: 1
    coordinates:
      x: -2612
      y: -3376
      z: 0
  - anchor2:
    id: 0x6854
    flag: 1
    coordinates:
      x: 666
      y: -3071
      z: 0
  - anchor3:
    id: 0x680b
    flag: 1
    coordinates:
      x: -2003
      y: 1008
      z: 0
  - anchor4:
    id: 0x6851
    flag: 1
    coordinates:
      x: -26
      y: 2071
      z: 0
```
