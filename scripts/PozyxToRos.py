#!/usr/bin/env python
from PozyxLocalizer import PozyxLocalizer
from geometry_msgs.msg import Vector3
import rospy

def pozyxToRos():
    """
    Running a ROS publisher that publishes the position from pozyx as a Vector3.
    """
    localizer = PozyxLocalizer(anchors = "/home/pi/pozyx_ws/src/pozyx_to_ros/config/PozyxConfig.yaml")
    pub = rospy.Publisher("pozyx", Vector3, queue_size = 10)
    rospy.init_node("PozyxToRos", anonymous = True)
    refreshRate = rospy.Rate(10)

    while not rospy.is_shutdown():
        localizer.loop()
        pub.publish(Vector3(localizer.position.x, localizer.position.y, localizer.position.z))
        refreshRate.sleep()

if __name__ == "__main__":
    try: 
        pozyxToRos()
    except rospy.ROSInterruptException:
        pass
