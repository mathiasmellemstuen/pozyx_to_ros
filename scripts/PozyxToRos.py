#!/usr/bin/env python
from PozyxLocalizer import PozyxLocalizer
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
import rospy
import pypozyx

def pozyxToRos():
    """
    Running a ROS publisher that publishes the position from pozyx as a Vector3.
    """
    path = __file__[0:-22] + "/config/PozyxConfig.yaml"
    print("Using config file: " + path)

    localizer = PozyxLocalizer(anchors = path)
    # pub = rospy.Publisher("pozyx", Vector3, queue_size = 10)
    pub = rospy.Publisher("pozyx", Pose, queue_size = 10)
    rospy.init_node("PozyxToRos", anonymous = True)
    refreshRate = rospy.Rate(10)

    while not rospy.is_shutdown():
        localizer.loop()
        # pub.publish(Vector3(localizer.position.x, localizer.position.y, localizer.position.z))
        point = Point(localizer.position.x, localizer.position.y, localizer.position.z)
        quaternion = Quaternion(localizer.orientation.x, localizer.orientation.y, localizer.orientation.z, localizer.orientation.w) 
        pub.publish(Pose(point, quaternion)) 
        refreshRate.sleep()

if __name__ == "__main__":
    try:
        pozyxToRos()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except pypozyx.core.PozyxConnectionError:
        print("Could not connect to Pozyx device")