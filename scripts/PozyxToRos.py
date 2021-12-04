#!/usr/bin/env python
from PozyxLocalizer import PozyxLocalizer
import gepmetry_msgs.msg.Pose as MsgPose
import rospy
import pypozyx

def pozyxToRos():
    """
    Running a ROS publisher that publishes the position from pozyx as a ROS Pose.
    """
    path = __file__[0:-22] + "/config/PozyxConfig.yaml"
    print("Using config file:", path)

    localizer = PozyxLocalizer(anchors = path)
    pub = rospy.Publisher("pozyx", MsgPose, queue_size = 10)
    rospy.init_node("PozyxToRos", anonymous = True)
    refreshRate = rospy.Rate(10)

    while not rospy.is_shutdown():
        localizer.loop()
        pub.publish(localizer.pose) 
        refreshRate.sleep()

if __name__ == "__main__":
    try:
        pozyxToRos()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except pypozyx.core.PozyxConnectionError:
        print("Could not connect to Pozyx device")