#!/usr/bin/env python
import PozyxLocalizer
import rospy

def pozyxToRos(): 
    localizer = PozyxLocalizer(anchors = "../config/PozyxConfig.yaml")
    pub = rospy.Publisher("pozyx", String, queue_size = 10)
    rospy.init_node("PozyxToRos", anonymous = True)
    refreshRate = rospy.Rate(10)

    while not rospy.is_shutdown():
        localizer.loop()
        pub.publish(localizer.position)
        refreshRate.sleep()

if __name__ == "__main__":
    try: 
        pozyxToRos()
    except rospy.ROSInterruptException:
        pass