#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt16, String

from lowlevel_control.sail_data import Sail_data

sail = Sail_data()


def node_publisher():
    pub = rospy.Publisher('/sail_servo', UInt16, queue_size=10)
    rospy.init_node('sail_servo', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/apparent_wind_direction', Float32, sail.update_true_wind_angle)
        node_publisher()
    except rospy.ROSInterruptException:
