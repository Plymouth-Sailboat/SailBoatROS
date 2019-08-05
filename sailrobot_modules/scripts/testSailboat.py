#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3

from numpy import pi

###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Initialisation
###################################################################

    rospy.init_node('testSailboat', anonymous=True)

#    Publishes a GPS standard frame
#    Contains xBoat, yBoat and others
    pub_send_GPS = rospy.Publisher('filter_send_trame_gps', String, queue_size = 2)

#    Publishes the speed of the wind
    pub_send_WIND_FORCE = rospy.Publisher('wind_force', Float32, queue_size = 2)

#    Publishes the direction of the wind
    pub_send_WIND_DIRECTION = rospy.Publisher('wind_direction', Float32, queue_size = 2)

#    Publishes the direction of the wind
    pub_send_EULER_ANGLES = rospy.Publisher('euler_angles', Vector3, queue_size = 2)

#    Publishes the direction of the wind
    pub_send_lineBegin = rospy.Publisher('regulator_send_lineBegin', Pose2D, queue_size = 2)
    pub_send_lineEnd = rospy.Publisher('regulator_send_lineEnd', Pose2D, queue_size = 2)



###################################################################
#    Test area
###################################################################

    rate = rospy.Rate(100)

    force = Float32(data = 0.1)

    direction = Float32(data = 0.1)

    gps = String(data = "$GPGGA,085153.000,5022.5187,N,00408.3332,W,2,04,2.4,44.4,M,51.5,M,,0000*73")

    euler = Vector3(x = 0.1, y = 0.2, z = 0.3)

    pos = Pose2D(x = 0.1, y = 0.1, theta = 0.0)

    while not rospy.is_shutdown():

        pos = Pose2D(x = 0.1, y = 0.1, theta = pos.theta+0.01)

        pub_send_WIND_FORCE.publish(force)
        pub_send_WIND_DIRECTION.publish(direction)
        pub_send_GPS.publish(gps)
        pub_send_EULER_ANGLES.publish(euler)
        pub_send_lineBegin.publish(pos)
        pub_send_lineEnd.publish(pos)

        rate.sleep()








