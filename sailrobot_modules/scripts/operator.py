#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu

from numpy import pi, sign

import time
import os

###################################################################
#    Program that allows sending commands from the keyboard
#   /!\ Same command in broadcast /!\
#For testing or if need to launch again:
#   Launch  key_teleop in a separate terminal
#   sudo apt install ros-melodic-teleop-tools ros-melodic-key-teleop
#   rosrun key_teleop key_teleop.py
###################################################################



def callback(data):
    global rudder, sail, sensibilite1, sensibilite2, pubCommand, initRudder, initSail, keyboardActive, timeLastCommand, timeLastMove

    if data.angular.z != 0:
        timeLastMove = time.time()

    rudder -= sensibilite1 * sign(data.angular.z)
    sail -= sensibilite2 * sign(data.linear.x)

    sail = max(0, min(sail, pi/2))
    rudder = max(-pi/4, min(rudder, pi/4))

    if (rudder, sail) != (initRudder, initSail):
        keyboardActive = True

    commands = String(data=str(rudder)+','+str(sail))
    pubCommand.publish(commands)

    if time.time()-timeLastMove > 0.1:
        rudder = initRudder

    timeLastCommand = time.time()





###################################################################
#    Main
###################################################################


def run():

###################################################################
#    Initialisation
###################################################################

    global rudder, sail, sensibilite1, sensibilite2, pubCommand, initRudder, initSail, keyboardActive, timeLastCommand, timeLastMove

    initRudder = 0.0
    initSail = pi/2
    keyboardActive = False
    keyboardWindow = False

    rudder = initRudder
    sail = initSail
    sensibilite1 = pi/100
    sensibilite2 = pi/100

    timeLastCommand = time.time()
    timeLastMove = time.time()


    rospy.init_node('testGenerator', anonymous=True)

    rospy.Subscriber("key_vel", Twist, callback)

#    Publishes the data relative to the target point
#    (depends on controlMode, common to all boats)
    pubCommand = rospy.Publisher('commands', String, queue_size = 2)

#    Publishes the string indicator of the control mode
    pubControlMode = rospy.Publisher('controlMode', String, queue_size = 2)



###################################################################
#    Test area
###################################################################

    rate = rospy.Rate(20)

    controlMode = String(data = "0")


    while not rospy.is_shutdown():
        if keyboardActive:
            controlMode.data = "1"
        else:
            controlMode.data = "0"

        if not keyboardWindow:
            os.system("gnome-terminal --  rosrun key_teleop key_teleop.py")
            rudder = initRudder
            sail = initSail
            timeLastCommand = time.time()
            keyboardWindow = True

        pubControlMode.publish(controlMode)

        if (time.time()-timeLastCommand) > 1:
            keyboardActive = False
            keyboardWindow = False
            rudder = initRudder
            sail = initSail


        rate.sleep()








