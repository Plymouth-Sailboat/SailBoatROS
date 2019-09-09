#!/usr/bin/env python3
# -*- coding: utf-8 -*-


###################################################################################################
###################################################################################################
#####################################     OVERVIEW     ############################################
###################################################################################################
###################################################################################################

#The purpose of this file is to manage the communications between the sailboat where it is launched
#and the remote control computer (Coordinator), which itself broadcasts data to all the sailboats
#that are connected to the same XBee network. Therefore, each boat of the network may have access
#to the data relative to each other sailboat, while they are not too far from the Coordinator.

#Stage 1: After the Coordinator has been launched (to be done on the remote computer),
#        the sailboat identifies its Xbee device, connects itself to the XBee network and waits
#        for all other sailboats to be connected (fleet size to be managed in the Coordinator
#        file before starting). Once all are connected, the sailboat receives the fleet size and
#        goes to next stage.

#Stage 2: The sailboat prepares the ROS topics that will be used to make transmissions part more
#        convenient for downstream use.
#        - 5 subscribers that receive wind direction, wind speed, gps frame, euler angles and
#          position in a local coordinate system.
#        - 5 publishers per boat connected in the network, that communicate the data above for
#          each of them.
#        - 2 publishers that communicate the data coming from the operator:
#            ->one giving the control mode (ie automatic, keyboard control, other...)
#            ->one giving the commands necessary for this control mode

#Stage 3: Synchronised transmission begins. The sailboat first waits to receive a message from
#        the Coordinator, then reads it (sends in publishers) and waits for its time to talk
#        (to avoid simultaneous talks), and eventually sends the message containing all the data
#        from the subscribers.

#Stage 4: The transmission loop ends when the Coordinator is shut down: it sends a shutdown signal
#        that makes the loop end on the sailboats' side.


#Non default dependences: rospy, pyudev

###################################################################################################
###################################################################################################
###################################################################################################
###################################################################################################



import rospy
from digi.xbee.devices import XBeeDevice

import math
import numpy as np

import pyudev
import sys

def main():
    rospy.init_node('xbeeTest', anonymous=True)
    rate = rospy.Rate(1)
    device = XBeeDevice("/dev/ttyUSB0",57600)
    device.open()
    while not rospy.is_shutdown():
        device.send_data_broadcast("hello")
        rate.sleep()
    device.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
