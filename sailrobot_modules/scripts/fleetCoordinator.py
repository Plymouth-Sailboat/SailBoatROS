#!/usr/bin/env python
# -*- coding: utf-8 -*-


###################################################################################################
###################################################################################################
#####################################     OVERVIEW     ############################################
###################################################################################################
###################################################################################################

#The purpose of this file is to manage the communications between the remote control computer
#(Coordinator) and the sailboats that are to be used as a fleet.
#The Coordinator broadcasts data to all the sailboats that are connected to the same XBee network.
#Therefore, each boat of the network may have access to the data relative to each other sailboat,
#while they are not too far from the Coordinator.

#Stage 1: The Coordinator identifies its XBee device and accepts the connections of the sailboats
#         while the number of sailboats is lower than the variable "expected", that sets the fleet
#         size. Once all are connected, the Coordinator sends a confirmation message and informs
#         them of the fleet size.

#Stage 2: The Coordinator prepares the ROS topics that will be use for communicating data to and
#         from the operator:
#         - 2 subscribers that communicate the data coming from the operator:
#             ->one giving the control mode (ie automatic, keyboard control, other...)
#             ->one giving the commands necessary for this control mode

#Stage 3: Synchronised transmission begins. The transmission loop of the Coordinator behaves in
#         the same way (fleetSize-1) times out of (fleetSize) and in a different way the last
#         time. In that case, it listens one message from the XBee (coming from a sailboat),
#         stores it in a list in which each index corresponds to one boat and then
#         sends the messages stored in the list. Else, it only listens and stores.

#Stage 4: The transmission loop ends when the Coordinator is shut down: it sends a shutdown signal
#        that makes the loop end on the sailboats' side.



#Non default dependences: rospy, pyudev

###################################################################################################
###################################################################################################
###################################################################################################
###################################################################################################

import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3
from gps_common.msg import GPSFix

import serial
from time import time, sleep

import pyudev


###################################################################################################
#    To execute when a message to transmit is received by the subscribers.
###################################################################################################

def targetTransmission(data):
    global targetString
    targetString = data.data

def modeTransmission(data):
    global modeString
    modeString = data.data


###################################################################################################
#    Check message validity and clean it for downstream use (remove begin and end signals + size)
###################################################################################################

def is_valid(line):
    a = (len(line) > 2)
    if a:
        b = (line[0] == '#')
        c = (line[-1] == '=')

        if b and c:
            msg = line[0:-1]
            msg = msg.replace('#','')
            msg = msg.replace('=','')

            try :
                size = int(msg.split('_')[0])

                if size == len(msg):
                    msg = msg[5:]
                    return True, msg
                else:
                    return False, ''

            except:
                return False, ''

        else:
            return False, ''
    else:
        return False, ''



###################################################################################################
###################################################################################################
#    Main
###################################################################################################
###################################################################################################



def run():
    expected_fleet_size = 2
    receiving_freq = 10. #Set the speed of the transmission loops

###################################################################################################
#    Look for XBee USB port, to avoid conflicts with other USB devices
###################################################################################################
    rospy.init_node('coordinator', anonymous=True)
    rospy.loginfo("Looking for XBee...")

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    ser = serial.Serial(usbPort,baudrate=57600, timeout = 2)


###################################################################################################
#    Get local XBee ID (should be 0, convention for Coordinator)
###################################################################################################

    # Enter XBee command mode
    ser.write('+++')
    rospy.sleep(1.2)
    ser.read(10)

    # Get local XBee adress
    ser.write('ATMY\r')
    rospy.sleep(0.1)
    ans = ser.read(10)
    ID = eval(ans.split('\r')[0])

    # Exit XBee "command mode"
    ser.write('ATCN\r')

    if ID == 0:
        rospy.loginfo("\nHello,\nI am Coordinator " + str(ID)+'\n')
    else:
        raise Exception("This XBee device is not the coordinator of the network,\nlook for the XBee device stamped with 'C'.")

###################################################################################################
#    Look for the boats connected in the XBee network
###################################################################################################

    # To check we have all the boats connected
    connected = [] #list of connected boats IDs

    while not rospy.is_shutdown() and len(connected) < expected_fleet_size:
        #Read a connection message from a sailboat
        c = ser.read(1)
        line = ''
        while c != '#' and not rospy.is_shutdown():
            c = ser.read(1)

        while c != '=' and not rospy.is_shutdown():
            line += c
            c = ser.read(1)
        line += c

        #Transmission checks, details in the transmission loop part below
        check, msgReceived = is_valid(line)

        if check:
            words = msgReceived.split()
            IDboat = int(words[-1])

#           if the sailboat is not connected yet, we connect it
            if IDboat not in connected:
                connected.append(IDboat)
                rospy.loginfo('|'+msgReceived+'|')

    fleetSize = len(connected)

    #link each ID to a minimal line number in the data storing structure
    linkDict = {connected[i]:i for i in range(fleetSize)}

    ser.write(str(fleetSize)+'_'+str(connected)+'_Connected'+ '\n')
    rospy.loginfo("Got boats " + str(connected)+' connected\n')

    sleep(5)


###################################################################################################
#    Initialisation
###################################################################################################
    #Variables storing the data received by the subscribers
    global targetString, modeString
    targetString, modeString = 'nan, nan', '0'


    emission_freq = receiving_freq/fleetSize #Frequency of emission for the Coordinator
    rate = rospy.Rate(receiving_freq)
    ser.timeout = 0.1

    compteur = 0

    pub_send_gps = [rospy.Publisher("xbee_send_gps_"+str(i), GPSFix, queue_size = 0) for i in connected]

    pub_send_euler_angles = [rospy.Publisher("xbee_send_euler_angles_"+str(i), Vector3, queue_size = 0) for i in connected]

    pub_send_line_begin = [rospy.Publisher("xbee_send_line_begin_"+str(i), Pose2D, queue_size = 0) for i in connected]

    pub_send_line_end = [rospy.Publisher("xbee_send_line_end_"+str(i), Pose2D, queue_size = 0) for i in connected]

    GPSdata = GPSFix()
    eulerAnglesData = Vector3()
    lineStartData, lineEndData = Pose2D(), Pose2D()


###################################################################################################
#Subscribe to the topics that send the data to communicate to the sailboats.
#This data comes from the operator's control systems (keyboard control...)
###################################################################################################

#    Receives the data relative to the target point
#    (depends on controlMode, common to all boats)
    rospy.Subscriber('commands', String, targetTransmission)

#    Receives the string indicator of the control mode
    rospy.Subscriber('controlMode', String, modeTransmission)



###################################################################################################
# Transmission Loop
###################################################################################################

    #For statistics and synchronisation
    emission = -1

    #Data storing structure
    received = ['ID_nothing_nothing_nothing_nothing_nothing_nothing']*fleetSize


    while not rospy.is_shutdown():
        emission += 1
        c = ''
        line = ''
        loopTime = time()
####################################################################################################
# Receive useful data from the sailboats
# Frame received:
# "#####msgSize_ID_windForceString_windDirectionString_gpsString_eulerAnglesString_lineBeginString_lineEndString=====\n"
####################################################################################################

        #If available, read a line from the XBee
        while c!= '#' and (time()-loopTime)<(1/receiving_freq) and not rospy.is_shutdown():
            c = ser.read(1)
        msgTime = time()
        if c == '#':
            while c != '=' and (time()-msgTime)<(1/receiving_freq) and not rospy.is_shutdown():
                c = ser.read(1)
                line += c


        if "Hello" in line:
            continue

#        rospy.loginfo('Line:\n|'+line+'|')

        # Check message syntax and checkSum and clean the message to use only the useful data
        check, msgReceived = is_valid(line)

        if check:
#            rospy.loginfo("Received\n|" +msgReceived+'|\n')
            compteur += 1

            #Organise the incoming data in the storing structure
            msgData = msgReceived.split('_')

            IDboat = int(msgData[0])
            received[linkDict[IDboat]-1] = msgReceived

            GPSframe = msgData[3]

            tmpEuler = msgData[4].split(',')
            eulerAnglesData.x = float(tmpEuler[0])
            eulerAnglesData.y = float(tmpEuler[1])
            eulerAnglesData.z = float(tmpEuler[2])

            tmpStartLine = msgData[5].split(',')
            lineStartData.x = float(tmpStartLine[0])
            lineStartData.y = float(tmpStartLine[1])
            lineStartData.theta = float(tmpStartLine[2])

            tmpEndLine = msgData[6].split(',')
            lineEndData.x = float(tmpEndLine[0])
            lineEndData.y = float(tmpEndLine[1])
            lineEndData.theta = float(tmpEndLine[2])

	    data = GPSframe.split(',')
            if GPSframe != "nothing" and data[0] == '$GPGGA' and data[2] != '':
                

                #header
                now = rospy.get_rostime()
                GPSdata.header.stamp.secs = now.secs
                GPSdata.header.stamp.nsecs = now.nsecs
                GPSdata.header.frame_id = '/xbee_send_gps'


                status = 1
                gps_time = float(data[1][0:2])*3600 + float(data[1][2:4])*60 + float(data[1][4:])
                latitude = float(data[2][0:(len(data[2])-7)])+float(data[2][(len(data[2])-7):])/60
                if data[3] == 'S':
                    latitude = -latitude
                longitude = float(data[4][0:(len(data[4])-7)])+float(data[4][(len(data[4])-7):])/60
                if data[5] == 'W':
                    longitude = -longitude
                frame_type = float(data[6])
                nbSat = float(data[7])
		if data[8] != '':                
		 hdop = float(data[8])
                altitude = float(data[9].split(',')[0])


                #status
                GPSdata.status.header.seq = GPSdata.header.seq
                GPSdata.status.header.stamp.secs = now.secs
                GPSdata.status.header.stamp.nsecs = now.nsecs
                GPSdata.status.header.frame_id = 'GPGGA'
                GPSdata.status.satellites_used = nbSat
                GPSdata.status.status = status

                #autres
                GPSdata.latitude = latitude
                GPSdata.longitude = longitude
                GPSdata.altitude = altitude
                GPSdata.time = gps_time
                GPSdata.hdop = hdop

                pub_send_gps[linkDict[IDboat]].publish(GPSdata)


            if eulerAnglesData.x != -999:
                pub_send_euler_angles[linkDict[IDboat]].publish(eulerAnglesData)

            if lineStartData.x != -999:
                pub_send_line_begin[linkDict[IDboat]].publish(lineStartData)

            if lineEndData.x != -999:
                pub_send_line_end[linkDict[IDboat]].publish(lineEndData)




        if not check:
#            rospy.loginfo("Could not read\n|"+line+'|')
            pass



        if emission%fleetSize == 0:
            #We are supposed to have the data of every boat at this point.
            #Logically, only a transmission failure (simultaneous talk, ...)
            #can prevent that.

##########################################################################################################################################
# Send useful data to the sailboats
# Frame emitted:
# "#####msgSize_ID1_windForceString1_windDirectionString1_gpsString1_eulerAnglesString1_lineBeginString1_lineEndString1_ID2_..._targetString_modeString=====\n"
##########################################################################################################################################

            #Collect the data from each boat and the operator and gather them in one string
            #Creating the core message
            receivedLines = ''
            for line in received:
                receivedLines += line+'_'

            msg = receivedLines+targetString+'_'+modeString

            #Generating the checkSum message control
            size = str(len(msg)+5)
            for i in range(len(size),4):
                size = '0'+size

            msg = "#####"+size+'_'+msg+"=====\n"

            #Emit the message
            ser.write(msg)
#            rospy.loginfo("Emitted\n|" + msg + '|')

            received = ['ID_nothing_nothing_nothing_nothing_nothing_nothing']*fleetSize


#            rospy.loginfo("Emission " + str(emission//fleetSize))


#        rate.sleep()
        if fleetSize==1:
            rospy.sleep(0.1/emission_freq)
        else:
            rospy.sleep(0.4/emission_freq)






###################################################################
#   Deconnection signal
###################################################################
    rospy.loginfo("End mission, disconnection of all connected boats\n")
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    rospy.sleep(1.)
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')
    ser.write('#####**********=====\n')

    rospy.loginfo("Emitted " + str(emission//fleetSize))
    rospy.loginfo("Received "+str(compteur-2))































