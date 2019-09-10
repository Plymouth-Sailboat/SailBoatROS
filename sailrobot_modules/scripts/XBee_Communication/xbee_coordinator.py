#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose2D, Vector3, Twist
from gps_common.msg import GPSFix

from time import time, sleep
import numpy

import struct
import pyudev
from digi.xbee.devices import XBeeDevice,RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress,XBee16BitAddress
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.util import utils
from digi.xbee.exception import TimeoutException,XBeeException

import pygame
from pygame.locals import *

import logging


###################################################################################################
#    To execute when a message to transmit is received by the subscribers.
###################################################################################################

def parse(data, name):
    global lastData
    splitdata = data.split(b'#')
    dataDict = {}
    ignoreData = False
    try:
        index = splitdata.index(b'G')
        #print(splitdata[index+1].decode('utf8'))
        dataDict["NMEA"] = splitdata[index+1].decode('utf8')
    except ValueError:
        dataDict["NMEA"] = ''
        #print("G data not found")

    try:
        index = splitdata.index(b'W')
        splited = splitdata[index+1].split(b',')

        #print(struct.unpack('d',splited[0])[0])
        dataDict["windF"] = struct.unpack('d',splited[0])[0]
        #print(struct.unpack('d',splited[1])[0])
        dataDict["windT"] = struct.unpack('d',splited[1])[0]
    except ValueError:
        dataDict["windF"] = 0.0
        dataDict["windT"] = 0.0
        #print("W data not found")

    try:
        index = splitdata.index(b'E')
        splited = splitdata[index+1].split(b',')

        #print(struct.unpack('d',splited[0])[0])
        dataDict["eulerx"] = struct.unpack('d',splited[0])[0]
        #print(struct.unpack('d',splited[1])[0])
        dataDict["eulery"] = struct.unpack('d',splited[1])[0]
        #print(struct.unpack('d',splited[2])[0])
        dataDict["eulerz"] = struct.unpack('d',splited[2])[0]
    except ValueError:
        dataDict["eulerx"] = 0.0
        dataDict["eulery"] = 0.0
        dataDict["eulerz"] = 0.0
        #print("E data not found")

    try:
        index = splitdata.index(b'R')
        msg = splitdata[index+1].decode('utf8')
        ignoreData = True
        print("Executed remotely, result : " + msg)
    except ValueError:
        pass

    try:
        index = splitdata.index(b'T')
        msg = splitdata[index+1].decode('utf8')
        ignoreData = True
        print("Executed remotely, result : " + msg)
    except ValueError:
        pass

    if not ignoreData:
        lastData[name] = dataDict

def data_received(xbee_message):
    global lastDataStr
    name = xbee_message.remote_device.get_node_id()
    data = ''
    #if(int(xbee_message.data[0]) == 72):
    data = xbee_message.data
    lastDataStr[name] = data
    parse(lastDataStr[name], name)

def network_callback(remote):
    global remote_devices, device, pubGps, pubEuler, pubLB, pubLE
    if(remote.get_node_id() not in remote_devices):
        pubGps.append(rospy.Publisher("xbee"+remote.get_node_id()+"/GPS/fix", GPSFix, queue_size = 0))
        pubEuler.append(rospy.Publisher("xbee"+remote.get_node_id()+"/euler_angles", Vector3, queue_size = 0))
        pubLB.append(rospy.Publisher("xbee"+remote.get_node_id()+"/line/begin", Pose2D, queue_size = 0))
        pubLE.append(rospy.Publisher("xbee"+remote.get_node_id()+"/line/end", Pose2D, queue_size = 0))
        print("Found " + remote.get_node_id())
        #device.send_data(remote, "Hi")

    remote_devices[remote.get_node_id()] = remote

def network_finished(status):
    if status == NetworkDiscoveryStatus.ERROR_READ_TIMEOUT:
        pass
    if status == NetworkDiscoveryStatus.SUCCESS:
        pass

def send_command():
    global mode, cmd

    if(mode):
        remdev = ''
        i = 0
        for x in remote_devices:
            if(i == chosen):
                remdev = x
            i=i+1
        try:
            sep = ("#").encode('utf8')
            begC = ("C").encode('utf8')
            begM = ("M").encode('utf8')
            spa = (",").encode('utf8')
            rud = struct.pack('d',cmd.angular.x)
            sai = struct.pack('d',cmd.angular.y)
            m = struct.pack('i',mode)
            data = bytearray(begC+sep+rud+spa+sai+sep+begM+sep+m+sep)
            device.send_data(remote_devices[remdev], data)
        except TimeoutException:
            #print("timeout")
            pass
        except XBeeException:
            pass


def display(screen):
    global remote_devices, chosen, mode, cmd

    pygame.event.pump()
    keys = pygame.key.get_pressed()

    cmd.angular.x = 0.0

    if keys[K_1]:
        chosen = 0
    if keys[K_2]:
        chosen = 1
    if keys[K_3]:
        chosen = 2
    if keys[K_4]:
        chosen = 3
    if keys[K_5]:
        chosen = 4
    if keys[K_6]:
        chosen = 5
    if keys[K_7]:
        chosen = 6
    if keys[K_SPACE]:
        mode = 1 - mode
    if keys[K_LEFT]:
        cmd.angular.x = -0.7
    if keys[K_RIGHT]:
        cmd.angular.x = 0.7
    if keys[K_UP]:
        cmd.angular.y = 1.5
    if keys[K_DOWN]:
        cmd.angular.y = 0.0

    if(chosen >= len(remote_devices)):
        chosen = 0

    remdev = ''
    i = 0
    for x in remote_devices:
        if(i == chosen):
            remdev = x
        i=i+1

    screen.fill((159, 182, 205))
    font = pygame.font.Font(None, 17)
    text = font.render(remdev, True, (255, 255, 255), (159, 182, 205))
    textRect = text.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery
    screen.blit(text, textRect)
    pygame.display.update()

    if remdev not in lastData:
        return

    text2 = font.render("WindF = " + str((lastData[remdev])["windF"]), True, (255, 255, 255), (159, 182, 205))
    textRect = text2.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery+10
    screen.blit(text2, textRect)

    text2 = font.render("WindT = " + str((lastData[remdev])["windT"]), True, (255, 255, 255), (159, 182, 205))
    textRect = text2.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery+20
    screen.blit(text2, textRect)

    text2 = font.render("NMEA = " + str((lastData[remdev])["NMEA"]), True, (255, 255, 255), (159, 182, 205))
    textRect = text2.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery+30
    screen.blit(text2, textRect)


    text2 = font.render("Controlling = " + str(mode), True, (255, 255, 255), (159, 182, 205))
    textRect = text2.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery+30
    screen.blit(text2, textRect)

    if(mode):
        text2 = font.render("r = " + str(cmd.angular.x), True, (255, 255, 255), (159, 182, 205))
        textRect = text2.get_rect()
        textRect.centerx = screen.get_rect().centerx-30
        textRect.centery = screen.get_rect().centery+40
        screen.blit(text2, textRect)


        text2 = font.render("s = " + str(cmd.angular.y), True, (255, 255, 255), (159, 182, 205))
        textRect = text2.get_rect()
        textRect.centerx = screen.get_rect().centerx+30
        textRect.centery = screen.get_rect().centery+40
        screen.blit(text2, textRect)

    pygame.display.update()


###################################################################################################
###################################################################################################
#    Main
###################################################################################################
###################################################################################################



def run():
    global device
    #dev_logger = utils.enable_logger("digi.xbee.devices", logging.DEBUG)
    dev_logger = utils.disable_logger("digi.xbee.devices")
###################################################################################################
#    Look for XBee USB port, to avoid conflicts with other USB devices
###################################################################################################
    rospy.init_node('fleetCoordinator', anonymous=True)

    rospy.loginfo("Looking for XBee...")

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    #device = XBeeDevice(usbPort, 57600)
    device = XBeeDevice("/dev/ttyUSB0", 57600)
    device.open()
    print("Current timeout: %d seconds" % device.get_sync_ops_timeout())
    device.set_sync_ops_timeout(0.1)

###################################################################################################
#    Get local XBee ID (should be 0, convention for Coordinator)
###################################################################################################
    ID = utils.bytes_to_int(device.get_16bit_addr().address)

    if ID == 0:
        rospy.loginfo("\nHello,\nI am Coordinator " + str(ID)+'\n')
    else:
        raise Exception("This XBee device is not the coordinator of the network,\nlook for the XBee device stamped with 'C'.")


###################################################################################################
#    Initialisation
###################################################################################################
    #Variables storing the data received by the subscribers
    global remote_devices, pubGps, pubEuler, pubLB, pubLE, lastDataStr, lastData
    remote_devices = {}
    lastDataStr = {}
    lastData = {}
    pubGps = []
    pubEuler = []
    pubLB = []
    pubLE = []

    xnet = device.get_network()
    xnet.add_device_discovered_callback(network_callback)
    xnet.add_discovery_process_finished_callback(network_finished)
    device.add_data_received_callback(data_received)

    rate = rospy.Rate(10)

    GPSdata = GPSFix()
    eulerAnglesData = Vector3()
    lineStartData, lineEndData = Pose2D(), Pose2D()

    global chosen, mode, cmd
    mode = 0
    chosen = 0
    cmd = Twist()

    pygame.init()
    screen = pygame.display.set_mode( (640,480) )
    pygame.display.set_caption('Python numbers')


###################################################################################################
# Transmission Loop
###################################################################################################

    while not rospy.is_shutdown():
        if(xnet.is_discovery_running() is False):
            xnet.start_discovery_process()

        display(screen)

        send_command()

        rate.sleep()

    if(xnet.is_discovery_running()):
        xnet.stop_discovery_process()
    device.close()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
