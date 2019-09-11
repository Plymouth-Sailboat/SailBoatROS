#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from std_msgs.msg import Float32, String
from gps_common.msg import GPSFix
from geometry_msgs.msg import Pose2D, Vector3, Twist
from sensor_msgs.msg import Imu

from time import time, sleep
import numpy

import struct
import pyudev
from digi.xbee.devices import XBeeDevice,RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress,XBee16BitAddress
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.util import utils
from digi.xbee.exception import TimeoutException,XBeeException

import subprocess


###################################################################################################
#    To execute when a message to transmit is received by the subscribers.
###################################################################################################

def sub_GPS(data):
    global lastData
    lastData["NMEA"] = data.data

def sub_WIND(data):
    global lastData
    windForceString = str(math.sqrt(data.x*data.x+data.y*data.y))
    windDirectionString = str(data.theta)
    lastData["windF"] = math.sqrt(data.x*data.x+data.y*data.y)
    lastData["windT"] = data.theta

def sub_IMU(data):
    global lastData
    quaternion = (
    data.orientation.x,
    data.orientation.y,
    data.orientation.z,
    data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    eulerAnglesString = str(euler[0])+','+str(euler[1])+','+str(euler[2])
    lastData["eulerx"] = euler[0]
    lastData["eulery"] = euler[1]
    lastData["eulerz"] = euler[2]

def sub_lineBegin(data):
    global lastData
    beginString = str(data.x)+','+str(data.y)+','+str(data.theta)

def sub_lineEnd(data):
    global lastData
    endString = str(data.x)+','+str(data.y)+','+str(data.theta)

def parse(data):
    global device, modeV, pub_send_u_rudder_sail,pub_send_control_mode, rudder_sail
    splitdata = data.split(b'#')

    try:
        index = splitdata.index(b'M')
        modeV = struct.unpack('i',splitdata[index+1])[0]
    except ValueError:
        pass

    try:
        index = splitdata.index(b'C')
        splitted = splitdata[index+1].split(b',')
        rudder = struct.unpack('d',splitted[0])[0]
        sail = struct.unpack('d',splitted[1])[0]

        rudder_sail = Twist()
        rudder_sail.angular.x = rudder
        rudder_sail.angular.y = sail
    except ValueError:
        pass
    try:
        index = splitdata.index(b'S')
        process = subprocess.run(splitdata[index+1].decode('utf8').split(), stdout=subprocess.PIPE)
        output = process.stdout
        try:
            device.send_data(RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("00")), bytearray(("R#").encode('utf8')+output))
        except TimeoutException:
            print("timeout")
    except ValueError:
        pass

def data_received(xbee_message):
    global lastDataStr
    #name = xbee_message.remote_device.get_node_id()
    parse(xbee_message.data)

    data = xbee_message.data.decode("utf8")
    lastDataStr = data

def network_callback(remote):
    global remote_devices
    remote_devices[remote.get_node_id()] = remote

def network_finished(status):
    if status == NetworkDiscoveryStatus.ERROR_READ_TIMEOUT:
        pass
    if status == NetworkDiscoveryStatus.SUCCESS:
        pass

def data_to_bytearray(data):
    nmea = data["NMEA"].encode('utf8')
    windf = struct.pack('d',data["windF"])
    windt = struct.pack('d',data["windT"])
    eulerx = struct.pack('d',data["eulerx"])
    eulery = struct.pack('d',data["eulery"])
    eulerz = struct.pack('d',data["eulerz"])
    sep = ("#").encode('utf8')
    begG = ("G").encode('utf8')
    begW = ("W").encode('utf8')
    begE = ("E").encode('utf8')
    spa = (",").encode('utf8')

    res = bytearray(begG + sep + nmea + sep + begW + sep + windf + spa + windt + sep + begE + sep + eulerx + spa + eulery + spa + eulerz + sep)
    return res



###################################################################################################
###################################################################################################
#    Main
###################################################################################################
###################################################################################################



def run():
    global device, modeV
    modeV = 0

    #dev_logger = utils.enable_logger("digi.xbee.devices", logging.DEBUG)
    dev_logger = utils.disable_logger("digi.xbee.devices")
###################################################################################################
#    Look for XBee USB port, to avoid conflicts with other USB devices
###################################################################################################
    rospy.init_node('fleetEndPoint', anonymous=True)

    rospy.loginfo("Looking for XBee...")

    context = pyudev.Context()
    usbPort = 'No XBee found'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            usbPort = device['DEVNAME']

    device = XBeeDevice(usbPort, 57600)
    #device = XBeeDevice("/dev/ttyUSB1", 57600)
    device.open()
    print("Current timeout: %d seconds" % device.get_sync_ops_timeout())
    device.set_sync_ops_timeout(0.1)

###################################################################################################
#    Get local XBee ID (should be 0, convention for Coordinator)
###################################################################################################
    ID = utils.bytes_to_int(device.get_64bit_addr().address)

    if ID == 0:
        raise Exception("\nThis Shouldn't be a Coordinator"+'\n')
    else:
        print("Hello This is " + str(ID))


###################################################################################################
#    Initialisation
###################################################################################################
    #Variables storing the data received by the subscribers
    global windForceString, windDirectionString, gpsString, eulerAnglesString, posString, beginString, endString, lastData, lastDataStr,pub_send_u_rudder_sail,pub_send_control_mode
    windForceData, windDirectionData, GPSdata, eulerAnglesData, lineStartData, lineEndData = Float32(), Float32(), String(), Vector3(), Pose2D(), Pose2D()

    remote_devices = {}
    lastDataStr = ''
    lastData = {}
    lastData["NMEA"] = ''
    lastData["windF"] = 0.0
    lastData["windT"] = 0.0
    lastData["eulerx"] = 0.0
    lastData["eulery"] = 0.0
    lastData["eulerz"] = 0.0

    #xnet = device.get_network()
    #xnet.add_device_discovered_callback(network_callback)
    #xnet.add_discovery_process_finished_callback(network_finished)
    device.add_data_received_callback(data_received)

    rate = rospy.Rate(10)

    rospy.Subscriber('/sailboat/GPS/NMEA', String, sub_GPS)
    rospy.Subscriber('/sailboat/wind', Pose2D, sub_WIND)
    rospy.Subscriber('/sailboat/IMU', Imu, sub_IMU)
    rospy.Subscriber('regulator_send_lineBegin', Pose2D, sub_lineBegin)
    rospy.Subscriber('regulator_send_lineEnd', Pose2D, sub_lineEnd)

    pub_send_u_rudder_sail = rospy.Publisher('xbee/sailboat_cmd', Twist, queue_size = 2)
    pub_send_control_mode = rospy.Publisher('xbee/mode', Float32, queue_size = 2)



###################################################################################################
# Transmission Loop
###################################################################################################

    while not rospy.is_shutdown():
        #if(xnet.is_discovery_running() is False):
        #    xnet.start_discovery_process()
        try:
            device.send_data(RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("00")), data_to_bytearray(lastData))
        except TimeoutException:
            #print("timeout")
            pass
        except XBeeException:
            pass

        if(modeV == 1):
            pub_send_u_rudder_sail.publish(rudder_sail)

        mode = Float32(data=0.)
        mode.data = modeV
        pub_send_control_mode.publish(mode)

        rate.sleep()

    #if(xnet.is_discovery_running()):
    #    xnet.stop_discovery_process()
    device.close()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
