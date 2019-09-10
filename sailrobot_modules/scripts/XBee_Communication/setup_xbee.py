#! /usr/bin/env python3

import serial
import yaml
import sys
import time
import math
import numpy as np

import pyudev
from digi.xbee.devices import XBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.util import utils

from optparse import OptionParser

#

help = """
%prog [options] my_adr

setup_xbee.py is a configuration script for Xbees.  It takes
factory fresh xbee and programs it to work with your rosserial network.
If XBee is not factory fresh, use Digi's X-CTU software to program it.

    my_adr:   MY address is the 16 bit address of this xbee in the
              network. This must be a unique address in the network.
              This address is always 0 for the coordinator.  """
parser = OptionParser(usage=help)


parser.add_option('-P', '--pan_id', action="store", type="string", dest="pan_id", default="1221", help="Pan ID of the xbee network.  This ID must be the same for all XBees in your network.")
parser.add_option('-c', '--channel', action="store", type="string", dest="channel", default="11", help="Frequency channel for the xbee network. The channel value must be the same for all XBees in your network.")
parser.add_option('-C', '--coordinator', action="store_true", dest="coordinator", default=False, help="Configures the XBee as Coordinator for the network.  Only make the XBee connected to the computer a coordiantor.")
parser.add_option('-n', '--name', action="store", type="string", dest="name", default="Xbee", help="Set the name of the XBee")


baud_lookup= { 1200   : 0,
			   2400   : 1,
			   4800   : 2,
			   9600   : 3,
			   19200  : 4,
			   38400  : 5,
			   57600  : 6,
			   115200 : 7}

if __name__ == '__main__':
    opts, args = parser.parse_args()

    if len(args) < 1:
    	print(help)
    	exit()

    baud = 57600

    #LOOK FOR XBEE
    context = pyudev.Context()
    port_name = '/dev/ttyUSB0'

    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR' in device and device['ID_VENDOR'] == 'FTDI':
            port_name = device['DEVNAME']

    my_address = int(args[0])

    #Connect to XBEE
    device = XBeeDevice(port_name,baud)
    device.open()

    dest_address = XBee64BitAddress.from_hex_string("000000000000FFFF")
    device.set_dest_address(dest_address)

    device.set_pan_id(bytearray.fromhex(opts.pan_id))

    #cmd = 'AP2,'

    device.set_parameter("AP",bytearray.fromhex("02"))
    if (opts.coordinator):
        device.set_parameter("CE",bytearray.fromhex("01"))
    else:
        device.set_parameter("CE",bytearray.fromhex("00"))
    	#cmd += 'CE1,' #API mode 2, and enable coordinator

    device.set_parameter("MY",utils.int_to_bytes(my_address))
    device.set_parameter("NI",bytearray(opts.name,'utf8'))
    device.set_parameter("BD",utils.int_to_bytes(baud_lookup[57600]))
    device.set_parameter("CH",bytearray.fromhex(opts.channel))
    device.set_parameter("RN",bytearray.fromhex("01"))
    device.set_parameter("RO",bytearray.fromhex("05"))

    device.apply_changes()
    device.write_changes()
    device.reset()
    device.close()

    # cmd += 'MY%d,'%int(args[1]) #set the xbee address
    # cmd += 'BD%d,'%baud_lookup[57600] #set the xbee to interface at 57600 baud
    # cmd += 'ID%d,'%opts.pan_id
    # cmd += 'CH%s,'%opts.channel
    # cmd += 'DL0,'
    # cmd += 'RN1,' #enables collision avoidance on first transmission
    # cmd += 'RO5,' #sets packetization timeout to 5 characters
    # cmd += 'WR' #wrtie the commands to nonvolatile memory
