The Sailrobot_modules package has all the optional package used with the sailrobot.
It contains the Camera Color Detection algorithm, AIS decoding using a RTL-SDR device...  
These modules cam be used without the Sailboat.

# Nodes
## Camera Nodes
### camera_dist
Gives a distance to an object detected by color. The distance is given depending on the known size of the object.

## AIS Nodes
### rtl_sdr_connect
This node connects to a RTL-SDR device and publishes the NMEA strings

### ais_parser
This node parses the NMEA data

### ais_decode
This node decodes the NMEA data and publishes custom ROS AIS_messages from the sailrobot_custom_msg
