# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from std_msgs.msg import String
from sensor_msgs.msg import TimeReference
from gps_common.msg import GPSFix,GPSStatus
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler
from datetime import datetime

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):

    def __init__(self):
        self.fix_pub = rospy.Publisher('GPS/fix', GPSFix, queue_size=1)
        self.nmea_pub = rospy.Publisher('GPS/NMEA', String, queue_size=1)

        self.valid_fix = False
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.current_fix = GPSFix()
        self.current_nmea = String()

        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 4.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                GPSStatus.STATUS_NO_FIX,
                GPSFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                GPSStatus.STATUS_NO_FIX,
                GPSFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                GPSStatus.STATUS_FIX,
                GPSFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                GPSStatus.STATUS_SBAS_FIX,
                GPSFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                GPSStatus.STATUS_GBAS_FIX,
                GPSFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                GPSStatus.STATUS_GBAS_FIX,
                GPSFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                GPSStatus.STATUS_GBAS_FIX,
                GPSFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False

        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(
            nmea_string)
        if not parsed_sentence:
            rospy.logdebug(
                "Failed to parse NMEA sentence. Sentence was: %s" %
                nmea_string)
            return False

        self.current_nmea.data = nmea_string

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()

        self.current_fix.header.stamp = current_time
        self.current_fix.header.frame_id = frame_id
        self.current_fix.status.header.stamp = current_time
        self.current_fix.status.header.frame_id = frame_id

        if 'GGA' in parsed_sentence:
            self.current_fix.position_covariance_type = \
                GPSFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            self.current_fix.status.status = gps_qual[1]
            self.current_fix.position_covariance_type = gps_qual[2]

            if gps_qual > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            self.current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            self.current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            self.current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            self.current_fix.hdop = data['hdop']
            self.current_fix.position_covariance[0] = (self.current_fix.hdop * self.lon_std_dev) ** 2
            self.current_fix.position_covariance[4] = (self.current_fix.hdop * self.lat_std_dev) ** 2
            self.current_fix.position_covariance[8] = (
                2 * self.current_fix.hdop * self.alt_std_dev) ** 2  # FIXME

            if not math.isnan(data['utc_time']):
                dt_time = datetime.fromtimestamp(data['utc_time']);
                self.current_fix.time = dt_time.hour*10000+dt_time.minute*100+dt_time.second;
                self.last_valid_fix_time = rospy.Time.from_sec(
                    data['utc_time'])

        if 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
        		self.current_fix.track = data['true_course']
        		self.current_fix.speed = data['speed']

        if 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    self.current_fix.status.status = GPSStatus.STATUS_FIX
                else:
                    self.current_fix.status.status = GPSStatus.STATUS_NO_FIX

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                self.current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                self.current_fix.longitude = longitude

                self.current_fix.altitude = float('NaN')
                self.current_fix.position_covariance_type = \
                    GPSFix.COVARIANCE_TYPE_UNKNOWN


                if not math.isnan(data['utc_time']):
                    self.current_fix.time = data['utc_time']


            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
        		self.current_fix.track = data['true_course']
        		self.current_fix.speed = data['speed']

        if 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        if 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                self.current_fix.dip = math.radians(data['heading'])
#                current_heading = QuaternionStamped()
#                current_heading.header.stamp = current_time
#                current_heading.header.frame_id = frame_id
#                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
#                current_heading.quaternion.x = q[0]
#                current_heading.quaternion.y = q[1]
#                current_heading.quaternion.z = q[2]
#                current_heading.quaternion.w = q[3]
#                self.heading_pub.publish(current_heading)
#        else:
#            return False
        self.fix_pub.publish(self.current_fix)
        self.nmea_pub.publish(self.current_nmea)

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        """Add the TF prefix"""
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
