#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('rosudp')
from std_msgs.msg import String
from rosudp.msg import *

import struct

# TODO: Create a listener on topic "rosudp/31122"
# TODO: Every time a new data packet comes in of type rosudp.UDPMsg, 
# convert using struct.unpack into ARS430 structure
# TODO: Take ARS430 Structure and emit to two topics: "ars430/event" and "ars430/status"
# TODO: Also convert information into XYZ coordinates in some meaningful way and publish to topic "ars430/points". This needs to be clarified more in to how we use stuff like probability, variance, elevation, radial distance, velocity, etc"

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
