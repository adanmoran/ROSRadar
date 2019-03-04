#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('rosudp')
from std_msgs.msg import String
from rosudp.msg import UDPMsg

import struct

# TODO: Create a listener on topic "rosudp/31122"
# TODO: Every time a new data packet comes in of type rosudp.UDPMsg, 
# convert using struct.unpack into ARS430 structure
# TODO: Take ARS430 Structure and emit to two topics: "ars430/event" and "ars430/status"
# TODO: Also convert information into XYZ coordinates in some meaningful way and publish to topic "ars430/points". This needs to be clarified more in to how we use stuff like probability, variance, elevation, radial distance, velocity, etc"

# Callback function for the subscriber
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard a message from %s", str(data.ip))

def listener():
    rospy.init_node('ars430', anonymous=True)

    rospy.Subscriber('rosudp/31122', UDPMsg, callback)

    #TODO Initialize a publisher and make it available to the callback function
    # OR create a class which holds the publisher and has a callback inside

    # spin() stops rospy from exiting until CTRL-C is done
    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
