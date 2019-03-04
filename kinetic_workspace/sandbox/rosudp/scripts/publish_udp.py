#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('rosudp')
from std_msgs.msg import String
from rosudp.msg import *

import socket
import struct
import binascii

# TODO: Set MCAST_GRP, MCAST_PORT, IP as inputs from ROS 
# Multicast ip, which is emmitted by UDP device
MCAST_GRP = '225.0.0.1'
# Multicast port on which to receive UDP messages
MCAST_PORT = 31122
# Listen to all multicast groups if true, otherwise listen only to MCAST_GRP
IS_ALL_GROUPS = False;
BUF_SIZE = 2048

# Initialize a connection to the UDP object on the given port, which is
# sent to the interface on this device with STATIC IP address given by hostIP.
def init_udp_connection(hostIP, mcastPort, mcastGrp, isAllGroups = IS_ALL_GROUPS):
    # Connect to the socket with the given data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32) 
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    if isAllGroups or mcastGrp == None:
        # On this port, receive all multicast groups
        sock.bind(('', MCAST_PORT))
    else:
        # On this port, listen only to mcastGrp
        sock.bind((mcastGrp, mcastPort))

    # Set the host information for the socket and listen to the right port
    sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(hostIP))
    sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(mcastGrp) + socket.inet_aton(hostIP))
    return sock

# Given a connected socket, read data from UDP an dpublish to the topic
# TODO: pass in the port as a parameter or read it from rospy
def publish_from(sock):
    # TODO: Change the publisher topic to contain the IP address or port
    # TODO: Determine if a queue-size of 10 is correct, or if we need more
    pub = rospy.Publisher('rosudp/' + str(MCAST_PORT), UDPMsg, queue_size = 10)
    rospy.init_node('udpnode', anonymous = True)
    # TODO: Determine what the rate is of the device and take that as an input
    rate = rospy.Rate(50) #Hz

    while not rospy.is_shutdown():
        try:
            # TODO: Make bufSize an input from ROS or make it suff. big
            data, addr = sock.recvfrom(BUF_SIZE)
            # Generate the message from the buffer
	    msg = UDPMsg()
	    msg.timestamp = rospy.get_time()
            msg.ip = str(addr)
            msg.data = data
            # Log stuff to the display
            rospy.loginfo("New Packet from " + str(addr) + " of size " + str(len(data)))
            print(binascii.hexlify(data))
            # rospy.loginfo(data)
            # Publish our data
            pub.publish(msg)
        # Handle errors gracefully
        except socket.error as e:
            rospy.logerr(e)
        except Exception, err:
            rospy.logerr(err)

        # Sleep so ROS can do other things and so this runs at given rate
#        rate.sleep()

    # Close the socket connection
    rospy.loginfo('Closing a connection to port ' + str(MCAST_PORT))
    sock.close()

# Main functionality
if __name__ == '__main__':
    # TODO: Initialize rospy node first so we can publish to the loginfo logerr
    # from InitializeUDP
    sock = init_udp_connection('192.168.1.30', MCAST_PORT, MCAST_GRP, True)
    try:
        publish_from(sock)
    except rospy.ROSInterruptException:
        pass

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
