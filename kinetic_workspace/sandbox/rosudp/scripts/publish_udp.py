#!/usr/bin/env python

###########
# Imports #
###########
import rospy
# TODO: replace this with our message type
from std_msgs.msg import String

import socket
import struct

# TODO: Set MCAST_GRP, MCAST_PORT, IP as inputs from ROS 
# Multicast ip, which is emmitted by UDP device
MCAST_GRP = '255.0.0.1'
# Multicast port on which to receive UDP messages
MCAST_PORT = 31122
# Listen to all multicast groups if true, otherwise listen only to MCAST_GRP
IS_ALL_GROUPS = False
BUFSIZE = 1154

# Initialize a connection to the UDP object on the given port, which is
# sent to the interface on this device with STATIC IP address given by hostIP.
# If isAllGroups is True, the mcastGrp input is ignored. Otherwise,
# listen on the mcastGrp channel.
# By default, isAllGroups is False.
def InitializeUDP(hostIP, mcastPort, mcastGrp = None, isAllGroups = IS_ALL_GROUPS):
    # Connect to the socket with the given data
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32) 
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    if isAllGroups:
        # On this port, receive all multicast groups
        sock.bind(('', MCAST_PORT))
    else:
        # On this port, listen only to mcastGrp
        sock.bind((mcastGrp, mcastPort))

    # Set the host information for the socket and listen to the right port
    sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(hostIP))
    sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(mcastGrp) + socket.inet_aton(hostIP))

def publish():
    # TODO: Change the publisher topic to contain the IP address or port
    # TODO: Change the message type from String to our UDP message
    # TODO: Determine if a queue-size of 10 is correct, or if we need more
    pub = rospy.Publisher('udpPublisher/' + str(MCAST_PORT), String, queue_size = 10)
    rospy.init_node('udpnode', anonymous = True)
    # TODO: Determine what the rate is of the device and take that as an input
    rate = rospy.Rate(10) #Hz

    while not rospy.is_shutdown():
        try:
            # TODO: Make bufSize an input from ROS or make it suff. big
            data, addr = sock.recvfrom(BUF_SIZE)
            rospy.loginfo("New Packet from " + str(addr))
            rospy.loginfo(data)
            pub.publish(data)
        except socket.error, e:
            rospy.logerr('Exception')
            hexdata = binascii.hexlify(data)
            rospy.logerr('Data = %s' % hexdata)

    # TODO: Close the socket connection
if __name__ == '__main__':
    InitializeUDP('192.168.1.30', 31122, '255.0.0.1', False)
    try:
        publish()
    except rospy.ROSInterruptException:
        pass

