#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('rosudp')
from std_msgs.msg import String
from rosudp.msg import UDPMsg

import struct
import enum
from enum import Enum

try:
    from enum import auto
except ImportError:
    __my_enum_auto_id = 0
    def auto():
        global __my_enum_auto_id
        i = __my_enum_auto_id 
        __my_enum_auto_id += 1
        return i

# Class for the ARS430, which contains functions for
# unpacking the UDPMsg from the ARS430 radar and turning it into
# a meaningful object here. It also contains a method for emitting
# ARS430Msg to a given topic.
class ARS430Publisher:
    # Enumerator defining the ARS430 header types
    class Headers(Enum):
        STATUS = auto() 
        FAR0   = auto() 
        FAR1   = auto()
        NEAR0  = auto()
        NEAR1  = auto()
        NEAR2  = auto()

    # Constructor - initializes rospy Publishers for each of the topics
    def __init__(self, statusTopic, eventTopic):
        # TODO: Replace String with an ARS430 message type
        self.statuses = rospy.Publisher(statusTopic, String, queue_size = 10)
        self.events = rospy.Publisher(eventTopic, String, queue_size = 10)

    # Unpack a UDPMsg into the ARS430Msg type, and returns that ARS430Msg as
    # well as the type
    def unpackAndPublish(self, udpData):
        headerType = findHeader(udpData)

        data = udpData[16:]

        # There is no switch-case in python :(
        if headerType == Headers.STATUS:
            status = unpackStatus(data)
            # TODO: publish the ARS430Msg type
            self.statuses.publish(status)
            return (status, headerType)
        else:
            event = unpackEvent(data)
            # TODO: publish the ARS430Msg type
            self.events.publish(event)
            return (event, headerType)

    def __findHeader(self, data):
        # TODO: Find the header in the UDPMsg.data object, and return
        # a Headers enum corresponding to that header type

        # TODO: return the actual header, not just STATUS
        return Headers.STATUS

    def __unpackStatus(self, statusData):
        # TODO: Return the ARS430StatusMsg after unpacking
        return String('TODO: Unpack the statusData into an ARS430StatusMsg')

    def __unpackEvent(self, eventData):
        # TODO: Unpack the event data, except for the radar detections
        # TODO: Find the radar detections list and send it to the function for unpacking
        # TODO: Return the ARS430EventMsg after unpacking. 
        return String('TODO: Unpack the eventData into an ARS430EventMsg')

    def __unpackRadarDetections(self, packet, detection_bytes):
        # TODO: unpack the radar detections list into the otherwise already-full packet, using the list of bytes corresponding to the RadarDetections 
        print('TODO: unpack the radar detections')
        packet.RadarDetections = []
        index = 0
        length = len(detection_bytes)
        while(i<length):
            detection = RadarDetection()
            chunk=detection_bytes[i:i+223]
            (f_Range, f_VrelRad, f_AzAng0, f_AzAng1, f_ElAng, f_RCS0, f_RCS1, f_Prob0, f_Prob1, f_RangeVar, f_VrelRadVar, f_AzAngVar0, f_AzAngVar1, f_ElAngVar, f_Pdh0, f_SNR) = struct.unpack("!HhhhhhhBBHHHHHBB", chunk)
            detection.Range = f_Range
            detection.RelativeRadialVelocity = f_VrelRad
            detection.AzimuthalAngle0 = f_AzAng0
            detection.AzimuthalAngle1 = f_AzAng1
            detection.ElevationAngle = f_ElAng
            detection.RadarCrossSection0 = f_RCS0
            detection.RadarCrossSection1 = f_RCS1
            detection.ProbablityAz0 = f_Prob0
            detection.ProbablityAz1 = f_Prob1
            detection.VarianceRange = f_RangeVar
            detection.VarianceRadialVelocity = f_VrelRadVar
            detection.VarianceAz0 = f_AzAngVar0
            detection.VarianceAz1 = f_AzAngVar1
            detection.VarianceElAng = f_ElAngVar
            detection.ProbablityFalseDetection = f_Pdh0
            detection.SignalNoiseRatio = f_SNR
            packet.RadarDetections.append(detection)
            i+=224
        return packet


# TODO: Every time a new data packet comes in of type rosudp.UDPMsg, 
# convert using struct.unpack into ARS430 structure
# TODO: Take ARS430 Structure and emit to two topics: "ars430/event" and "ars430/status"
# TODO: Also convert information into XYZ coordinates in some meaningful way and publish to topic "ars430/points". This needs to be clarified more in to how we use stuff like probability, variance, elevation, radial distance, velocity, etc"

# Global variable corresponding to a publisher for this node
arsPublisher = None

# Callback function for the subscriber
def callback(data):
    # Declare that we are using the global arsPublisher object
    global arsPublisher
    rospy.loginfo(rospy.get_caller_id() + "I heard a message from %s", str(data.ip))

    packet, type = arsPublisher.unpackAndPublish(data.data)

    # TODO: Do stuff with the packet

def listener():
    rospy.init_node('ars430', anonymous=True)

    # Initialize a publisher and make it available to the callback function
    arsPublisher = ARS430Publisher('ars430/statuses', 'ars430/events')

    # Listen for UDPMsg types and call the callback function
    rospy.Subscriber('rosudp/31122', UDPMsg, callback)

    # spin() stops rospy from exiting until CTRL-C is done
    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
