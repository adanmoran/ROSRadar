#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('ars430')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String
from rosudp.msg import UDPMsg
from ars430.msg import ARS430Event
from ars430.msg import ARS430Status
from ars430.msg import RadarDetection

import struct
import binascii
import math
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
        STATUS = 5
        FAR0   = 0
        FAR1   = 1
        NEAR0  = 2
        NEAR1  = 3
        NEAR2  = 4

    # The ARS430 has a header length of 16 bytes
    HEADER_LEN = 16
    # The headers in byte format (hexadecimal)
    STATUS_HEADER_BYTES = '\x00\xc8\x00\x00'
    FAR0_HEADER_BYTES   = '\x00\xdc\x00\x01'
    FAR1_HEADER_BYTES   = '\x00\xdc\x00\x02'
    NEAR0_HEADER_BYTES  = '\x00\xdc\x00\x03' 
    NEAR1_HEADER_BYTES  = '\x00\xdc\x00\x04' 
    NEAR2_HEADER_BYTES  = '\x00\xdc\x00\x05' 

    # Other global information
    RADAR_DETECTION_START = 32 # byte value of Event data at which the RadarDetection list begins
    RADAR_DETECTION_PACKAGE_LENGTH = 28 # number of bytes in one element of RadarDetection list
    SERIAL_NUMBER_START = 29 # byte value of StatusData where SerialNumber begins
    SERIAL_NUMBER_LENGTH = 26 # number of bytes in the SerialNumber of Status data

    # Constructor - initializes rospy Publishers for each of the topics
    def __init__(self, ip, statusTopic, eventTopic):
        # Initialize publishers for Event and Status topics
        self.statuses = rospy.Publisher(statusTopic, ARS430Status, queue_size = 10)
        self.events = rospy.Publisher(eventTopic, ARS430Event, queue_size = 10)
        self.ip = ip
        self.nearPackets = []
        self.farPackets = []

    def get_ip(self):
         return self.ip

    # Find the header in the UDPMsg.data object, and return
    # a Headers enum corresponding to that header type
    @staticmethod
    def FindHeader(data):
        # Split the header into its components
        header = data[:ARS430Publisher.HEADER_LEN]
        headerID = header[:4]
        e2eLength = header[4:8]
        # If necessary, find out what [8:16] are

        # Determine what type of object this is
        if headerID == ARS430Publisher.STATUS_HEADER_BYTES:
            return ARS430Publisher.Headers.STATUS
        elif headerID == ARS430Publisher.FAR0_HEADER_BYTES:
            return ARS430Publisher.Headers.FAR0
        elif headerID == ARS430Publisher.FAR1_HEADER_BYTES:
            return ARS430Publisher.Headers.FAR1
        elif headerID == ARS430Publisher.NEAR0_HEADER_BYTES:
            return ARS430Publisher.Headers.NEAR0
        elif headerID == ARS430Publisher.NEAR1_HEADER_BYTES:
            return ARS430Publisher.Headers.NEAR1
        elif headerID == ARS430Publisher.NEAR2_HEADER_BYTES:
            return ARS430Publisher.Headers.NEAR2

    # Method to unpack a status-type message emitted by an ARS430 radar
    @staticmethod
    def UnpackStatus(statusData):
        def merge24(int8_part1, int8_part2, int8_part3):
            merged = (int8_part1 << 16) | (int8_part2 << 8) | int8_part3
            return merged
        # The serial number is an array of uint8s, so we need to split the unpacking into 3 sections: before SN, SN, and after SN
        before_sn = statusData[:ARS430Publisher.SERIAL_NUMBER_START]
        serial_number_end_byte = ARS430Publisher.SERIAL_NUMBER_START + ARS430Publisher.SERIAL_NUMBER_LENGTH
        _SerialNumber = statusData[ARS430Publisher.SERIAL_NUMBER_START : serial_number_end_byte]
        after_sn = statusData[serial_number_end_byte:]

        # Unpack the statusData into an ARS430Status
        # Step 1: unpack the "before SN" section
        (_CRC, _Len, _SQC, _PartNumber, _AssemblyPartNumber, _SWPartNumber) = struct.unpack("!HHBQQQ", before_sn)

        # Step 2: unpack the "after SN" section, _SerialNumber, 
        (_BLVersion1, _BLVersion2,
         _BLVersion3, _BLCRC, _SWVersion1, _SWVersion2,_SWVersion3, _SWCRC, _UtcTimeStamp, _TimeStamp, _CurrentDamping,
         _OpState, _CurrentFarCF, _CurrentNearCF, _Defective, _SupplVoltLimit, _SensorOffTemp, _GmMissing, _TxOutReduced,
         _MaximumRangeFar, _MaximumRangeNear
         ) = struct.unpack("!BBBLBBBLQLLBBBBBBBBHH", after_sn)
        # Merge the 24-byte objects together
        _SWVersion = merge24(_SWVersion1, _SWVersion2, _SWVersion3)
        _BLVersion = merge24(_BLVersion1, _BLVersion2, _BLVersion3)

        # Copy the data over into the ARS430Status object
        packet = ARS430Status()
        packet.EventType = ARS430Publisher.Headers.STATUS.value                                 # (custom by us, not continental)
        packet.CRC=_CRC                                                                         # (unitless)
        packet.Len = _Len                                                                       # (unitless)
        packet.SQC = _SQC                                                                       # (unitless)
        packet.PartNumber = _PartNumber                                                         # (unitless)
        packet.AssemblyPartNumber =_AssemblyPartNumber                                          # (unitless)
        packet.SWPartNumber =_SWPartNumber                                                      # (unitless)
        packet.SerialNumber = _SerialNumber                                                     # (unitless)
        packet.BLVersion=_BLVersion                                                             # (unitless)
        packet.BLCRC=_BLCRC                                                                     # (unitless)
        packet.SWVersion=_SWVersion                                                             # (unitless)
        packet.SWCRC=_SWCRC                                                                     # (unitless)
        packet.UTCTimestamp=_UtcTimeStamp                                                       # nsec
        packet.Timestamp=_TimeStamp                                                             # usec
        packet.CurrentDamping= ((_CurrentDamping * 0.931322575049159) - 2000000000) /100000000  # dB
        packet.Opstate=_OpState                                                                 # (unitless)
        packet.CurrentFarCF=_CurrentFarCF                                                       # (unitless)
        packet.CurrentNearCF=_CurrentNearCF                                                     # (unitless)
        packet.Defective=_Defective                                                             # (unitless)
        packet.SupplyVoltLimit=_SupplVoltLimit                                                  # (unitless)
        packet.SensorOffTemp=_SensorOffTemp                                                     # (unitless)
        packet.GmMissing=_GmMissing                                                             # (unitless)
        packet.TxOutReduced=_TxOutReduced                                                       # (unitless)
        packet.MaximumRangeFar=_MaximumRangeFar * 0.1                                           # m
        packet.MaximumRangeNear=_MaximumRangeNear * 0.1                                         # m

        # Return the ARS430Status object for publishing
        return packet

    @staticmethod
    def UnpackRadarDetections(packet, detection_bytes, numDetections):
        packet.DetectionList = []
        index = 0
        length = len(detection_bytes)
        # Unpack all RadarDetection bytes from the UDP data, which comes in detection_bytes
        while(index<length and (index/ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH) < numDetections):

            # Create a new RadarDetection message
            detection = RadarDetection()
            # Unpac the UDP data to get the Radar Detection signals
            chunk=detection_bytes[index:index+ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH]
            (f_Range, f_VrelRad, f_AzAng0,
             f_AzAng1, f_ElAng, f_RCS0, 
             f_RCS1, f_Prob0, f_Prob1, 
             f_RangeVar, f_VrelRadVar, f_AzAngVar0, 
             f_AzAngVar1, f_ElAngVar, f_Pdh0, f_SNR) = struct.unpack("!HhhhhhhBBHHHHHBB", chunk)

            # Fill in the RadarDetection message with the unpacked signals
            # Note that these signals are converted to their actual physical value
            detection.Range = f_Range/65534.0 * 300                         # meters
            detection.RelativeRadialVelocity = f_VrelRad/65534.0 * 300      # meters/s
            detection.AzimuthalAngle0 = f_AzAng0/65534.0 * (2*math.pi)      # rad
            detection.AzimuthalAngle1 = f_AzAng1/65534.0 * (2*math.pi)      # rad
            detection.ElevationAngle = f_ElAng/65534.0 * (2*math.pi)        # rad
            detection.RadarCrossSection0 = f_RCS0/65534.0 * 200             # dBm^2
            detection.RadarCrossSection1 = f_RCS1/65534.0 * 200             # dBm^2
            detection.ProbabilityAz0 = f_Prob0/254.0                        # (unitless)
            detection.ProbabilityAz1 = f_Prob1/254.0                        # (unitless)
            detection.RangeVariance = f_RangeVar/65534.0 * 10               # m^2
            detection.RadialVelocityVariance = f_VrelRadVar/65534.0 * 10    # (m/s)^2
            detection.Az0Variance = f_AzAngVar0/65534.0                     # rad^2
            detection.Az1Variance = f_AzAngVar1/65534.0                     # rad^2
            detection.ElAngleVariance = f_ElAngVar/65534.0                  # rad^2
            # PdH0 is a bitstream of flags, not an actual probability value. Split it into those bits and set flags accordingly.
            # We also map Pdh0 to a "probability", so that if it is above some threshold we
            # can check these flags to find out which ones apply
            detection.ProbabilityFalseDetection   = f_Pdh0/254.0            # (unitless)
            detection.FalseDetectionNear          = (f_Pdh0 & 0b0000001)>0  # boolean
            detection.FalseDetectionFromInference = (f_Pdh0 & 0b0000010)>0  # boolean
            detection.FalseDetectionFromSidelobe  = (f_Pdh0 & 0b0000100)>0  # boolean
            detection.BiasCorrectionInaccurate    = (f_Pdh0 & 0b0001000)>0  # boolean
            detection.ClusterNotLocalMax          = (f_Pdh0 & 0b0010000)>0  # boolean
            detection.BeamFormerMonopulseDiffer1  = (f_Pdh0 & 0b0100000)>0  # boolean
            detection.BeamFormerMonopulseDiffer2  = (f_Pdh0 & 0b1000000)>0  # boolean
            # SNR = Signal-to-noise ratio
            detection.SNR = (f_SNR + 110.0)/10.0               # dBr
            # Add this RadarDetection message to the ARS430Event message
            packet.DetectionList.append(detection)
            # Go to the next Radar Detection segment of the UDP data to unpack it
            index+=ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH

        # Return the packet, which is now filled with detections
        return packet

    @staticmethod
    def UnpackEvent(eventData):
        # Split the Event and RadarDetection sections into two.
        # The event only data is 32 bytes long
        # The RadarDetection list is the rest of the package
        eventOnlyData=eventData[:ARS430Publisher.RADAR_DETECTION_START]

        # Unpack the data for Events using struct.unpack
        (RDI_CRC,RDI_Len,RDI_SQC,RDI_MessageCounter,
         RDI_UtcTimeStamp,RDI_TimeStamp,RDI_MeasureCounter,
         RDI_CycleCounter,RDI_NofDetections,RDI_Vambig,
	 RDI_CenterFrequency,RDI_DetectionsInPacket) =struct.unpack("!HHBBQLLLHhBB",eventOnlyData)

        # Convert the unpacked data into an ARS430Event type
	packet=ARS430Event()
	
        packet.CRC=RDI_CRC                          # (unitless)
	packet.Len=RDI_Len                          # (unitless)
	packet.SQC=RDI_SQC                          # (unitless)
	packet.MessageCounter=RDI_MessageCounter    # (unitless)
	packet.UtcTimeStamp=RDI_UtcTimeStamp        # nsec
	packet.TimeStamp=RDI_TimeStamp              # usec
	packet.MeasureCounter=RDI_MeasureCounter    # (unitless)
	packet.CycleCounter=RDI_CycleCounter        # (unitless)
	packet.NofDet=RDI_NofDetections             # (unitless)
	packet.Vambig=RDI_Vambig/65534.0 * 200      # m/s
	packet.CenterFreq=RDI_CenterFrequency       # GHz
	packet.DetInPack=RDI_DetectionsInPacket     # (unitless)

        if RDI_DetectionsInPacket > 0:
            #calling the class Radar Detection for the data starting from the 256th bit/32th byte position 
            packet = ARS430Publisher.UnpackRadarDetections(packet, eventData[ARS430Publisher.RADAR_DETECTION_START:], RDI_DetectionsInPacket)
        else:
            packet.DetectionList = []

        # Return the ARS430Event message
        return packet

    # Unpack a UDPMsg into the ARS430Msg type, and returns that ARS430Msg as
    # well as the type
    @staticmethod
    def Unpack(udpData):
        # Determine what header is in this UDP packet
        headerType = ARS430Publisher.FindHeader(udpData)

        # Separate the header and the data itself
        data = udpData[ARS430Publisher.HEADER_LEN:]

        # Unpack the relevant data and publish it to the topics
        if headerType == ARS430Publisher.Headers.STATUS:
            status = ARS430Publisher.UnpackStatus(data)
            return status
        else:
            event = ARS430Publisher.UnpackEvent(data)
            event.EventType = headerType.value
            return event

    # Determine if a header is of status type
    @staticmethod
    def IsStatus(packet):
        return packet.EventType == ARS430Publisher.Headers.STATUS.value

    # determine if a header is of near type
    @staticmethod
    def IsNear( packet):
        if packet.EventType == ARS430Publisher.Headers.NEAR0.value or \
           packet.EventType == ARS430Publisher.Headers.NEAR1.value or \
           packet.EventType == ARS430Publisher.Headers.NEAR2.value:
            return True
        return False

    # Determine if a header is of far type
    @staticmethod
    def IsFar( packet):
        if packet.EventType == ARS430Publisher.Headers.FAR0.value or \
           packet.EventType == ARS430Publisher.Headers.FAR1.value:
            return True
        return False

    # Immediately publish a packet to the relevant topic
    def publishNow(self, packet):

        # Set the IP of the packet
        packet.sourceIP = self.get_ip()

        # Publish it to the relevant topics
        if ARS430Publisher.IsStatus(packet):
            self.statuses.publish(packet)
        # Only publish an event packet if it had any detections in it
        elif packet.DetInPack > 0:
            self.events.publish(packet)

    # Given an event packet and a list of previous events, do the following:
    # * if the list is empty, save the packet to the list and return nothing
    # * if the packet has the same timestamp as everything in the list, save the packet and return nothing
    # * if the packet has a different timestamp than everything in the list, return the list,
    #      then overwrite the list with the inputted packet.
    def __storeEvent(self, event, previousEventList):
        # If the list is empty, save the packet
        if not previousEventList:
            previousEventList.append(event)
            return []

        # The list is not empty; we need to compare the timestamp of the packet with the
        # timestamp of the list. For simplicity, we will only take the last element of the list
        # to compare timestamps.

        # If the timestamps match, just save the packet
        if previousEventList[-1].TimeStamp == event.TimeStamp:
            previousEventList.append(event)
            return []

        # Since the timestamps don't match, now we need to return the previous list
        # and overwrite it with a new packet
        # 1. copy the previous list over
        packetList = list(previousEventList)
        # 2. clear the previous list
        del previousEventList[:]
        # 3. overwrite with the new packet
        previousEventList.append(event)
        # 4. return the copy of the old list
        return packetList

    # Given any event type packet, this will collect NEAR and FAR packages
    # together so long as they have the same internal timestamp. It will wait until 
    # a packet of the same type with a new timestamp appears, and then return the previous list all at once.
    # Returns (wasPublished, packet) where
    # * wasPublished = true if a new collected packet list was returned
    # * packets = the new combined packet containing all detections for this timestamp (or a status packet)
    def collect(self, packet):
        # status messages are returned immediately
        if ARS430Publisher.IsStatus(packet):
            return (False, packet)

        # don't do anything if there were no detections in this packet. We don't care.
        if packet.DetInPack == 0:
            return (False, packet)

        # Save the package to the relevant list of packages.
        packetList = []
        if ARS430Publisher.IsNear(packet):
            packetList = self.__storeEvent(packet, self.nearPackets)
        else:
            packetList = self.__storeEvent(packet, self.farPackets)

        # If the packet list is empty, then the storePacket function just saved the packet. This means
        # the packet should not be combined for anything else.
        if not packetList:
            return (False, packet)

        # If the packet list is NOT empty, then we have started a new list and are supposed to 
        # emit this one. Let's collect all the packets together into one event and return it for whatever we need. We don't set CRC or Len since they don't often match. 

        combinedPacket = ARS430Event()
        combinedPacket.sourceIP = packet.sourceIP
        combinedPacket.EventType = packet.EventType
        combinedPacket.SQC = packet.SQC
        combinedPacket.MessageCounter = packet.MessageCounter
        combinedPacket.UtcTimeStamp = packet.UtcTimeStamp
        combinedPacket.TimeStamp = packet.TimeStamp
        combinedPacket.MeasureCounter = packet.MeasureCounter
        combinedPacket.CycleCounter = packet.CycleCounter
        combinedPacket.NofDet = packetList[0].NofDet
        combinedPacket.Vambig = packet.Vambig
        combinedPacket.DetInPack = 0
        combinedPacket.DetectionList = []

        for event in packetList:
            combinedPacket.DetInPack += event.DetInPack
            combinedPacket.DetectionList += event.DetectionList

        return (True, combinedPacket)

# TODO: Convert information into XYZ coordinates in some meaningful way and publish to topic "ars430/points". This needs to be clarified more in to how we use stuff like probability, variance, elevation, radial distance, velocity, etc"

# Global variable corresponding to a publisher for this node
arsPublisher = None
rvizPublisher = None

# Callback function for the subscriber
def callback(data):
    # Declare that we are using the global publisher objects
    global arsPublisher
    global rvizPublisher

    # Tell people we heard a UDP message!
    # rospy.loginfo(rospy.get_caller_id() + "I heard a message from %s", str(data.ip))
    # Only publish data if it comes from a desired IP address
    if (arsPublisher.get_ip() == data.ip):
        packet = arsPublisher.Unpack(data.data)
        arsPublisher.publishNow(packet)
        collected, jointPacket = arsPublisher.collect(packet)
        # Convert every element of the packet into XYZ marker and emit to rviz
        if collected:
            # Create a POINTS marker object with the correct header, frame name, id, etc
            marker = Marker()
            marker.header.frame_id = "/map";
            marker.header.stamp = rospy.Time.now();
            marker.ns = "ars430_points"
            marker.type = Marker.POINTS
            marker.lifetime = rospy.Duration()
            if ARS430Publisher.IsNear(jointPacket):
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
                marker.id = 0
            elif ARS430Publisher.IsFar(jointPacket):
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 1.0;
                marker.id = 1
            else:
                return
                    

            #for detection in jointPacket.DetectionList:
                # TODO: Add a point to the POINTS marker, in XYZ coordinates
                # This requires converting the points to XYZ 

#            marker.points.append(Point(0, 1, 0))
#            marker.points.append(Point(1,0,1))
#            marker.action = Marker.ADD
#            marker.pose.position.x = 0;
#            marker.pose.position.y = 0;
#            marker.pose.position.z = 0;
#            marker.pose.orientation.x = 0.0;
#            marker.pose.orientation.y = 0.0;
#            marker.pose.orientation.z = 0.0;
#            marker.pose.orientation.w = 1.0;
#            marker.scale.x = 1.0;
#            marker.scale.y = 1.0;
#            marker.scale.z = 1.0;

            # Publish the POINTS marker to rvizPublisher
            rvizPublisher.publish(marker)

def listener():
    rospy.init_node('ars430', anonymous=True)

    # Initialize a publisher and make it available to the callback function
    global arsPublisher # modify the global variable
    global rvizPublisher # modify the global rviz variable

    arsPublisher = ARS430Publisher('192.168.1.2', 'ars430/status', 'ars430/event')

    rvizPublisher = rospy.Publisher('visualization_marker', Marker, queue_size = 5)

    # Listen for UDPMsg types and call the callback function
    rospy.Subscriber('rosudp/31122', UDPMsg, callback)

    # spin() stops rospy from exiting until CTRL-C is done
    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
