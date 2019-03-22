#!/usr/bin/env python

###########
# Imports #
###########
import rospy
import roslib; roslib.load_manifest('ars430')
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

     def get_ip(self):
         return ip

    # Find the header in the UDPMsg.data object, and return
    # a Headers enum corresponding to that header type
    def __findHeader(self, data):
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

    def __unpackStatus(self, statusData):
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

    def __unpackRadarDetections(self, packet, detection_bytes, numDetections):
        packet.DetectionList = []
        index = 0
        length = len(detection_bytes)
        # Unpack all RadarDetection bytes from the UDP data, which comes in detection_bytes
        while(index<length and (index/ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH) <= numDetections):

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
            detection.ProbabilityAz0 = f_Prob0/254.0                         # (unitless)
            detection.ProbabilityAz1 = f_Prob1/254.0                         # (unitless)
            detection.RangeVariance = f_RangeVar/65534.0 * 10               # m^2
            detection.RadialVelocityVariance = f_VrelRadVar/65534.0 * 10    # (m/s)^2
            detection.Az0Variance = f_AzAngVar0/65534.0                     # rad^2
            detection.Az1Variance = f_AzAngVar1/65534.0                     # rad^2
            detection.ElAngleVariance = f_ElAngVar/65534.0                    # rad^2
            # TODO: PdH0 is a bitstream of flags, not an actual probability value. Split it into those bits and set flags accordingly.
            detection.ProbabilityFalseDetection = f_Pdh0/254.0               # (unitless)
            detection.SNR = (f_SNR + 110.0)/10.0               # dBr
            # Add this RadarDetection message to the ARS430Event message
            packet.DetectionList.append(detection)
            # Go to the next Radar Detection segment of the UDP data to unpack it
            index+=ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH

        # Return the packet, which is now filled with detections
        return packet

    def __unpackEvent(self, eventData):
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
            packet = self.__unpackRadarDetections(packet, eventData[ARS430Publisher.RADAR_DETECTION_START:], RDI_DetectionsInPacket)
        else:
            packet.DetectionList = []

        # Return the ARS430Event message
        return packet

    # Unpack a UDPMsg into the ARS430Msg type, and returns that ARS430Msg as
    # well as the type
    def unpackAndPublish(self, udpData):
        # Determine what header is in this UDP packet
        headerType = self.__findHeader(udpData)

        # Separate the header and the data itself
        data = udpData[ARS430Publisher.HEADER_LEN:]
        # TODO: publish should emit the IP address of the radar

        # There is no switch-case in python :(
        # Unpack the relevant data and publish it to the topics
        if headerType == ARS430Publisher.Headers.STATUS:
            status = self.__unpackStatus(data)
            # TODO: fill status.sourceIP
            self.statuses.publish(status)
            return (status, headerType)
        else:
            event = self.__unpackEvent(data)
            event.EventType = headerType.value;
            # TODO: fill event.sourceIP
            # Only publish a packet if it had any detections in it
            if event.DetInPack > 0:
                self.events.publish(event)
            return (event, headerType)


# TODO: Convert information into XYZ coordinates in some meaningful way and publish to topic "ars430/points". This needs to be clarified more in to how we use stuff like probability, variance, elevation, radial distance, velocity, etc"

# Global variable corresponding to a publisher for this node
arsPublisher = None

# Callback function for the subscriber
def callback(data):
    # Declare that we are using the global arsPublisher object
    global arsPublisher

    # Tell people we heard a UDP message!
    # rospy.loginfo(rospy.get_caller_id() + "I heard a message from %s", str(data.ip))
    if (arsPublisher.get_ip() == data.ip):

        # doneTODO: Only publish data if it comes from a desired IP address, which can be stored
        # in the publisher itself
        packet, type = arsPublisher.unpackAndPublish(data.data)

    # TODO: Do stuff with the packet
    # TODO: Convert the range and azimuth of the packet into xyz coordinates and send to Rviz IF it is an event packet and has detections

def listener():
    rospy.init_node('ars430', anonymous=True)

    # Initialize a publisher and make it available to the callback function
    global arsPublisher # modify the global variable

    arsPublisher = ARS430Publisher('192.168.1.2', 'ars430/status', 'ars430/event')

    # Listen for UDPMsg types and call the callback function
    rospy.Subscriber('rosudp/31122', UDPMsg, callback)

    # spin() stops rospy from exiting until CTRL-C is done
    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
