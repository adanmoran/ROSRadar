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
import enum
import binascii
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
    RADAR_DETECTION_PACKAGE_LENGTH = 224 # number of bytes in one element of RadarDetection list

    # Constructor - initializes rospy Publishers for each of the topics
    def __init__(self, statusTopic, eventTopic):
        # Initialize publishers for Event and Status topics
        self.statuses = rospy.Publisher(statusTopic, ARS430Status, queue_size = 10)
        self.events = rospy.Publisher(eventTopic, ARS430Event, queue_size = 10)

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
        # Unpack the statusData into an ARS430Status
        (_CRC, _Len, _SQC, _PartNumber, _AssemblyPartNumber, _SWPartNumber, _SerialNumber, _BLVersion1, _BLVersion2,
         _BLVersion3, _BLCRC, _SWVersion1, _SWVersion2,_SWVersion3, _SWCRC, _UtcTimeStamp, _TimeStamp, _CurrentDamping,
         _OpState, _CurrentFarCF, _CurrentNearCF, _Defective, _SupplVoltLimit, _SensorOffTemp, _GmMissing, _TxOutReduced,
         _MaximumRangeFar, _MaximumRangeNear
         ) = struct.unpack("!HHBQQQBBBBLBBBLQLLBBBBBBBBHH", statusData)
        _SWVersion = merge24(_SWVersion1, _SWVersion2, _SWVersion3)
        _BLVersion = merge24(_BLVersion1, _BLVersion2, _BLVersion3)

        # Copy the data over into the ARS430Status object
        packet = ARS430Status()
        packet.CRC=_CRC
        packet.Len = _Len
        packet.SQC = _SQC
        packet.PartNumber = _PartNumber
        packet.AssemblyPartNumber =_AssemblyPartNumber
        packet.SWPartNumber =_SWPartNumber
        packet.SerialNumber = _SerialNumber
        packet.BLVersion=_BLVersion
        packet.BLCRC=_BLCRC
        packet.SWVersion=_SWVersion
        packet.SWCRC=_SWCRC
        packet.UTCTimestamp=_UtcTimeStamp
        packet.Timestamp=_TimeStamp
        packet.CurrentDamping=_CurrentDamping
        packet.Opstate=_OpState
        packet.CurrentFarCF=_CurrentFarCF
        packet.CurrentNearCF=_CurrentNearCF
        packet.Defective=_Defective
        packet.SupplyVoltLimit=_SupplVoltLimit
        packet.SensorOffTemp=_SensorOffTemp
        packet.GmMissing=_GmMissing
        packet.TxOutReduced=_TxOutReduced
        packet.MaximumRangeFar=_MaximumRangeFar
        packet.MaximumRangeNear=_MaximumRangeNear

        # Return the ARS430Status object for publishing
        return packet

    def __unpackRadarDetections(self, packet, detection_bytes):
        packet.RadarDetections = []
        index = 0
        length = len(detection_bytes)
        # Unpack all RadarDetection bytes from the UDP data, which comes in detection_bytes
        while(i<length):

            # Create a new RadarDetection message
            detection = RadarDetection()
            # Unpac the UDP data to get the Radar Detection signals
            chunk=detection_bytes[i:i+ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH]
            (f_Range, f_VrelRad, f_AzAng0, f_AzAng1, f_ElAng, f_RCS0, f_RCS1, f_Prob0, f_Prob1, f_RangeVar, f_VrelRadVar, f_AzAngVar0, f_AzAngVar1, f_ElAngVar, f_Pdh0, f_SNR) = struct.unpack("!HhhhhhhBBHHHHHBB", chunk)

            # TODO: Convert these from int (or uint) to their actual physical value.

            # Fill in the RadarDetection message with the unpacked signals
            # TODO: RadarDetection.msg should emit floats, doubles, etc... when necessary, not uints
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
            # Add this RadarDetection message to the ARS430Event message
            packet.RadarDetections.append(detection)
            # Go to the next Radar Detection segment of the UDP data to unpack it
            i+=ARS430Publisher.RADAR_DETECTION_PACKAGE_LENGTH

        # Return the packet, which is now filled with detections
        return packet

    def __unpackEvent(self, eventData):
        # Split the Event and RadarDetection sections into two.
        # The event only data is 32 bytes long
        # The RadarDetection list is the rest of the package
        eventOnlyData=eventData[:ARS430Publisher.RADAR_DETECTION_START]

        # Unpack the data for Events using struct.unpack
        (RDI_CRC,RDI_Len,RDI_SQC,RDI_MessageCounter,
         RDI_UtcTimeStamp,RDI_TimeStamp,RDI_MeasurementCounter,
         RDI_CycleCounter,RDI_NofDetections,RDI_Vambig,
	 RDI_CenterFrequency,RDI_DetectionsInPacket) =struct.unpack("!HHBBQLLLHhBB",eventOnlyData)

        # Convert the unpacked data into an ARS430Event type
	packet=ARS430Event()
	
        packet.CRC=RDI_CRC
	packet.Len=RDI_Len
	packet.SQC=RDI_SQC
	packet.MessageCounter=RDI_MessageCounter
	packet.UtcTimeStamp=RDI_UtcTimeStamp
	packet.TimeStamp=RDI_TimeStamp
	packet.MeasureCounter=RDI_MeasureCounter
	packet.CycleCounter=RDI_CycleCounter
	packet.NofDet=RDI_NofDetections
	packet.Vambig=RDI_Vambig
	packet.CenterFreq=RDI_CenterFrequency
	packet.DetInPack=RDI_DetectionsInPacket

	#calling the class Radar Detection for the data starting from the 256th bit/32th byte position 
        packet = self.__unpackRadarDetections(self, packet, eventData[ARS430Publisher.RADAR_DETECTION_START:])

        # Return the ARS430Event message
        return packet

    # Unpack a UDPMsg into the ARS430Msg type, and returns that ARS430Msg as
    # well as the type
    def unpackAndPublish(self, udpData):
        # Determine what header is in this UDP packet
        headerType = self.__findHeader(udpData)

        # Separate the header and the data itself
        data = udpData[ARS430Publisher.HEADER_LEN:]

        # There is no switch-case in python :(
        # Unpack the relevant data and publish it to the topics
        if headerType == ARS430Publisher.Headers.STATUS:
            status = self.__unpackStatus(data)
            self.statuses.publish(status)
            return (status, headerType)
        else:
            event = self.__unpackEvent(data)
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
    rospy.loginfo(rospy.get_caller_id() + "I heard a message from %s", str(data.ip))

    # TODO: Only publish data if it comes from a desired IP address, which can be stored
    # in the publisher itself
    packet, type = arsPublisher.unpackAndPublish(data.data)

    # TODO: Do stuff with the packet

def listener():
    rospy.init_node('ars430', anonymous=True)

    # Initialize a publisher and make it available to the callback function
    global arsPublisher # modify the global variable

    arsPublisher = ARS430Publisher('ars430/status', 'ars430/event')

    # Listen for UDPMsg types and call the callback function
    rospy.Subscriber('rosudp/31122', UDPMsg, callback)

    # spin() stops rospy from exiting until CTRL-C is done
    rospy.spin()

if __name__ == '__main__':
    listener()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
