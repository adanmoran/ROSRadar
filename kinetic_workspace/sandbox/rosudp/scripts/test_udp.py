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
from random import randint

HEADER_SIZE = 16

def randint8():
    return randint(0,255)
def randsint8():
    return randint(-127,127)
def randint16():
    return randint8() << 8 | randint8()
def randint24():
    return randint16() << 8 | randint8()
def randint32():
    return randint24() << 8 | randint8()
def randint64():
    return randint32() << 32 | randint32()
def randsint16():
    return randsint8() << 8 | randsint8()


def generateStatusMsg():
    CRC = randint16()
    rospy.loginfo('CRC = ' + str(CRC))

    Len = randint16()
    rospy.loginfo('Len = ' + str(Len))

    SQC = randint8()
    rospy.loginfo('SQC = ' + str(SQC))

    PartNumber = randint64()
    rospy.loginfo('PartNumber = ' + str(PartNumber))

    AssemblyPartNumber = randint64()
    rospy.loginfo('AssemblyPN = ' + str(AssemblyPartNumber))

    SWPartNumber = randint64()
    rospy.loginfo('SWPN = ' + str(SWPartNumber))

    SerialNumber = randint8()
    rospy.loginfo('SerialNumber = ' + str(SerialNumber))

    BLVersion1 = randint8()
    BLVersion2 = randint8()
    BLVersion3 = randint8()
    rospy.loginfo('BLVersion = ' + str(BLVersion1 << 16 | BLVersion2 << 8 | BLVersion3))

    BLCRC = randint32()
    rospy.loginfo('BLCRC = ' + str(BLCRC))

    SWVersion1 = randint8()
    SWVersion2 = randint8()
    SWVersion3 = randint8()
    rospy.loginfo('SWVersion = ' + str(SWVersion1 << 16 | SWVersion2 << 8 | SWVersion3))

    SWCRC = randint32()
    rospy.loginfo('SWCRC = ' + str(SWCRC))

    UTCTimestamp = randint64()
    rospy.loginfo('UTCTime = ' + str(UTCTimestamp))

    Timestamp = randint32()
    rospy.loginfo('Time = ' + str(Timestamp))

    CurrentDamping = randint32()
    rospy.loginfo('CurrentDamp = ' + str(CurrentDamping))

    OpState = randint8()
    rospy.loginfo('OpState = ' + str(OpState))

    CurrentFarCF =randint8() 
    rospy.loginfo('CurrentFCF = ' + str(CurrentFarCF))

    CurrentNearCF = randint8()
    rospy.loginfo('CurrentNCF = ' + str(CurrentNearCF))

    Defective = randint8()
    rospy.loginfo('Defective = ' + str(Defective))

    SupVoltLimit = randint8()
    rospy.loginfo('SupVL = ' + str(SupVoltLimit))

    SensorOffTemp = randint8()
    rospy.loginfo('SensorOffTemp = ' + str(SensorOffTemp))

    GMMissing = randint8()
    rospy.loginfo('GMMissing = ' + str(GMMissing))

    TXOutReduced = randint8()
    rospy.loginfo('TXOutRed = ' + str(TXOutReduced))

    MaximumRangeFar = randint16()
    rospy.loginfo('MaxRangeFar = ' + str(MaximumRangeFar))

    MaximumRangeNear = randint16()
    rospy.loginfo('MaxRangeNear = ' + str(MaximumRangeNear))

    msg = UDPMsg();
    msg.ip = '192.168.1.2'
    msg.port = 31122
    data = struct.pack('!HHBQQQBBBBLBBBLQLLBBBBBBBBHH',CRC, Len, SQC, PartNumber, AssemblyPartNumber, SWPartNumber, SerialNumber, BLVersion1, BLVersion2, BLVersion3, BLCRC, SWVersion1, SWVersion2, SWVersion3, SWCRC, UTCTimestamp, Timestamp, CurrentDamping, OpState, CurrentFarCF, CurrentNearCF, Defective, SupVoltLimit, SensorOffTemp, GMMissing, TXOutReduced, MaximumRangeFar, MaximumRangeNear);

    
    header = '\x00\xC8\x00\x00\x00\x00\x00\x69\x00\x00\x00\x00\x01\x01\x01\x00'
    msg.data = header + data

    rospy.loginfo('Status Data is ' + str(len(msg.data)) + ' bytes long')
    return msg


def generateEventMsg():
    CRC = randint16()
    rospy.loginfo('CRC = ' + str(CRC))

    Len = randint16()
    rospy.loginfo('Len = ' + str(Len))

    SQC = randint8()
    rospy.loginfo('SQC = ' + str(SQC))

    MsgCnt = randint8()
    rospy.loginfo('Message Counter = ' + str(MsgCnt))

    UtcTimeStamp = randint64()
    rospy.loginfo('UtcTimeStamp = ' + str(UtcTimeStamp))

    TimeStamp = randint32()
    rospy.loginfo('TimeSTamp = ' + str(TimeStamp))

    MeasurementCounter = randint32()
    rospy.loginfo('MeasurementCounter = ' + str(MeasurementCounter))

    CycleCounter = randint32()
    rospy.loginfo('CycleCounter = ' + str(CycleCounter))

    NofDetections = randint16()
    rospy.loginfo('NofDetections = ' + str(NofDetections))

    Vambig = randsint16()
    rospy.loginfo('Vambig = ' + str(Vambig))
    
    
    CenterFreq = randint8()
    rospy.loginfo('CenterFrequency = ' + str(CenterFreq))

    DetectionsInPacket = randint8()
    rospy.loginfo('DetectionsInPacket = ' + str(DetectionsInPacket))


    msg = UDPMsg();
    msg.ip = '192.168.1.2'
    msg.port = 31122
    data = struct.pack('!HHBBQLLLHhBB',CRC, Len, SQC, MsgCnt, UtcTimeStamp, TimeStamp, MeasureCounter, 
    CycleCounter, NofDetections, Vambig, CenterFrequency, DetectionsInPacket);

    
    header = '\x00\xDC\x00\x03\x00\x00\x00\x69\x00\x00\x00\x00\x01\x01\x01\x00'
    msg.data = header + data

    rospy.loginfo('Status Data is ' + str(len(msg.data)) + ' bytes long')
    return msg



def publish():

    # Initialize the publisher
    rospy.init_node('udptestnode', anonymous=True)
    pub = rospy.Publisher('rosudp/31122', UDPMsg, queue_size = 10)
    rate = rospy.Rate(50)

    # Generate random status data
    statusMsg = generateStatusMsg()

    
    eventMsg = generateEventMsg()

    while not rospy.is_shutdown():
        pub.publish(statusMsg)
        pub.publish(eventMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
    
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
