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

def generateRadarDetections():

    irand1 = randrange(0,300)
    print 'f_Range ='
    f_Range = ((irand1/300)*65534)
    print f_Range
    rospy.loginfo('f_range = ' + str(f_Range))


    irand2 = randrange(-150,150)
    print 'f_VrelRad ='
    f_VrelRad = ((irand2/150)*32767)
    print f_VrelRad
    rospy.loginfo('f_VrelRad = ' + str(f_VrelRad))

  
    irand3 = randrange(-3.1415927,3.1415927)
    print 'f_AzAng0='
    f_AzAng0 = ((irand3/3.1415927)*32767)
    print f_AzAng0
    rospy.loginfo('f_AzAng0 = ' + str(f_AzAng0))

    irand4 = randrange(-3.1415927,3.1415927)
    print 'f_AzAng1 ='
    f_AzAng1 = ((irand4/3.1415927)*32767)
    print f_AzAng1
    rospy.loginfo('f_AzAng1 = ' + str(f_AzAng1))


    irand5 = randrange(-3.1415927,3.1415927)
    print 'f_ElAng ='
    f_ElAng = ((irand5/3.1415927)*32767)
    print f_ElAng
    rospy.loginfo('f_ElAng = ' + str(f_ElAng))


    irand6 = randrange(-100,100)
    print 'f_RCS0 ='
    f_RCS0 = ((irand6/100)*32767)
    print f_RCS0
    rospy.loginfo('f_RCS0 = ' + str(f_RCS0))


    irand7 = randrange(-100,100)
    print 'f_RCS1 ='
    f_RCS1 = ((irand7/100)*32767)
    print f_RCS1
    rospy.loginfo('f_RCS1 = ' + str(f_RCS1))


    irand8 = randrange(0,1)
    print 'f_Prob0 ='
    f_Prob0 = ((irand8/1)*254)
    print f_Prob0
    rospy.loginfo('f_Prob0 = ' + str(f_Prob0))


    irand9 = randrange(0,1)
    print 'f_Prob1 ='
    f_Prob1 = ((irand9/1)*254)
    print f_Prob1
    rospy.loginfo('f_Prob1 = ' + str(f_Prob1))


    irand10 = randrange(0,10)
    print 'f_RangeVar ='
    f_RangeVar = ((irand10/10)*65534)
    print f_RangeVar
    rospy.loginfo('f_RangeVar = ' + str(f_RangeVar))


    irand11 = randrange(0,10)
    print 'f_VrelRadVar ='
    f_VrelRadVar = ((irand11/10)*65534)
    print f_VrelRadVar
    rospy.loginfo('f_VrelRadVar = ' + str(f_VrelRadVar))


    irand12 = randrange(0,1)
    print 'f_AzAngVar0 ='
    f_AzAngvar0 = ((irand12/1)*65534)
    print f_AzAngVar0
    rospy.loginfo('f_AzAngVar0 = ' + str(f_AzAngVar0))


    irand13 = randrange(0,1)
    print 'f_AzAngVar1 ='
    f_AzAngVar1 = ((irand13/1)*65534)
    print f_AzAngVar1
    rospy.loginfo('f_AzAngVar1 = ' + str(f_AzAngVar1))


    irand14 = randrange(0,1)
    print 'f_ElAngVar ='
    f_ElAngVar = ((irand8/1)*65534)
    print f_ElAngVar
    rospy.loginfo('f_ElAngVar = ' + str(f_ElAngVar))

 
    irand15 = randrange(0,1)
    print 'f_Pdh0 ='
    f_pdh0 = ((irand15/1)*254)
    print f_Pdh0
    rospy.loginfo('f_Pdh0 = ' + str(f_Pdh0))


    irand16 = randrange(11,36.5)
    print 'f_SNR ='
    f_SNR = ((irand16/36.5)*255)
    print f_SNR
    rospy.loginfo('f_SNR = ' + str(f_SNR))


    msg = UDPMsg();
    msg.ip = '192.168.1.2'
    msg.port = 31122
    data = struct.pack('!HhhhhhhBBHHHHHBB',f_Range, f_VrelRad, f_AzAng0, f_AzAng1, f_ElAng, f_RCS0, f_RCS1, f_Prob0, f_Prob1, f_RangeVar, f_VrelRadVar, f_AzAngVar0, f_AzAngVar1, f_ElAngVar, f_Pdh0, f_SNR)

    header = '\x00\xDC\x00\x03\x00\x00\x00\x69\x00\x00\x00\x00\x01\x01\x01\x00'
    msg.data = header + data

    rospy.loginfo('Radar Data is ' + str(len(msg.data)) + ' bytes long')
    return msg


def publish():

    # Initialize the publisher
    rospy.init_node('udptestnode', anonymous=True)
    pub = rospy.Publisher('rosudp/31122', UDPMsg, queue_size = 10)
    rate = rospy.Rate(50)

    # Generate random status data
    statusMsg = generateStatusMsg()

    
    eventMsg = generateEventMsg()


    radarMsg = generateRadarDetections()

    while not rospy.is_shutdown():
        pub.publish(statusMsg)
        pub.publish(eventMsg)
        pub.publish(radarMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
    
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
