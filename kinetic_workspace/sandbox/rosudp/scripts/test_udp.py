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
def randint16():
    return randint8() << 8 | randint8()
def randint24():
    return randint16() << 8 | randint8()
def randint32():
    return randint24() << 8 | randint8()
def randint64():
    return randint32() << 32 | randint32()

def publish():

    rospy.init_node('udptestnode', anonymous=True)

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

    rospy.loginfo('Data is ' + str(len(msg.data)) + ' bytes long')

    pub = rospy.Publisher('rosudp/31122', UDPMsg, queue_size = 10)

    rate =rospy.Rate(50)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
    
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
