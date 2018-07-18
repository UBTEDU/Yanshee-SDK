#!/usr/bin/python
# _*_ coding: utf-8 -*-

import time
import yanshee_api


yanshee_api.ubtInitialize()

#------------------------------Connect----------------------------------------
ret = yanshee_api.ubtConnectRobot("127.0.0.1")
if (0 != ret):
    print ("Can not connect to robot")
    exit(1)

#-----------------------------Get all servos' angle------------------------------


servoinfo = yanshee_api.UBT_ROBOTSERVO_T()

while True:
    ret = yanshee_api.ubtRecordMotion(servoinfo)
    if 0 != ret:
        print "Read servos' angle failed"
        exit(1)
    print "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d" % (servoinfo.SERVO1_ANGLE, servoinfo.SERVO2_ANGLE, servoinfo.SERVO3_ANGLE,servoinfo.SERVO4_ANGLE, servoinfo.SERVO5_ANGLE, servoinfo.SERVO6_ANGLE, servoinfo.SERVO7_ANGLE, servoinfo.SERVO8_ANGLE, servoinfo.SERVO9_ANGLE, servoinfo.SERVO10_ANGLE, servoinfo.SERVO11_ANGLE, servoinfo.SERVO12_ANGLE, servoinfo.SERVO13_ANGLE, servoinfo.SERVO14_ANGLE, servoinfo.SERVO15_ANGLE, servoinfo.SERVO16_ANGLE, servoinfo.SERVO17_ANGLE)

#--------------------------DisConnect---------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

