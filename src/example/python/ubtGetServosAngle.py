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

ret = yanshee_api.ubtGetServosAngle(servoinfo)
print "servoinfo.SERVO1_ANGLE = %d" %(servoinfo.SERVO1_ANGLE)
print "servoinfo.SERVO2_ANGLE = %d" %(servoinfo.SERVO2_ANGLE)
print "servoinfo.SERVO3_ANGLE = %d" %(servoinfo.SERVO3_ANGLE)
print "servoinfo.SERVO4_ANGLE = %d" %(servoinfo.SERVO4_ANGLE)
print "servoinfo.SERVO5_ANGLE = %d" %(servoinfo.SERVO5_ANGLE)
print "servoinfo.SERVO6_ANGLE = %d" %(servoinfo.SERVO6_ANGLE)
print "servoinfo.SERVO7_ANGLE = %d" %(servoinfo.SERVO7_ANGLE)
print "servoinfo.SERVO8_ANGLE = %d" %(servoinfo.SERVO8_ANGLE)
print "servoinfo.SERVO9_ANGLE = %d" %(servoinfo.SERVO9_ANGLE)
print "servoinfo.SERVO10_ANGLE = %d" %(servoinfo.SERVO10_ANGLE)
print "servoinfo.SERVO11_ANGLE = %d" %(servoinfo.SERVO11_ANGLE)
print "servoinfo.SERVO12_ANGLE = %d" %(servoinfo.SERVO12_ANGLE)
print "servoinfo.SERVO13_ANGLE = %d" %(servoinfo.SERVO13_ANGLE)
print "servoinfo.SERVO14_ANGLE = %d" %(servoinfo.SERVO14_ANGLE)
print "servoinfo.SERVO15_ANGLE = %d" %(servoinfo.SERVO15_ANGLE)
print "servoinfo.SERVO16_ANGLE = %d" %(servoinfo.SERVO16_ANGLE)
print "servoinfo.SERVO17_ANGLE = %d" %(servoinfo.SERVO17_ANGLE)

#--------------------------DisConnect--------------------------------- 
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

