#!/usr/bin/python
# _*_ coding: utf-8 -*-

import yanshee_api
import time

yanshee_api.ubtInitialize()
#--------------------------------------------

#The robot name you want to connect
ret = yanshee_api.ubtConnectRobot("127.0.0.1")
if (0 != ret):
	print ("Can not connect to robot")
	exit(1)

#----------------------- block program start ----------------------
ver = '----------------------'
verlen = yanshee_api.UBT_ROBOT_VERSION_LEN


yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SDK, ver, verlen)
print("#### SDK version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_RASPI, ver, verlen)
print("#### Raspi version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_STM32, ver, verlen)
print("#### STM32 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS1, ver, verlen)
print("#### Servo1 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS2, ver, verlen)
print("#### Servo2 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS3, ver, verlen)
print("#### Servo3 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS4, ver, verlen)
print("#### Servo4 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS5, ver, verlen)
print("#### Servo5 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS6, ver, verlen)
print("#### Servo6 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS7, ver, verlen)
print("#### Servo7 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS8, ver, verlen)
print("#### Servo8 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS9, ver, verlen)
print("#### Servo9 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS10, ver, verlen)
print("#### Servo10 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS11, ver, verlen)
print("#### Servo11 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS12, ver, verlen)
print("#### Servo12 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS13, ver, verlen)
print("#### Servo13 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS14, ver, verlen)
print("#### Servo14 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS15, ver, verlen)
print("#### Servo15 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS16, ver, verlen)
print("#### Servo16 version = %s  verlen = %s " % (ver, verlen))
yanshee_api.ubtGetSWVersion(yanshee_api.UBT_ROBOT_SOFTVERSION_TYPE_SERVOS17, ver, verlen)
print("#### Servo17 version = %s  verlen = %s " % (ver, verlen))


#----------------------- block program end ----------------------

yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

