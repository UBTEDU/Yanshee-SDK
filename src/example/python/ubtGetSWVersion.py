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


#----------------------- block program end ----------------------

yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

