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

#--------------------------Test servo 6------------------------------------

servoinfo = yanshee_api.UBT_ROBOTSERVO_T()
servoinfo.SERVO2_ANGLE = 60
servoinfo.SERVO3_ANGLE = 40
ret = yanshee_api.ubtSetServosAngle(servoinfo, 20)


#--------------------------DisConnection--------------------------------- 
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

