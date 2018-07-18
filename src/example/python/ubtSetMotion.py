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

#---------------------------Set Robot Motion-------------------------------
# Test bow , direction front, speed 3, repeat once
pcType = "bow"
pcDirect = "front"
iSpeed = 3
iRepeat = 1
ret = yanshee_api.ubtSetMotion(pcType,pcDirect,iSpeed,iRepeat)
if ret != 0:
    print("Can not set motion for robot! Error Code: %d" % ret)
    exit(3)


#---------------------------Disconnect--------------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
