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


#-------------------------------Set Robot Volume (0-100)--------------------------------
RobotVolume = 100
ret = yanshee_api.ubtSetVolume(RobotVolume)
if ret != 0:
    print("Can not set volume for robot! Error Code: %d" % ret)
    exit(4)



#--------------------------DisConnection---------------------------------            
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
