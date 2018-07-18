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


#------------------------Test Stop Voice Recognition Service------------------
ret = yanshee_api.ubtStopAsr()
if ret != 0:
    print("Can not close voice recognition service. Error code: %d" % ret)
    exit(3)
print("Voice recognition service is stopped")

#--------------------------DisConnect--------------------------------- 
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
