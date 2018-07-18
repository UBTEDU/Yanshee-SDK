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

#---------------------------Voice recognition Start---------------------------
ret = yanshee_api.ubtStartAsr()
if ret != 0:
    print("Can not start start voice recognition. Error Code: %d" % ret)
    exit(3)
print("Voice recognition service started!")


#--------------------------DisConnect--------------------------------- 
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()




