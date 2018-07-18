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

#---------------------------Dooneoftheaction-----------------------------------
pcName = ['Forward','Hit left','Hit right','Left slide tackle','reset','Hit Right']
iRepeat = 1
for name in pcName:
#pcName = "Hit left"
    ret = yanshee_api.ubtStartAction(name, iRepeat)
    if ret != 0:
        print("Can not start robot action! Error Code: %d" % ret)
        exit(3)
    #print("Current action: %s" % name)
    #time.sleep(2)


#---------------------------Disconnect--------------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
