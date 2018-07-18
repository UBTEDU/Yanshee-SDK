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
pcName = ['Forward']
iRepeat = 1
for name in pcName:
    print("Start the action now, action name: %s", name)
    ret = yanshee_api.ubtStartAction(name, iRepeat)
    if ret != 0:
        print("Can not start robot action! Error Code: %d" % ret)
        exit(3)

time.sleep(3)
print("Stop the action now")
ret = yanshee_api.ubtStopAction()
if ret != 0:
    print("Can not stop robot action. Error code: %d" % ret)
    exit(3)
else:
    print("Stop success")

#---------------------------Disconnect--------------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

