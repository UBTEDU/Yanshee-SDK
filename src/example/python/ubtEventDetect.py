#!/usr/bin/python
# _*_ coding: utf-8 -*-

import yanshee_api
import time

yanshee_api.ubtInitialize()
#------------------------------Connect----------------------------------------
ret = yanshee_api.ubtConnectRobot("127.0.0.1")
if (0 != ret):
	print ("Can not connect to robot" )
	exit(1)

#----------------------- block program start ----------------------

pcEventType="button"
iTimeout = 15
pcValue = "0"

ret = yanshee_api.ubtEventDetect(pcEventType,pcValue,iTimeout)
print "Example Button Detect pcValue = %s " %(pcValue)

#----------------------- block program end ----------------------

yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()

