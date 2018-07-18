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

#----------------------------LED API------------------------------------
#The LED will test the button color and their modes
# pcColor = 'white','red','green','blue','yellow','purple','cyan'
# pcMode = 'off','on','blink','breath','colorful'

pcType = "button"
pcColor = ['white','red','green','blue','yellow','purple','cyan']
pcMode =  ['on','blink','breath','colorful','off']
for color in pcColor:
    for mode in pcMode:
        print("Current collor %s, mode %s"% (color, mode))
        ret = yanshee_api.ubtSetRobotLED(pcType,color,mode)
        time.sleep(5)
        if ret != 0:
            print("Can not set color for robot! Error code: %d" % ret)
            exit(3)


#--------------------------DisConnect---------------------------------            
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
