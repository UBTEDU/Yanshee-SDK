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


#---------------------------Test TTS message------------------------------
isInterrputed = 1
pcTTS = "Hello world"
ret = yanshee_api.ubtStartTts(isInterrputed,pcTTS)
if ret != 0:
    print("Can not play TTS voice. Error code: %d" % ret)
    exit(3)
print("Play TTS voice successfully!")


#--------------------------DisConnect--------------------------------- 
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
