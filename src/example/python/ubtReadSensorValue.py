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

#---------------------------Read Sensor Value-------------------------------

infrared_sensor = yanshee_api.UBT_ROBOTINFRARED_SENSOR_T()
ret = yanshee_api.ubtReadSensorValue("infrared",infrared_sensor,4)
if ret != 0:
    print("Can not read Sensor value. Error code: %d" % (ret))  
else:
    print("Read Sensor Value: %d" % (infrared_sensor.iValue))
    
#---------------------------Disconnect--------------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
