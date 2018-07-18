#!/usr/bin/python
# _*_ coding: utf-8 -*-

import time
import yanshee_api


yanshee_api.ubtInitialize()
#------------------------------Connect----------------------------------------
ret = yanshee_api.ubtConnectRobot("127.0.0.1")
if (0 != ret):
	print ("Can not connect to robot.")
	exit(1)

#------------------------ReadSensorValueByAddr---------------------------------
#Have 16, 17, 18 on Yanshee_8F83
infrared_sensor = yanshee_api.UBT_ROBOTINFRARED_SENSOR_T()
iAddr = [17,18,19]
for Addr in iAddr:
    ret = yanshee_api.ubtReadSensorValueByAddr("infrared",Addr,infrared_sensor,4)#Use ctypes size
    if ret != 0:
        print("Cannot find sensor: %d" % Addr)
    else:
        print("Received new sensor value. Addr: %d \t Value: %d" % (Addr,infrared_sensor.iValue))





#---------------------------Disconnect--------------------------------------
yanshee_api.ubtDisconnectRobot("127.0.0.1")
yanshee_api.ubtDeinitialize()
