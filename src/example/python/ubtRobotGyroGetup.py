#!/usr/bin/python
# _*_ coding: utf-8 -*-

import time
import yanshee_api as api

api.ubtInitialize()
#------------------------------Connect----------------------------------------


ret = api.ubtConnectRobot('127.0.0.1')
if (0 != ret):
        print ("Can not connect to robot")
        exit(1)

#---------------------------Read Sensor Value-------------------------------
gyro_size = 96
gyro_sensor = api.UBT_ROBOTGYRO_SENSOR_T()
while True:
        time.sleep(2)
        ret = api.ubtReadSensorValue("gyro",gyro_sensor, gyro_size)
        if ret != 0:
            print("Can not read Sensor value. Error code: %d" % (ret))
	    continue
        else:
            print("Read dGyroxValue : %f" % (gyro_sensor.dGyroxValue))
            print("Read dGyroyValue : %f" % (gyro_sensor.dGyroyValue))
            print("Read dGyrozValue : %f" % (gyro_sensor.dGyrozValue))
            print("Read dAccexValue : %f" % (gyro_sensor.dAccexValue))
            print("Read dAcceyValue : %f" % (gyro_sensor.dAcceyValue))
            print("Read dAccezValue : %f" % (gyro_sensor.dAccezValue))
            print("Read dCompassxValue : %f" % (gyro_sensor.dCompassxValue))
            print("Read dCompassyValue : %f" % (gyro_sensor.dCompassyValue))
            print("Read dCompasszValue : %f" % (gyro_sensor.dCompasszValue))
            print("Read dEulerxValue : %f" % (gyro_sensor.dEulerxValue))
            print("Read dEuleryValue : %f" % (gyro_sensor.dEuleryValue))
            print("Read dEulerzValue : %f" % (gyro_sensor.dEulerzValue))
            print("------------------------------------------------")
			
			
	if gyro_sensor.dEulerxValue > 160 or gyro_sensor.dEulerxValue < -160:
		print 'getup from fall back'
		api.ubtStartAction("getup_in_back", 1)
	elif gyro_sensor.dEulerxValue > -20 and gyro_sensor.dEulerxValue < 20:
		print 'geup from fall front'
		api.ubtStartAction("getup_in_front", 1)

#---------------------------Disconnect--------------------------------------
api.ubtDisconnectRobot("SDK","1",gIPAddr)
api.ubtDeinitialize()
