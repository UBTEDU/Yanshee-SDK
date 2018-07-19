/** @file   yanshee_api.h
 * @note    UBT Education Co., Ltd. All Right Reserved.
 * @brief   Defines the APIs for UBT SDK
 *
 * @author   Cygnus Yang
 * @date     2017-8-14
 * @version  1.0.0.0
 *
 * @note
 * @note History:
 * @note     Cygnus Yang   2017-12-01    1.0   first import
 * @note
 * @warning
 */


#ifndef __YANSHEE_API_H__
#define __YANSHEE_API_H__


/**
 * @brief      ubtGetSWVersion
 * @details    Get the robot versions including embedded system, raspberry,
             SDK and servos
 * @param[in]   eType
 *                                      Please see the defination UBT_ROBOT_SOFTVERSION_TYPE_E
 * @param[out]  pcVersion
 *                                      The output buffer for versions.
 *                                      In most cases, the version length is no more than 20 bytes.
 * @param[in]   iVersionLen
 *                                      The max output buffer for versions length.

 * @retval     UBT_RC_T
 */

UBT_RC_T ubtGetSWVersion(UBT_ROBOT_SOFTVERSION_TYPE_E eType, char *pcVersion, int iVersionLen);

/**
 * @brief      ubtGetPowerStatus
 * @details    Get the power status
 * @param[out]  piVoltage		Voltage(mv)
 * @param[out]  piRechargeStatus	Recharge(0 No,1 Yes,2 No battery)
 * @param[out]  piCapacity			Percent(Range 0~100)
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtGetPowerStatus(int *piVoltage, int *piRechargeStatus, int *piCapacity);

/**
 * @brief       ubtRecordMotion
 * @details     Read all servo's angle
 * @param[in]   pstServosAngle All servos angle
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtRecordMotion(UBT_ROBOTSERVO_T *pstServosAngle);

/**
 * @brief:      ubtGetServosAngle
 * @details     Read one/multiple/all servo's angle
 * @param[in]   pstServoAngle	The angle for the servos, details please see UBT_ROBOTSERVO_T
 *                  bit 0 indicates the first servo's angle.
 *                  FF means the invalid value.
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtGetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle);


/**
 * @brief       ubtSetServosAngle
 * @details     Set the servo's acAngle with speed
 * @param[in]   pstServoAngle   The angle for the servos, details please see UBT_ROBOTSERVO_T
 * @param[in]   iTime       It is the time for servo, the value is smaller, the speed is faster.
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtSetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle, int iTime);

/**
 * @brief      ubtSetVolume
 * @details    Set the volume for the Robot
 * @param[in]   iVolume  [0-100] Volume percent
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtSetVolume(int iVolume);

/**
 * @brief      ubtGetVolume
 * @details    Get the volume for the Robot
 * @param[out]   piVolume  [0-100] Volume percent
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtGetVolume(int *piVolume);


/**
 * @brief      ubtReadSensorValue
 * @details    Read the sensor's value
 * @param[in]   pcSensorType  The sensor's type.
 *                                  gyro
 *                                  environment
 *                                  board
 *                                  infrared
 *                                  ultrasonic
 *                                  touch
 *                                  color
 *                                  pressure
 *                                  gas
 * @param[out]  pValue        The sensor value. More details please see the defination as below variable type
 *                                  UBT_ROBOTGYRO_SENSOR_T
 *                                  UBT_ROBOTENV_SENSOR_T
 *                                  UBT_ROBOTRASPBOARD_SENSOR_T
 *                                  UBT_ROBOTINFRARED_SENSOR_T
 *                                  UBT_ROBOTULTRASONIC_SENSOR_T
 *                                  UBT_ROBOTTOUCH_SENSOR_T
 *                                  UBT_ROBOTCOLOR_SENSOR_T
 *                                  UBT_ROBOTPRESSURE_SENSOR_T
 *                                  UBT_ROBOTGAS_SENSOR_T
 * @param[in]   iValueLen       The max length of pValue
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtReadSensorValue(char *pcSensorType, void *pValue, int iValueLen);

/**
 * @brief      ubtReadSensorValueByAddr
 * @details    Read the sensor's value by it's type and address
 * @param[in]   pcSensorType  The sensor's type.
 *                                  gryo
 *                                  environment
 *                                  board
 *                                  infrared
 *                                  ultrasonic
 *                                  touch
 *                                  color
 *                                  pressure
 *                                  gas
 * @param[in]   iAddr             The sensor's 7bit I2C address
 * @param[out]  pValue      The sensor value. More details please see the defination as below variable type
 *                                  UBT_ROBOTGYRO_SENSOR_T
 *                                  UBT_ROBOTENV_SENSOR_T
 *                                  UBT_ROBOTRASPBOARD_SENSOR_T
 *                                  UBT_ROBOTINFRARED_SENSOR_T
 *                                  UBT_ROBOTULTRASONIC_SENSOR_T
 *                                  UBT_ROBOTTOUCH_SENSOR_T
 *                                  UBT_ROBOTCOLOR_SENSOR_T
 *                                  UBT_ROBOTPRESSURE_SENSOR_T
 *                                  UBT_ROBOTGAS_SENSOR_T
 * @param[in]   iValueLen       The max length of pValue
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtReadSensorValueByAddr(char *pcSensorType, int iAddr, void *pValue, int iValueLen);

/**
 * @brief      ubtSetRobotLED
 * @details    Set the LED mode
 * @param[in]   pcType
 *                              botton
 *                              camera
 *                              mic
 * @param[in]   pcColor
 *                                  When pcType == "botton"
 *                                  pcColor can be set as
 *                                  white
 *                                  red
 *                                  green
 *                                  blue
 *                                  yellow
 *                                  purple
 *                                  cyan
 *                                  When pcType == "camera"
 *                                  pcColor can be set as
 *                                  red
 *                                  green
 *                                  blue
 *                                  When pcType == "mic"
 *                                  pcColor can be set as
 *                                  green
 * @param[in]   pcMode
 *                                  When pcType == "button"
 *                                  pcMOde can be set as
 *                                  off
 *                                  on
 *                                  blink
 *                                  breath
 *                                  colorful
 *                                  When pcType == "camera"
 *                                  pcMOde can be set as
 *                                  on
 *                                  off
 *                                  When pcType == "mic"
 *                                  pcMOde can be set as
 *                                  on
 *                                  off
 *
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtSetRobotLED(char *pcType, char *pcColor, char *pcMode);

/**
 * @brief      ubtAynsStartAction
 * @details    Execute a default action asynchronous
 * @param[in]   pcName  The action file's name For
 *                              example: push up, bow
 * @param[in]   iRepeat   Repeat times. 0 means infinite
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtAsynStartAction(char *pcName, int iRepeat);

/**
 * @brief      ubtStartAction
 * @details    Let the robot play an action
 * @param[in]   pcName  The action file's name For
 *                             push up
 *                             crouch
 *                             raise
 *                             stretch
 *                             come on
 *                             wave
 *                             bend
 *                             walk
 *                             turn around
 *                             bow ...
 * @param[in]   iRepeat   Repeat times. 0 means infinite
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtStartAction(char *pcName, int iRepeat);

/**
  * @brief      Stop to run the robot action file
  *
  * @return  UBT_RC_T 0 Success,  Others    Failed
  *
  */
UBT_RC_T ubtStopAction();

/**
 * @brief      ubtSetMotion
 * @details    Set the robot's action
 * @param[in]   pcType
 *                             crouch
 *                             raise
 *                             stretch
 *                             come on
 *                             wave
 *                             bend
 *                             walk
 *                             turn around
 *                             bow
 * @param[in]   pcDirect
 *                             left
 *                             right
 *                             both
 *                             front
 *                             back
 * @param[in]   iSpeed      1/2/3/4/5  The default value is 3
 * @param[in]   iRepeat     Repeat times. 0 means infinite
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtSetMotion(char *pcType, char *pcDirect, int iSpeed, int iRepeat);


/**
 * @brief   Start voice recognition
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 *
 */
UBT_RC_T ubtStartAsr();

/**
 * @brief   Stop voice recognition
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 *
 */
UBT_RC_T ubtStopAsr();

/**
 * @brief   Play the TTS voice
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 * @param   isInterrputed   Interrupt the previous TTS, if it is not finished.
 *                              0   Not interrupt the previous TTS
 *                              1   Interrupt the previous TTS, start the current TTS immediately
 * @param   pcTTS The message to be sent to TTS
 *
 */
UBT_RC_T ubtStartTts(int isInterrputed, char *pcTTS);

/**
 * @brief   Stop tts
 *
 * @retval	UBT_RC_T
 *
 */
UBT_RC_T ubtStopTts();


/**
 * @brief	ubtEventDetect
 * @details	Detect robot's event include the push power button, voice detecting etc.
 * @param[in]	pcEventType	
 * @param[in]	iTimeout
 * @param[out]	pcValue
 * @retval	UBT_RC_T
 */
UBT_RC_T ubtEventDetect(char *pcEventType, char *pcValue, int iTimeout);

/**
 * @brief      ubtConnectRobot
 * @details    Connect to Robot
 * @param[in]   pcIPAddr   Robot IP address
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtConnectRobot(char *pcIPAddr);

/**
 * @brief      ubtDisconnectRobot
 * @details    Disconnect from the robot
 * @param[in]   pcIPAddr   Robot IP address
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtDisconnectRobot(char *pcIPAddr);

/**
 * @brief       ubtSearchExtendSensor
 * @details     Search all extend sensor include infrared ultrsonic touch environment press
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtSearchExtendSensor(void);

/**
 * @brief       ubtModifyExtendSensorID
 * @details     Modify   Yanshee's extend sensor ID
 * @param[in]   pcType   Sensor type
 * @param[in]   iCurrID  Sensor ID
 * @param[in]   iDstID   modify id value 
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtModifyExtendSensorID(char *pcType, int iCurrID, int iDstID);


/**
 * @brief      ubtInitialize
 * @details    Init the SDK for 1x
 * @retval		UBT_RC_T
 */
UBT_RC_T ubtInitialize();

/**
 * @brief      ubtDeinitialize
 * @details    Destroy the SDK for 1x
 * @retval		UBT_RC_T
 */
void ubtDeinitialize();


#endif

