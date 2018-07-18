
%module yanshee_api


#define UBT_SDK_SW_VER              "01"        /**< SDK software version */
#define UBT_ROBOT_IP_ADDR_LEN       (16)        /**< MAX length of the IP address */
#define MAX_SHELL_CMD_LEN           (256)       /**< MAX length of the Shell cmd */
#define UBT_ROBOT_VERSION_LEN		(32)        /**< Robot name length */
#define UBT_ROBOT_NAME_LEN           (32)       /**< Define the MAX length of the robot name */
#define UBT_ROBOT_IP_ADDR_LEN        (16)       /**< Define the MAX length of the IP address */

typedef enum
{
    UBT_ROBOT_STATUS_TYPE_PLAYACTION = 1,    /**< Play an action file */
    UBT_ROBOT_STATUS_TYPE_VOLUME,            /**< Volume status */
    UBT_ROBOT_STATUS_TYPE_POWER_VOLTAGE,     /**< Power voltage status */
    UBT_ROBOT_STATUS_TYPE_POWER_RECHARGE,    /**< Power recharge status */
    UBT_ROBOT_STATUS_TYPE_POWER_PERCENT,     /**< Power percent status */
    UBT_ROBOT_STATUS_TYPE_INVALID            /**< Invalid type */
} UBT_ROBOT_STATUS_TYPE_E;

typedef enum
{
    UBT_ROBOT_PLAY_STATUS_IDLE,          /**< Idle status */
    UBT_ROBOT_PLAY_STATUS_PLAYING,       /**< Playing */
    UBT_ROBOT_PLAY_STATUS_PAUSED,        /**< Paused */
    UBT_ROBOT_PLAYSTATUS_END,            /**< End */
    UBT_ROBOT_PLAY_STATUS_INVALID        /**< Invalid status */
} UBT_ROBOT_PLAYMUSIC_STATUS_E;

typedef enum
{
    UBT_RC_SUCCESS = 0,    /**< Success */
    UBT_RC_FAILED,         /**< Failed */
    UBT_RC_NORESOURCE,     /**< No resource */
    UBT_RC_NOT_FOUND,      /**< Not found */
    UBT_RC_WRONG_PARAM,    /**< Wrong parameter */


    UBT_RC_SOCKET_FAILED = 100,    /**< Socket error */
    UBT_RC_SOCKET_NORESOURCE,      /**< No resource when sending message out */
    UBT_RC_SOCKET_TIMEOUT,         /**< Recevied message timeout */
    UBT_RC_SOCKET_ENCODE_FAILED,   /**< Encode the message failed */
    UBT_RC_SOCKET_DECODE_FAILED,   /**< Decode the message failed */
    UBT_RC_SOCKET_ENCODE_ERROR,    /**< Encode the message error */
    UBT_RC_SOCKET_DECODE_ERROR,    /**< Decode the message error. It is possible that received a wrong message */
    UBT_RC_SOCKET_SENDERROR,       /**< Error when sending message out */

    UBT_RC_VOICE_FAILED,           /**< Voice recognition failed */
    UBT_RC_VOICE_GRAMMAR_ERROR,    /**< Voiice recognition grammer error */
    UBT_RC_VOICE_AIUIDECODE_ERROR, /**< Decode AIUI message failed */
    UBT_RC_LAST                    /**< The last return value */
} UBT_RC_T;

typedef enum
{
    UBT_ROBOT_SOFTVERSION_TYPE_STM32 = 0,    /**< Embedded system version */
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS1,      /**< Servos' version */
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS2,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS3,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS4,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS5,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS6,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS7,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS8,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS9,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS10,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS11,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS12,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS13,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS14,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS15,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS16,
    UBT_ROBOT_SOFTVERSION_TYPE_SERVOS17,
    UBT_ROBOT_SOFTVERSION_TYPE_SDK = 30,         /**< SDK version */
    UBT_ROBOT_SOFTVERSION_TYPE_RASPI = 31,       /**< Robot management application version */
    UBT_ROBOT_SOFTVERSION_TYPE_INVALID       /**< Invalid type */
} UBT_ROBOT_SOFTVERSION_TYPE_E;

typedef struct _RobotServo
{
    int SERVO1_ANGLE;	/**< The 1st servo's angle */
    int SERVO2_ANGLE;	/**< The 2nd servo's angle */
    int SERVO3_ANGLE;	/**< The 3rd servo's angle */
    int SERVO4_ANGLE;	/**< The 4th servo's angle */
    int SERVO5_ANGLE;	/**< The 5th servo's angle */
    int SERVO6_ANGLE;	/**< The 6th servo's angle */
    int SERVO7_ANGLE;	/**< The 7th servo's angle */
    int SERVO8_ANGLE;	/**< The 8th servo's angle */
    int SERVO9_ANGLE;	/**< The 9th servo's angle */
    int SERVO10_ANGLE;	/**< The 10th servo's angle */
    int SERVO11_ANGLE;	/**< The 11th servo's angle */
    int SERVO12_ANGLE;	/**< The 12th servo's angle */
    int SERVO13_ANGLE;	/**< The 13th servo's angle */
    int SERVO14_ANGLE;	/**< The 14th servo's angle */
    int SERVO15_ANGLE;	/**< The 15th servo's angle */
    int SERVO16_ANGLE;	/**< The 16th servo's angle */
    int SERVO17_ANGLE;	/**< The 17th servo's angle */
} UBT_ROBOTSERVO_T;

typedef struct _RobotInfo
{
    char acIpAddr[UBT_ROBOT_IP_ADDR_LEN];
} UBT_ROBOTINFO_T;

typedef struct _RobotGyroSensor
{
    double  dGyroxValue; /**< Gyro x value */
    double  dGyroyValue; /**< Gyro y value */
    double  dGyrozValue; /**< Gyro z value */
    double  dAccexValue; /**< accelerate x value */
    double  dAcceyValue; /**< accelerate y value */
    double  dAccezValue; /**< accelerate z value */
    double  dCompassxValue; /**< compass x value */
    double  dCompassyValue; /**< compass y value */
    double  dCompasszValue; /**< compass z value */
    double  dEulerxValue; /**< euler x value */
    double  dEuleryValue; /**< euler y value */
    double  dEulerzValue; /**< euler z value */
} UBT_ROBOTGYRO_SENSOR_T;

typedef struct _RobotEnvSensor
{

    int iTempValue;      /**<  temperature value */
    int iHumiValue;      /**<  humidity value */
    int iPresValue;      /**<  pressure value */
	
} UBT_ROBOTENV_SENSOR_T;

typedef struct _RobotRaspPiBoardSensor
{
    int iValue;         /**<    Board temperature */
} UBT_ROBOTRASPBOARD_SENSOR_T;

typedef struct _RobotUltrasonicSensor
{
    int iValue;         /**<    The distance via ultrasonic sensor */
} UBT_ROBOTULTRASONIC_SENSOR_T;

typedef struct _RobotInfraredSensor
{
    int iValue;         /**<    The distance via infrared sensor */
} UBT_ROBOTINFRARED_SENSOR_T;

typedef struct _RobotTouchSensor
{
    int iValue;         /**<    The Touch  sensor */
} UBT_ROBOTTOUCH_SENSOR_T;

typedef struct _RobotColorSensor
{
    int iRedValue;         /**<    The red value of color sensor */
    int iGreenValue;         /**<    The Green value of color sensor */
    int iBlueValue;         /**<    The Bluevalue of color sensor */
    int iClearValue;         /**<    The Clear value of color sensor */
} UBT_ROBOTCOLOR_SENSOR_T;

typedef struct _RobotPressureSensor
{
    int iValue;         /**<    The Pressure via Pressure sensor */
} UBT_ROBOTPRESSURE_SENSOR_T;

/**
 * @brief  Charging status. (0 Not in charge,1 In charging,2 Battery is not installed),
*/
typedef enum
{
    UBT_BATTERY_STATUS_NOTCHARGING,     /**< Not in charge */
    UBT_BATTERY_STATUS_CHARING,         /**< Charging */
    UBT_BATTERY_STATUS_MISS             /**< Battery is not installed */
} UBT_BATTERY_STATUS_E;

/**
 * @brief   Battery data
*/
typedef struct _RobotBatteryInfo
{
    UBT_BATTERY_STATUS_E iChargeStatus;        /**< Charge status */
    int                  iVoltage;             /**< Voltage */
    int                  iCapacity;            /**< Battery capacity */
} UBT_ROBOT_BATTERY_T;

/**
 * @brief   Version
*/
typedef struct _RobotVersionInfo
{
    char	acVersion[UBT_ROBOT_VERSION_LEN];
} UBT_ROBOT_VERSION_T;

%{
#include "ubt_datatypes.h"
#include "yanshee_api.h"

UBT_RC_T ubtGetSWVersion(UBT_ROBOT_SOFTVERSION_TYPE_E eType, char *pcVersion, int iVersionLen);
UBT_RC_T ubtGetPowerStatus(int *piVoltage, int *piRechargeStatus, int *piCapacity);
UBT_RC_T ubtRecordMotion(UBT_ROBOTSERVO_T *pstServosAngle);
UBT_RC_T ubtGetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle);
UBT_RC_T ubtSetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle, int iTime);
UBT_RC_T ubtSetVolume(int iVolume);
UBT_RC_T ubtGetVolume(int *piVolume);
UBT_RC_T ubtReadSensorValue(char *pcSensorType, void *pValue, int iValueLen);
UBT_RC_T ubtReadSensorValueByAddr(char *pcSensorType, int iAddr, void *pValue, int iValueLen);
UBT_RC_T ubtSetRobotLED(char *pcType, char *pcColor, char *pcMode);
UBT_RC_T ubtAsynStartAction(char *pcName, int iRepeat);
UBT_RC_T ubtStartAction(char *pcName, int iRepeat);
UBT_RC_T ubtStopAction();
UBT_RC_T ubtSetMotion(char *pcType, char *pcDirect, int iSpeed, int iRepeat);
UBT_RC_T ubtStartAsr();
UBT_RC_T ubtStopAsr();
UBT_RC_T ubtStartTts(int isInterrputed, char *pcTTS);
UBT_RC_T ubtStopTts();
UBT_RC_T ubtEventDetect(char *pcEventType, char *pcValue, int iTimeout);
UBT_RC_T ubtConnectRobot(char *pcIPAddr);
UBT_RC_T ubtDisconnectRobot(char *pcIPAddr);
UBT_RC_T ubtInitialize();
void ubtDeinitialize();

%}
#include "yanshee_api.h"

UBT_RC_T ubtGetSWVersion(UBT_ROBOT_SOFTVERSION_TYPE_E eType, char *pcVersion, int iVersionLen);
UBT_RC_T ubtGetPowerStatus(int *piVoltage, int *piRechargeStatus, int *piCapacity);
UBT_RC_T ubtRecordMotion(UBT_ROBOTSERVO_T *pstServosAngle);
UBT_RC_T ubtGetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle);
UBT_RC_T ubtSetServosAngle(UBT_ROBOTSERVO_T *pstServoAngle, int iTime);
UBT_RC_T ubtSetVolume(int iVolume);
UBT_RC_T ubtGetVolume(int *piVolume);
UBT_RC_T ubtReadSensorValue(char *pcSensorType, void *pValue, int iValueLen);
UBT_RC_T ubtReadSensorValueByAddr(char *pcSensorType, int iAddr, void *pValue, int iValueLen);
UBT_RC_T ubtSetRobotLED(char *pcType, char *pcColor, char *pcMode);
UBT_RC_T ubtAsynStartAction(char *pcName, int iRepeat);
UBT_RC_T ubtStartAction(char *pcName, int iRepeat);
UBT_RC_T ubtStopAction();
UBT_RC_T ubtSetMotion(char *pcType, char *pcDirect, int iSpeed, int iRepeat);
UBT_RC_T ubtStartAsr();
UBT_RC_T ubtStopAsr();
UBT_RC_T ubtStartTts(int isInterrputed, char *pcTTS);
UBT_RC_T ubtStopTts();
UBT_RC_T ubtEventDetect(char *pcEventType, char *pcValue, int iTimeout);
UBT_RC_T ubtConnectRobot(char *pcIPAddr);
UBT_RC_T ubtDisconnectRobot(char *pcIPAddr);
UBT_RC_T ubtInitialize();
void ubtDeinitialize();