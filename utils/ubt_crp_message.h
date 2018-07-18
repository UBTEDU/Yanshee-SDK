/**
 * @file                RobotMsg.h
 * @brief           RobotMsg.c header file
 * @author          Cygnus Yang
 * @date            Thursday, August 31, 2017
 * @version         Initial Draft
 * @par             Copyright (C),  2017-2023, UBT Education
 * @par History:
 * 1.Date:          Thursday, August 31, 2017
 *   Author:            Cygnus Yang
 *   Modification:      Created file
*/
#ifndef __UBT_CRP_MESSAGE_H__
#define __UBT_CRP_MESSAGE_H__


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */

typedef struct
{
    char acMacAddr[ROBOT_MAC_LEN];
} ROBOTAGENT_MAC_T;

typedef struct robotagent_sc_coordinates_t
{
    int iX;
    int iY;
    int iZ;
    int iOrder;
} ROBOTAGENT_SC_COORDINATES_T;

extern UBT_RC_T ubtRobot_Msg_Decode_ConnectRobot(char *pcRecvBuf, char *pcRobotName, int iRobotNameLen);

extern UBT_RC_T ubtRobot_Msg_Decode_DetectVoiceMsg(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_DisconnectRobot(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_ReadRobotServo(char *pcRecvBuf, char *pcAllAngle, int iAngleLen);
extern UBT_RC_T
ubtRobot_Msg_Decode_ReadSensorValue(char *pcRecvBuf, char *pcSensorType, void *pValue, int iValueLen);


extern UBT_RC_T ubtRobot_Msg_Decode_RobotStatus(char *pcType, char *pcRecvBuf, void *pStatus);

extern UBT_RC_T ubtRobot_Msg_Decode_RobotStatus(char *pcType, char *pcRecvBuf, void *pStatus);

extern UBT_RC_T ubtRobot_Msg_Decode_SetRobotLED(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_SetRobotMotion(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_SetRobotServo(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_SetRobotVolume(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_StartRobotAction(char *pcRecvBuf, int *piTime);

extern UBT_RC_T ubtRobot_Msg_Decode_StopRobotAction(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_SWVersion(char *pcRecvBuf, char *pcVersion, int iVersionLen);

extern UBT_RC_T ubtRobot_Msg_Decode_VisionDetect(char *pcRecvBuf, char *pcValue);

extern UBT_RC_T ubtRobot_Msg_Decode_VoiceStart(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_VoiceStop(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Decode_VoiceTTS(char *pcRecvBuf);

extern UBT_RC_T ubtRobot_Msg_Encode_ConnectRobot(int iPort,
                                                    char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_DetectVoiceMsg(int iPort,
                                                      char *pcBuf, char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_StopVoiceRecognition(int iPort,
                                                            char *pcBuf, char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_DisconnectRobot(int iPort,
                                                       char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_ReadRobotServo(int iPort, char *pcSendBuf, int iBufLen);
extern UBT_RC_T ubtRobot_Msg_Encode_ReadRobotServoHold(int iPort, char *pcSendBuf, int iBufLen);
extern UBT_RC_T ubtRobot_Msg_Encode_ReadSensorValue(char *pcSensorType, int iPort,
                                                       char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_ReadSensorValueByAddr(char *pcSensorType, int iAddr, int iPort,
                                                             char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_RobotStatus(char *pcCmd, char *pcType, char *pcParam, int iPort,
                                                   char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotLED(int iPort, char *pcType,
                                                   char *pcColor, char *pcMode,
                                                   char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotMotion(char *pcCmd, char *pcType,
                                                      int iPort, int iVolume,
                                                      char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotServo(int iPort, char *pcAllAngle, int iTime,
        char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume(int iPort, int iVolume,
                                                      char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume_Minus(int iPort, char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume_Plus(int iPort, char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_StartRobotAction(int iPort,
                                                        char *pcName, int iRepeat,
                                                        char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_StopRobotAction(int iPort,
                                                       char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_SWVersion(char *pcParam, int iPort,
                                                 char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_EventDetect(char *pcEventType, int iPort,
                                                   char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Decode_EventDetect(char *pcRecvBuf, char *pcValue);

extern UBT_RC_T ubtRobot_Msg_Encode_VisionDetect(char *pcVisionType, int iPort,
                                                    char *pcSendBuf, int iBufLen, int iTimeout);

extern UBT_RC_T ubtRobot_Msg_Decode_VisionDetect(char *pcRecvBuf, char *pcValue);

extern UBT_RC_T ubtRobot_Msg_Encode_VoiceStart(int iPort,
                                                  char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_VoiceStop(int iPort,
                                                 char *pcSendBuf, int iBufLen);

extern UBT_RC_T ubtRobot_Msg_Encode_VoiceTTS(int iPort,
                                                int isInterrupted, char *pcTTS,
                                                char *pcSendBuf, int iBufLen);
extern UBT_RC_T ubtRobot_Msg_Encode_SearchSensor(int iPort,
        char *pcSendBuf, int iBufLen);
extern UBT_RC_T ubtRobot_Msg_Decode_SearchSensor(char *pcRecvBuf);
extern UBT_RC_T ubtRobot_Msg_Encode_ModifySensorID(int iPort,
        char *pcType, int iCurrID,int iDstID,
        char *pcSendBuf, int iBufLen);
extern UBT_RC_T ubtRobot_Msg_Decode_ModifySensorID(char *pcRecvBuf);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __UBT_CRP_MESSAGE_H__ */
