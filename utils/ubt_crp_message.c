/**
 * @file                RobotMsg.c
 * @brief           Defines the messages to robot
 * @author          Cygnus Yang
 * @date            Wednesday, August 30, 2017
 * @version         Initial Draft
 * @par             Copyright (C),  2017-2023, UBT Education
 * @par History:
 * 1.Date:          Wednesday, August 30, 2017
 *   Author:            Cygnus Yang
 *   Modification:      Created file
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <linux/sockios.h>

#include "cJSON.h"
#include "ubt_datatypes.h"
#include "yanshee_api.h"
#include "robotlogstr.h"
#include "ubt_crp_message.h"

#ifdef __DEBUG_PRINT__                                            // 对于DEBUG版本，增加打印信息
#define DebugTrace(...)\
        do{\
            fprintf(stderr,__VA_ARGS__);\
        }while(0)
#else
#define DebugTrace( ... )                // 对于RELEASE版本，把__DEBUG_PRINT__宏关闭 
#endif


UBT_RC_T ubtRobot_Msg_Encode_SWVersion(char *pcParam, int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    /* Check parameters */
    if ((NULL == pcSendBuf) || (NULL == pcParam))
    {
        return UBT_RC_WRONG_PARAM;
    }

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Query);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Version);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Para, pcParam);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port, iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SWVersion(char *pcRecvBuf, char *pcVersion, int iVersionLen)
{
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acRet[MSG_CMD_STR_MAX_LEN];
    char acType[MSG_CMD_STR_MAX_LEN];
    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if ((NULL == pcVersion) || (NULL == pcRecvBuf))
    {
        return UBT_RC_WRONG_PARAM;
    }
    acRet[0] = '\0';
    acType[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (0 != strcmp(pNode->valuestring, pcStr_Msg_Cmd_Query_Ack))
                {
                    /* Wrong cmd value */
                    ret = UBT_RC_SOCKET_DECODE_ERROR;
                    break;
                }
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strcpy(acType, pNode->valuestring) ;
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strcpy(acRet, pNode->valuestring) ;
            }
        }

        if (!strcmp(acRet, "ok") && !strcmp(acType, pcStr_Msg_Type_Version))
        {
            pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type_Version);
            if (pNode != NULL)
            {
                if (pNode->type == cJSON_String)
                {
                    strncpy(pcVersion, pNode->valuestring, iVersionLen);
                    ret = UBT_RC_SUCCESS;
                    break;
                }
            }
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ret;
}

UBT_RC_T ubtRobot_Msg_Encode_RobotStatus(char *pcCmd, char *pcType, char *pcParam, int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    /* Check parameters */
    if ((NULL == pcSendBuf) || (NULL == pcCmd) || (NULL == pcType))
    {
        return UBT_RC_WRONG_PARAM;
    }

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcCmd);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port, iPort);
    if (NULL != pcParam)
    {
        cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Para, pcParam);
    }
    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_RobotStatus(char *pcType, char *pcRecvBuf, void *pStatus)
{
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    cJSON *pSubNode = NULL;
    char acType[MSG_CMD_STR_MAX_LEN];
    char acParam[MSG_CMD_STR_MAX_LEN];
    UBT_ROBOT_BATTERY_T *pstRobotBattery;
    int *piValue;

    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if ((NULL == pcRecvBuf) || (NULL == pcType))
    {
        return UBT_RC_WRONG_PARAM;
    }

    acType[0] = '\0';
    acParam[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pJson   = cJSON_Parse(pcRecvBuf);
        if (pJson == NULL)
        {
            printf("Parse json message filed!\r\n");
            return UBT_RC_SOCKET_DECODE_FAILED;
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                /* Check the cmd value */
                if (0 != strcmp(pNode->valuestring, pcStr_Msg_Cmd_Query_Ack))
                {
                    /* Wrong cmd value */
                    ret = UBT_RC_SOCKET_DECODE_ERROR;
                    break;
                }
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (0 != strcmp(pNode->valuestring, pcType))
                {
                    /* Wrong cmd value */
                    ret = UBT_RC_SOCKET_DECODE_ERROR;
                    break;
                }
                strncpy(acType, pNode->valuestring, sizeof(acType));
            }
        }


        if (!strcmp(acType, pcStr_Msg_Type_Play))
        {
            pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type_Play);
            if (pNode != NULL)
            {
                if (pNode->type == cJSON_String)
                {
                    strncpy(acParam, pNode->valuestring, sizeof(acParam));
                }
                piValue = (int *)pStatus;
                if (!strcmp(acParam, pcStr_Msg_Param_Query_Play_Playing))
                {
                    *piValue = UBT_ROBOT_PLAY_STATUS_PLAYING;
                }
                else if (!strcmp(acParam, pcStr_Msg_Param_Query_Play_Pause))
                {
                    *piValue = UBT_ROBOT_PLAY_STATUS_PAUSED;
                }
                else if (!strcmp(acParam, pcStr_Msg_Param_Query_Play_End))
                {
                    *piValue = UBT_ROBOT_PLAYSTATUS_END;
                }
                else if (!strcmp(acParam, pcStr_Msg_Param_Query_Play_idle))
                {
                    *piValue = UBT_ROBOT_PLAY_STATUS_IDLE;
                }
                else
                {
                    ret = UBT_RC_SOCKET_DECODE_ERROR;
                }
                ret     = UBT_RC_SUCCESS;
            }
        }


        if (!strcmp(acType, pcStr_Msg_Type_Volume))
        {
            pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type_Volume);
            if (pNode != NULL)
            {
                piValue = (int *)pStatus;
                if (pNode->type == cJSON_Number)
                {
                    *piValue = pNode->valueint;
                }
                ret     = UBT_RC_SUCCESS;
            }
        }

        if (!strcmp(acType, pcStr_Msg_Type_Battery))
        {
            if ((pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Type_Battery)) != NULL)
            {
                pstRobotBattery = (UBT_ROBOT_BATTERY_T*)pStatus;
                if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Power_Charging)) != NULL)
                {
                    if (pSubNode->type == cJSON_Number)
                    {
                        pstRobotBattery->iChargeStatus = pSubNode->valueint;
                    }
                }

                if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Power_Voltage)) != NULL)
                {
                    if (pSubNode->type == cJSON_Number)
                    {
                        pstRobotBattery->iVoltage = pSubNode->valueint;
                    }
                }

                if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Power_Percent)) != NULL)
                {
                    if (pSubNode->type == cJSON_Number)
                    {
                        pstRobotBattery->iCapacity = pSubNode->valueint;
                        ret     =  UBT_RC_SUCCESS;
                    }
                }
            }
        }

    }
    while (0);
    cJSON_Delete(pJson);
    return ret;
}

UBT_RC_T ubtRobot_Msg_Encode_StopVoiceRecognition(int iPort,
        char *pcBuf, char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Voice);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Recognition_Stop);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Data, pcBuf);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);

    cJSON_Delete(pJsonRoot);
    return UBT_RC_SUCCESS;
}


UBT_RC_T ubtRobot_Msg_Encode_DetectVoiceMsg(int iPort,
        char *pcBuf, char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;



    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Voice);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Voice_Detecting);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Data, pcBuf);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);

    cJSON_Delete(pJsonRoot);
    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_DetectVoiceMsg(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];



    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Voice_Ack))
                {
                    ubtRet     =  UBT_RC_SUCCESS;
                }
            }
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_ReadRobotServo(int iPort, char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Servo);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Servo_Read);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Encode_ReadRobotServoHold(int iPort, char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Servo);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Servo_Read_Hold);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_ReadRobotServo(char *pcRecvBuf, char *pcAllAngle, int iAngleLen)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL, *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Type_Servo_Angle);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(pcAllAngle, pNode->valuestring, iAngleLen);
                ubtRet = UBT_RC_SUCCESS;
            }
        }

        if (strcmp(acCmd, pcStr_Msg_Cmd_Servo_Ack))
        {
            ubtRet = UBT_RC_SOCKET_DECODE_ERROR;
            break;
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}



UBT_RC_T ubtRobot_Msg_Encode_SetRobotServo(int iPort, char *pcAllAngle, int iTime,
        char *pcSendBuf, int iBufLen)
{
    cJSON *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    /* 20 means 1s */
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Servo);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Servo_Write);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Type_Servo_Time, iTime);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type_Servo_Angle, pcAllAngle);
    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SetRobotServo(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Servo_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }

    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}

UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume_Plus(int iPort, char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Set);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Volume_Plus);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume_Minus(int iPort, char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Set);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Volume_Minus);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}


UBT_RC_T ubtRobot_Msg_Encode_SetRobotVolume(int iPort, int iVolume,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Set);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Volume);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Type_Volume, iVolume);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SetRobotVolume(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Set_Ack))
                {
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}



UBT_RC_T ubtRobot_Msg_Encode_SetRobotMotion(char *pcCmd, char *pcType,
        int iPort, int iVolume,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcCmd);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Cmd_Voice, iVolume);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SetRobotMotion(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Set_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}



UBT_RC_T ubtRobot_Msg_Encode_ReadSensorValue(char *pcSensorType, int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Query);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Sensor);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Para, pcSensorType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port, iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Encode_ReadSensorValueByAddr(char *pcSensorType, int iAddr, int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Query);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Sensor);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Para, pcSensorType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Param_Query_Sensor_ID, iAddr);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port, iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_ReadSensorValue(char *pcRecvBuf, char *pcSensorType, void *pValue, int iValueLen)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON       *pJson = NULL;
    cJSON       *pNode = NULL;
    cJSON       *pSubArray = NULL, *pSubNode = NULL;
    UBT_ROBOTGYRO_SENSOR_T       *pstRobotGyro = NULL;
    UBT_ROBOTENV_SENSOR_T        *pstRobotEnv = NULL;
    UBT_ROBOTRASPBOARD_SENSOR_T  *pstRobotBrdTemp = NULL;
    UBT_ROBOTINFRARED_SENSOR_T   *pstRobotInfrared = NULL;
    UBT_ROBOTULTRASONIC_SENSOR_T *pstRobotUltrasnic = NULL;
    UBT_ROBOTTOUCH_SENSOR_T   *pstRobotTouch = NULL;
    UBT_ROBOTCOLOR_SENSOR_T   *pstRobotColor = NULL;
    UBT_ROBOTPRESSURE_SENSOR_T   *pstRobotPressure = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];
    char acType[MSG_CMD_STR_MAX_LEN];
    int iCount, i;

    UBT_RC_T ret = UBT_RC_FAILED;

    pstRobotInfrared = pstRobotInfrared;
    /* Check parameters */
    if ((NULL == pcRecvBuf) || (NULL == pValue) || (NULL == pcSensorType))
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';
    acType[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strcpy(acType, pNode->valuestring);
            }
        }

        if ((pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_GYRO)) != NULL)
        {
            pstRobotGyro = (UBT_ROBOTGYRO_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTGYRO_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_GYRO_X)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dGyroxValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_GYRO_Y)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dGyroyValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_GYRO_Z)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dGyrozValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ACCEL_X)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dAccexValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ACCEL_Y)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dAcceyValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ACCEL_Z)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dAccezValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_COMPASS_X)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dCompassxValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_COMPASS_Y)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dCompassyValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_COMPASS_Z)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dCompasszValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_EULER_X)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dEulerxValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_EULER_Y)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dEuleryValue = pSubNode->valuedouble;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_EULER_Z)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotGyro->dEulerzValue = pSubNode->valuedouble;
                }
            }
            ubtRet = UBT_RC_SUCCESS;

        }
        else if ((pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_ENV)) != NULL)
        {
            pstRobotEnv = (UBT_ROBOTENV_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTENV_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ENV_Temperature)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotEnv->iTempValue = pSubNode->valueint;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ENV_Humidity)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotEnv->iHumiValue = pSubNode->valueint;
                }
            }
            if ((pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_ENV_Pressure)) != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotEnv->iPresValue = pSubNode->valueint;
                }
            }

            ubtRet = UBT_RC_SUCCESS;
        }
        else if ((pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Board)) != NULL)
        {
            pstRobotBrdTemp = (UBT_ROBOTRASPBOARD_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTRASPBOARD_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            pSubNode = cJSON_GetObjectItem(pNode, pcStr_Msg_Param_Query_Sensor_Board_Temperature);
            if (pSubNode != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotBrdTemp->iValue = pSubNode->valueint;
                }
            }

            ubtRet = UBT_RC_SUCCESS;
        }
        else if ((pNode= cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Ultrasonic)) != NULL)
        {
            pstRobotUltrasnic = (UBT_ROBOTULTRASONIC_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTULTRASONIC_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            iCount = cJSON_GetArraySize(pNode);
            DebugTrace("Ultrasonic Array Number:%d\r\n", iCount);
            pstRobotUltrasnic->iValue = 0xffff;
            for(i=0; i<iCount; i++)
            {
                pSubArray = cJSON_GetArrayItem(pNode, i);
                pSubNode = cJSON_GetObjectItem(pSubArray, pcStr_Msg_Param_Query_Sensor_Value);
                if (pSubNode != NULL)
                {
                    if (pSubNode->type == cJSON_Number)
                    {
                        if(pSubNode->valueint  && (pstRobotUltrasnic->iValue > pSubNode->valueint))
                            pstRobotUltrasnic->iValue = pSubNode->valueint;
                        ubtRet = UBT_RC_SUCCESS;
                    }
                }
            }
        }
        else if ((pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Infrared)) != NULL)
        {
            pstRobotUltrasnic = (UBT_ROBOTULTRASONIC_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTULTRASONIC_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            iCount = cJSON_GetArraySize(pNode);
            DebugTrace("Infrared Array Number:%d\r\n", iCount);
            pstRobotUltrasnic->iValue = 0xffff;
            for(i=0; i<iCount; i++)
            {
                pSubArray = cJSON_GetArrayItem(pNode, i);
                pSubNode = cJSON_GetObjectItem(pSubArray, pcStr_Msg_Param_Query_Sensor_Value);
                if (pSubNode != NULL)
                {
                    if (pSubNode->type == cJSON_Number)
                    {
                        if(pSubNode->valueint  && (pstRobotUltrasnic->iValue > pSubNode->valueint))
                            pstRobotUltrasnic->iValue = pSubNode->valueint;
                        ubtRet = UBT_RC_SUCCESS;
                    }
                }
            }
        }
        else if ((pNode= cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Touch)) != NULL)
        {
            pstRobotTouch= (UBT_ROBOTTOUCH_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTTOUCH_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            iCount = cJSON_GetArraySize(pNode);
            if(iCount < 1)
            {
                DebugTrace("Touch Array is NULL");
                return UBT_RC_NOT_FOUND;
            }
            pSubArray = cJSON_GetArrayItem(pNode, 0);
            pSubNode = cJSON_GetObjectItem(pSubArray, pcStr_Msg_Param_Query_Sensor_Value);
            if (pSubNode != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotTouch->iValue = pSubNode->valueint;
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }
        else if ((pNode= cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Color)) != NULL)
        {
            pstRobotColor= (UBT_ROBOTCOLOR_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTCOLOR_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            iCount = cJSON_GetArraySize(pNode);
            if(iCount < 1)
            {
                DebugTrace("Color Array is NULL");
                return UBT_RC_NOT_FOUND;
            }
            pSubArray = cJSON_GetArrayItem(pNode, 0);
            pSubNode = cJSON_GetObjectItem(pSubArray, pcStr_Msg_Param_Query_Sensor_Value);
            if (pSubNode != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotColor->iRedValue = pSubNode->valueint&0xff;
                    pstRobotColor->iGreenValue = (pSubNode->valueint>>8) &0xff;
                    pstRobotColor->iBlueValue = (pSubNode->valueint>>16) &0xff;
                    pstRobotColor->iClearValue = (pSubNode->valueint>>24) &0xff;
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }
        else if ((pNode= cJSON_GetObjectItem(pJson, pcStr_Msg_Param_Query_Sensor_Pressure)) != NULL)
        {
            pstRobotPressure= (UBT_ROBOTPRESSURE_SENSOR_T *)pValue;
            if (iValueLen != sizeof(UBT_ROBOTPRESSURE_SENSOR_T))
            {
                return UBT_RC_WRONG_PARAM;
            }
            iCount = cJSON_GetArraySize(pNode);
            if(iCount < 1)
            {
                DebugTrace("Pressure Array is NULL");
                return UBT_RC_NOT_FOUND;
            }
            pSubArray = cJSON_GetArrayItem(pNode, 0);
            pSubNode = cJSON_GetObjectItem(pSubArray, pcStr_Msg_Param_Query_Sensor_Value);
            if (pSubNode != NULL)
            {
                if (pSubNode->type == cJSON_Number)
                {
                    pstRobotPressure->iValue = pSubNode->valueint;
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}

UBT_RC_T ubtRobot_Msg_Encode_SetRobotLED(int iPort, char *pcType,
        char *pcColor, char *pcMode,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;
    cJSON   *pArraySubJson = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Set);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_LED);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    if ((pArraySubJson = cJSON_CreateObject()) == NULL)
    {
        cJSON_Delete(pJsonRoot);
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    /* Only the voice recognition LED can be set by user */
    /* The other LED such as servo, camera and mic LED cannot be controled
    by the user */
    cJSON_AddStringToObject(pArraySubJson, pcStr_Msg_Type, pcType);
    cJSON_AddStringToObject(pArraySubJson, pcStr_Msg_Param_Led_Color, pcColor);
    cJSON_AddStringToObject(pArraySubJson, pcStr_Msg_Param_Led_Mode, pcMode);
    cJSON_AddItemToObject(pJsonRoot, pcStr_Msg_Para, pArraySubJson);


    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SetRobotLED(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Set_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}



UBT_RC_T ubtRobot_Msg_Encode_StartRobotAction(int iPort,
        char *pcName, int iRepeat,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;
    cJSON   *pArrayJson = NULL;
    cJSON   *pArraySubJson = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Action);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Action_Start);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    if ((pArrayJson = cJSON_CreateArray()) == NULL)
    {
        cJSON_Delete(pJsonRoot);
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    if ((pArraySubJson = cJSON_CreateObject()) == NULL)
    {
        cJSON_Delete(pJsonRoot);
        cJSON_Delete(pArrayJson);
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pArraySubJson, pcStr_Msg_Name, pcName);
    cJSON_AddNumberToObject(pArraySubJson, pcStr_Msg_Repeat, iRepeat);
    cJSON_AddItemToObject(pJsonRoot, pcStr_Msg_Para, pArraySubJson);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_StartRobotAction(char *pcRecvBuf, int *piTime)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_TotalTime);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_Number)
            {
                *piTime = pNode->valueint;
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Action_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_StopRobotAction(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Action);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Action_Stop);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_StopRobotAction(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Action_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }

        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_VoiceStart(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Voice);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Recognition_Start);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_VoiceStart(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Voice_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_VoiceStop(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Voice);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Recognition_Stop);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_VoiceStop(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Voice_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}



UBT_RC_T ubtRobot_Msg_Encode_VoiceTTS(int iPort,
        int isInterrupted, char *pcTTS,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Voice);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Voice_TTS);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Type_Voice_TTS_IsInterrupted,  isInterrupted);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Data, pcTTS);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_VoiceTTS(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];


    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Voice_Ack))
                {
                    ret = UBT_RC_SUCCESS;
                }
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}

UBT_RC_T ubtRobot_Msg_Encode_EventDetect(char *pcEventType, int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Event);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcEventType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}


UBT_RC_T ubtRobot_Msg_Decode_EventDetect(char *pcRecvBuf, char *pcValue)
{
	UBT_RC_T ubtRet = UBT_RC_FAILED;
	cJSON *pJson = NULL;
	cJSON *pNode = NULL;
	char acCmd[MSG_CMD_STR_MAX_LEN];
	char type[32];

	UBT_RC_T ret = UBT_RC_FAILED;

	/* Check parameters */
	if (NULL == pcRecvBuf)
	{
		return UBT_RC_WRONG_PARAM;
	}
	acCmd[0] = '\0';

	pJson	= cJSON_Parse(pcRecvBuf);
	if (pJson == NULL)
	{
		printf("Parse json message filed!\r\n");
		ret = UBT_RC_SOCKET_DECODE_FAILED;
		return ret;
	}

	do
	{
		pNode	= cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
		if (pNode != NULL)
		{
			if (pNode->type == cJSON_String)
			{
				strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
			}
		}
		pNode	= cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
		if (pNode != NULL)
		{
			if (pNode->type == cJSON_String)
			{
				strncpy(type, pNode->valuestring, sizeof(type));
			}
		}

		pNode	= cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
		if (pNode != NULL)
		{
			if (pNode->type == cJSON_String)
			{
				if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Event_Ack))
				{
					
						pNode	= cJSON_GetObjectItem(pJson, pcStr_Msg_Data);
						if (pNode != NULL)
						{
							if (pNode->type == cJSON_String)
							{
								strcpy(pcValue, pNode->valuestring);
								DebugTrace("OK buttonValue Detected!!!!! pcValue = %s \r\n", pcValue);

								if( !strcmp(pcValue, "0"))
								{
									ubtRet = UBT_RC_FAILED;
								}
								else
								{
									ubtRet = UBT_RC_SUCCESS;
								}
							}
						}

				}

				ret = UBT_RC_SUCCESS;
			}
		}

	}
	while (0);
	cJSON_Delete(pJson);
	return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_VisionDetect(char *pcVisionType, int iPort,
        char *pcSendBuf, int iBufLen, int iTimeout)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Vision);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcVisionType);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Time_Out,  iTimeout);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}


UBT_RC_T ubtRobot_Msg_Decode_VisionDetect(char *pcRecvBuf, char *pcValue)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN];
    char type[32];

    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(type, pNode->valuestring, sizeof(type));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (!strcmp(pNode->valuestring, "ok") && !strcmp(acCmd, pcStr_Msg_Cmd_Vision_Ack))
                {
                    if( !strcmp(type, pcStr_Msg_Type_Vision_Face))
                    {
                        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Data);
                        if (pNode != NULL)
                        {
                            if (pNode->type == cJSON_String)
                            {
                                strcpy(pcValue, pNode->valuestring);
                                DebugTrace("OK faceValue Detected!!!!! pcValue = %s \r\n", pcValue);

                                if( !strcmp(pcValue, "0"))
                                {
                                    ubtRet = UBT_RC_FAILED;
                                }
                                else
                                {
                                    ubtRet = UBT_RC_SUCCESS;
                                }
                            }
                        }
                    }

                    if( !strcmp(type, pcStr_Msg_Type_Vision_Hand))
                    {
                        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Data);
                        if (pNode != NULL)
                        {
                            if (pNode->type == cJSON_String)
                            {
                                strcpy(pcValue, pNode->valuestring);
                                DebugTrace("OK handValue Detected!!!!! pcValue = %s \r\n", pcValue);

                                if( !strcmp(pcValue, "0"))
                                {
                                    ubtRet = UBT_RC_FAILED;
                                }
                                else
                                {
                                    ubtRet = UBT_RC_SUCCESS;
                                }
                            }
                        }
                    }

                }

                ret = UBT_RC_SUCCESS;
            }
        }

    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}

UBT_RC_T ubtRobot_Msg_Encode_ConnectRobot(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if ((pJsonRoot == NULL) || (NULL == pcSendBuf))
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Connect);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port, iPort);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type_Version, UBT_SDK_SW_VER);


    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_ConnectRobot(char *pcRecvBuf, char *pcRobotName, int iRobotNameLen)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acStatus[MSG_CMD_STR_MAX_LEN];

    /* Check parameters */
    if ((NULL == pcRecvBuf) || (NULL == pcRobotName))
    {
        return UBT_RC_WRONG_PARAM;
    }
    acStatus[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (0 != strcmp(pNode->valuestring, pcStr_Msg_Cmd_Connect_Ack))
                {
                    continue;
                }
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Name);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(pcRobotName, pNode->valuestring, iRobotNameLen);
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acStatus, pNode->valuestring, sizeof(acStatus));
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}


UBT_RC_T ubtRobot_Msg_Encode_DisconnectRobot(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if ((pJsonRoot == NULL) || (NULL == pcSendBuf))
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Disconnect);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type_Version, UBT_SDK_SW_VER);


    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_DisconnectRobot(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acName[MSG_CMD_STR_MAX_LEN];
    char acStatus[MSG_CMD_STR_MAX_LEN];
    UBT_RC_T ret = UBT_RC_FAILED;

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acName[0] = '\0';
    acStatus[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ret = UBT_RC_SOCKET_DECODE_FAILED;
        return ret;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (0 != strcmp(pNode->valuestring, pcStr_Msg_Cmd_Disconnect_Ack))
                {
                    continue;
                }
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Name);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acName, pNode->valuestring, sizeof(acName));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acStatus, pNode->valuestring, sizeof(acStatus));
            }
        }
        ubtRet = UBT_RC_SUCCESS;
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}

UBT_RC_T ubtRobot_Msg_Encode_SearchSensor(int iPort,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Sensor_Config);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Sensor_SEARCH);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_SearchSensor(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN],acType[MSG_CMD_STR_MAX_LEN];

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';
    acType[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acType, pNode->valuestring, sizeof(acType));
            }
        }

        pNode = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (strcmp(acCmd, pcStr_Msg_Cmd_Sensor_Config_Ack)) break;
                if (strcmp(acType, pcStr_Msg_Type_Sensor_SEARCH)) break;
                if (!strcmp(pNode->valuestring, "ok"))
                {
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }        
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}
UBT_RC_T ubtRobot_Msg_Encode_ModifySensorID(int iPort,
        char *pcType, int iCurrID,int iDstID,
        char *pcSendBuf, int iBufLen)
{
    cJSON   *pJsonRoot = NULL;
    cJSON   *pSubJson = NULL;

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Sensor_Config);
    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Type, pcStr_Msg_Type_Sensor_MODIFY);
    cJSON_AddNumberToObject(pJsonRoot, pcStr_Msg_Port,  iPort);

    if ((pSubJson = cJSON_CreateObject()) == NULL)
    {
        cJSON_Delete(pJsonRoot);
        printf("Failed to create json message!\r\n");
        return UBT_RC_NORESOURCE;
    }

    cJSON_AddStringToObject(pSubJson, pcStr_Msg_Type, pcType);
    cJSON_AddNumberToObject(pSubJson, pcStr_Msg_Param_Query_Sensor_ID, iCurrID);
    cJSON_AddNumberToObject(pSubJson, pcStr_Msg_Value, iDstID);
    cJSON_AddItemToObject(pJsonRoot, pcStr_Msg_Para, pSubJson);

    strncpy(pcSendBuf, cJSON_Print(pJsonRoot), iBufLen);
    cJSON_Delete(pJsonRoot);

    return UBT_RC_SUCCESS;
}

UBT_RC_T ubtRobot_Msg_Decode_ModifySensorID(char *pcRecvBuf)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    cJSON *pJson = NULL;
    cJSON *pNode = NULL;
    char acCmd[MSG_CMD_STR_MAX_LEN],acType[MSG_CMD_STR_MAX_LEN];

    /* Check parameters */
    if (NULL == pcRecvBuf)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acCmd[0] = '\0';
    acType[0] = '\0';

    pJson   = cJSON_Parse(pcRecvBuf);
    if (pJson == NULL)
    {
        printf("Parse json message filed!\r\n");
        ubtRet = UBT_RC_SOCKET_DECODE_FAILED;
        return ubtRet;
    }

    do
    {
        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Cmd);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acCmd, pNode->valuestring, sizeof(acCmd));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Msg_Type);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                strncpy(acType, pNode->valuestring, sizeof(acType));
            }
        }

        pNode   = cJSON_GetObjectItem(pJson, pcStr_Ret_Msg_Status);
        if (pNode != NULL)
        {
            if (pNode->type == cJSON_String)
            {
                if (strcmp(acCmd, pcStr_Msg_Cmd_Sensor_Config_Ack)) break;
                if (strcmp(acType, pcStr_Msg_Type_Sensor_MODIFY)) break;
                if (!strcmp(pNode->valuestring, "ok"))
                {
                    ubtRet = UBT_RC_SUCCESS;
                }
            }
        }        
    }
    while (0);
    cJSON_Delete(pJson);
    return ubtRet;
}
