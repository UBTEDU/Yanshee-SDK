/**
 * @file            yanshee_api.c
 * @brief           UBT Alpha 1x SDK
 * @author          Cygnus Yang
 * @date            Wednesday, August 16, 2017
 * @version         Initial Draft
 * @par             Copyright (C),  2017-2023, UBT Education
 * @par History:
 * 1.Date:          Wednesday, August 16, 2017
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
#include <dirent.h>
#include <sys/un.h>

#include "cJSON.h"
#include "robotlogstr.h"

#include "ubt_datatypes.h"
#include "ubt_crp_message.h"
#include "yanshee_api.h"


#define MOTION_TYPE_TURNAROUND          "turn around"   /**< Turn around motion */
#define MOTION_TYPE_HEAD            "head"
#define MOTION_DIRECTION_LEFT       "left"
#define MOTION_DIRECTION_RIGHT      "right"
#define MOTION_DIRECTION_BOTHHAND   "both"
#define MOTION_DIRECTION_FRONT      "front"
#define MOTION_DIRECTION_BACK       "back"


/* define all servo index */
#define SERVO_LEFT_SHOULD_INDEX     (1<<0)
#define SERVO_LEFT_ELBOW_INDEX      (1<<1)
#define SERVO_LEFT_WRIST_INDEX      (1<<2)
#define SERVO_RIGHT_SHOULD_INDEX    (1<<3)
#define SERVO_RIGHT_ELBOW_INDEX     (1<<4)
#define SERVO_RIGHT_WRIST_INDEX     (1<<5)
#define SERVO_LEFT_WAIST_INDEX      (1<<6)
#define SERVO_LEFT_HIPBONE_INDEX    (1<<7)
#define SERVO_LEFT_KNEE_INDEX       (1<<8)
#define SERVO_LEFT_ANKLE_INDEX      (1<<9)
#define SERVO_LEFT_FOOT_INDEX       (1<<10)
#define SERVO_RIGHT_WAIST_INDEX     (1<<11)
#define SERVO_RIGHT_HIPBONE_INDEX   (1<<12)
#define SERVO_RIGHT_KNEE_INDEX      (1<<13)
#define SERVO_RIGHT_ANKLE_INDEX     (1<<14)
#define SERVO_RIGHT_FOOT_INDEX      (1<<15)
#define SERVO_HEAD_INDEX            (1<<16)
#define SERVO_RANG_INDEX(min, max)   ((0x1ffff<<(min-1))&(0x1ffff>>(17-(max))))
#define SERVO_MAX_LOW_SPEED 60 //=> 1.2S
#define SERVO_DEFAULT_ANGLE        (0)


/** @brief    Servo's speed
   */
typedef enum
{
    SERVO_MOVE_SPEED_VERY_LOW = 1, /**< Very slow. */
    SERVO_MOVE_SPEED_LOW,          /**< Slow. */
    SERVO_MOVE_SPEED_DEFAULT,      /**< Default */
    SERVO_MOVE_SPEED_FAST,         /**< Fast */
    SERVO_MOVE_SPEED_VERY_FAST,    /**< Very Fast */
    SERVO_MOVE_SPEED_LAST_INVALID  /**< Invalid value */
} SERVO_MOVE_SPEED_e;


/* Socket to robot */
#define SDK_REMOTE_SOCKET_PORT      20001

/* SDK_LOCAL_IP */
#define SDK_LOCAL_IP                "127.0.0.1"
/* Max message length which sent to robot */
#define SDK_MESSAGE_MAX_LEN     (1024)
/* Flags indicate that the timer is used */
#define UBT_ROBOT_TIMER_USED 1
/* Flags indicate that the timer is not used */
#define UBT_ROBOT_TIMER_NOTUSED 0

/* Receive message from robot via this socket */
static int                g_iRobot2SDK        = -1;
/* Receive message from robot via this port */
static int                g_iRobot2SDKPort    = -1;
/* Send message to robot via this socket */
static int                g_iSDK2Robot        = -1;
/* Send message to robot via this port */
static int                g_iSDK2RobotPort    = -1;
/* Socket address struct for sending message to robot */
static struct sockaddr_in g_stSDK2RobotSockAddr;
/* Connected robot infomation */
UBT_ROBOTINFO_T           g_stConnectedRobotInfo;
/* Mutex */
static pthread_mutex_t    stMutex;
/* SDK connecting status */
static int                g_iConnectingStatus = 0;

#define UNIX_PATH_PREVIEW  "preview.d"

#ifdef __DEBUG_PRINT__                                            // 对于DEBUG版本，增加打印信息
#define DebugTrace(...)\
        do{\
            fprintf(stderr,__VA_ARGS__);\
        }while(0)
#else
#define DebugTrace(...)                // 对于RELEASE版本，把__DEBUG_PRINT__宏关闭
#endif


static int _udpServerInit(int *piPort, int iTimeout)
{
    int                fd, iRet, i, iPort;
    struct sockaddr_in addr;
    int                addr_len = sizeof(struct sockaddr_in);
    struct timeval     tsock    = {3, 0};
    struct timeval     tpstart;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("create socket failed!\r\n");
        return -1;
    }

    if (iTimeout > 0)
    {
        tsock.tv_sec = iTimeout;
    }
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tsock, sizeof(tsock)) < 0)
    {
        printf("Set SO_RCVTIMEO setsockopt error:%s\r\n", strerror(errno));
        return -1;
    }
    DebugTrace("Set SO_RCVTIMEO:%ld s\r\n", tsock.tv_sec);

    memset(&addr, 0, addr_len);
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htons(INADDR_ANY);

    for (i = 0; i < 100; i++)
    {
        gettimeofday(&tpstart, NULL);
        srand(tpstart.tv_usec);
        iPort = 9100 + (int) (500.0 * rand() / (RAND_MAX + 1.0));
        addr.sin_port = htons(iPort);
        iRet = bind(fd, (struct sockaddr *) &addr, addr_len);
        if (iRet == 0)
        {
            break;
        }
        else if (EADDRINUSE == errno)
        {
            printf(" udp iPort:%d in use!\r\n", iPort);
        }
        else
        {
            printf("udp server bind failed!\r\n");
        }
    }

    *piPort = iPort;

    if (iRet)
    {
        close(fd);
        return -1;
    }

    return fd;
}

/**
 * @brief:      _ubtMsgSend2Robot
 * @details:    Send UDP message out
 * @param[in]   int iFd         Socket ID
 * @param[in]   char *pcIpAddr  IP address
 * @param[in]   int iPort       socket port
 * @param[in]   char *pcBuf     Send buffer
 * @param[in]   int iLen        Buffer length
 * @param[out]  None
 * @retval:     int
 */
static int _ubtMsgSend2Robot(int iFd, char *pcIpAddr, int iPort, char *pcBuf, int iLen)
{
    int                iRet = 0;
    struct sockaddr_in stAddr;

    stAddr.sin_family      = AF_INET;
    stAddr.sin_port        = htons(g_iSDK2RobotPort);
    stAddr.sin_addr.s_addr = inet_addr(pcIpAddr);

    DebugTrace("Send message to %s, Port %d, Buffer %s, iLen %d, Socket ID %d\n", pcIpAddr, iPort, pcBuf, iLen, iFd);
    iRet = sendto(iFd, pcBuf, iLen, 0, (struct sockaddr *) &stAddr, sizeof(struct sockaddr));
    if (-1 == iRet)
    {
        printf("Send message error. %s\r\n", strerror(errno));
    }
    return iRet;
}

static int _ubtMsgRecvFromRobot(int iFd, char *pcRecvBuf, int iBufLen)
{
    int                iRet     = 0;
    struct sockaddr_in stAddr;
    int                iAddrLen = sizeof(struct sockaddr_in);

    memset(&stAddr, 0, sizeof(stAddr));
    do
    {
        memset(pcRecvBuf, 0, iBufLen);
        iRet = recvfrom(iFd, pcRecvBuf, iBufLen, 0, (struct sockaddr *) &stAddr,
                        (socklen_t *) &iAddrLen);
        DebugTrace("Received from %s, Buffer %s, Length %d, Socket ID %d\n",
                   inet_ntoa(stAddr.sin_addr), pcRecvBuf, iBufLen, iFd);
        if (iRet <= 0)
        {
            if (errno == EINTR)   //(errno == EAGAIN))
            {
                continue;
            }
            /* Socket receiving data timeout */
            DebugTrace("Recevie data timeout. \n");
        }
        break;
    } while (1);

    return iRet;
}


/**
 * @brief:      _ubtSendUDPMsg
 * @details:    Send UDP message
 * @param[in]   int iFd         Socket ID
 * @param[in]   char *pcIP  IP address
 * @param[in]   int iPort       socket port
 * @param[in]   char *pcBuffer    Send buffer
 * @param[in]   int iLen        Buffer length
 * @param[out]  None
 * @retval:     int
 */
static int _ubtSendUDPMsg(char *pcIP, int iPort, char *pcBuffer, int iLen)
{
    int                fd;
    struct sockaddr_in addr;
    int                addr_len = sizeof(addr);

    if ((iPort < 1024) || (iPort > 65535))
    {
        return -1;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("UDP socket open failed!");
        return -1;
    }

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(iPort);
    addr.sin_addr.s_addr = inet_addr(pcIP);
    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
        printf("%s error ip", __FUNCTION__);
        close(fd);
        return -1;
    }

    sendto(fd, pcBuffer, iLen, 0, (struct sockaddr *) &addr, addr_len);
    DebugTrace("%s: Send to:%s iPort:%d, Buffer[%d]:%s \n", __FUNCTION__, pcIP, iPort, iLen, pcBuffer);

    close(fd);
    return 0;
}

/**
 * @brief:      _ubtCommWithRobot
 * @details:    Send and Recv UDP message
 * @param[in]   char *pcIpAddr     Dst IP addr
 * @param[in/out]   char *pcBuffer     pcBuffer
 * @param[in]   int iBufLen        Buffer length
 * @param[in]   int iTimeout      Recv timeout
 * @retval:     UBT_RC_T
 */
static UBT_RC_T _ubtCommWithRobot(char *pcIpAddr, char *pcBuffer, int iBufLen, int iTimeout)
{
    int                iRet, iSocketFd, iPort;
    struct sockaddr_in stAddr;
    int                iAddrLen = sizeof(struct sockaddr_in);
    char               acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char               *pStr;

    iSocketFd = _udpServerInit(&iPort, iTimeout);
    if (iSocketFd < 0)
    {
        printf("Create robot to SDK socket failed!\r\n");
        return UBT_RC_SOCKET_FAILED;
    }

//replace iPort
    //DebugTrace("Debug string:%s-begin\r\n",pcBuffer);
    memset(acSocketBuffer, 0, sizeof(acSocketBuffer));
    pStr = strstr(pcBuffer, pcStr_Msg_Port);
    if (pStr != NULL)
    {
        pStr += sizeof(pcStr_Msg_Port) + 2;
        *pStr = '\0';
        strcpy(acSocketBuffer, pcBuffer);
        sprintf(acSocketBuffer + strlen(acSocketBuffer), " %d", iPort);
        while (*++pStr)
        {
            if ((*pStr < '0') || (*pStr > '9'))
            { break; }
        }
        strcpy(acSocketBuffer + strlen(acSocketBuffer), pStr);
    }
    else
    {
        sprintf(acSocketBuffer, "{ \"%s\":%d, ", pcStr_Msg_Port, iPort);
        strcpy(acSocketBuffer + strlen(acSocketBuffer), pcBuffer + 1);
    }
    //DebugTrace("Debug string:%s-end\r\n",acSocketBuffer);
    pthread_mutex_lock(&stMutex);
    iPort = g_iSDK2RobotPort;
    pthread_mutex_unlock(&stMutex);

    iRet = _ubtSendUDPMsg(pcIpAddr, iPort, acSocketBuffer, strlen(acSocketBuffer));
    if (iRet < 0)
    {
        printf("Send  UDP Msg failed!\r\n");
        return UBT_RC_SOCKET_SENDERROR;
    }

    memset(pcBuffer, 0, iBufLen);
    memset(&stAddr, 0, sizeof(stAddr));
    do
    {
        iRet = recvfrom(iSocketFd, pcBuffer, iBufLen, 0, (struct sockaddr *) &stAddr, (socklen_t *) &iAddrLen);
        DebugTrace("SDK Received from %s, Buffer[Len:%d] %s, \n", inet_ntoa(stAddr.sin_addr), iRet, pcBuffer);
        if (iRet <= 0)
        {
            if (errno == EINTR)   //(errno == EAGAIN))
            {
                continue;
            }
            /* Socket receiving data timeout */
            DebugTrace("Recevie data timeout. \n");
        }
        break;
    } while (1);

    close(iSocketFd);
    if (iRet > 0)
    { return UBT_RC_SUCCESS; }

    return UBT_RC_SOCKET_TIMEOUT;
}


/**
 * @brief:      _ubtTimerTimeout
 * @details:    When the heart beat timer timeout, send the hart beat message
             to robot
 * @param[in]   None
 * @param[out]  None
 * @retval:     void
 */
static void *_ubtTimerTimeout()
{
    char  acIPAddr[UBT_ROBOT_IP_ADDR_LEN];    /**< Robot's IP address */
    int   iPort      = 0;
    cJSON *pJsonRoot = NULL;
    char  *pcSendBuf = NULL;

    acIPAddr[0] = '\0';

    if (!strcmp(acIPAddr, SDK_LOCAL_IP))
    {
        DebugTrace("The target IP is %s. Heartbeat message will not be sent!\n", SDK_LOCAL_IP);
        return NULL;
    }

    pJsonRoot = cJSON_CreateObject();
    if (pJsonRoot == NULL)
    {
        printf("Failed to create json message!\r\n");
        return NULL;
    }

    cJSON_AddStringToObject(pJsonRoot, pcStr_Msg_Cmd, pcStr_Msg_Cmd_Heartbeat);

    pcSendBuf = cJSON_Print(pJsonRoot);

    /* g_iConnectingStatus only read in this timer pthread and written in the main pthread */
    while (1 == g_iConnectingStatus)
    {
        pthread_mutex_lock(&stMutex);

        strncpy(acIPAddr, g_stConnectedRobotInfo.acIpAddr, sizeof(acIPAddr));
        iPort = g_iSDK2RobotPort;
        _ubtMsgSend2Robot(g_iSDK2Robot, acIPAddr, iPort, pcSendBuf, strlen(pcSendBuf));
        pthread_mutex_unlock(&stMutex);
        sleep(5);
    }

    free(pcSendBuf);
    cJSON_Delete(pJsonRoot);


    return NULL;
    /* TODO: Should receive heart beat message from robot */
    /* Start the heart beat timer every 5 seconds */
}


/**
 * @brief:      _ubtTranslat
 * @details:    just for Translat hex to dec number

 * @param[in]   char c
 * @param[out]  int
 * @retval:     char
 */
static int _ubtTranslat(char c)
{
    if (c <= '9' && c >= '0')
    { return c - '0'; }
    if (c >= 'a' && c <= 'f')
    { return c - 87; }
    if (c >= 'A' && c <= 'F')
    { return c - 55; }
    return -1;
}

/**
 * @brief:      _ubt_getAngle
 * @details:    just get one servo angle
 * @param[in]   char *pcAllAngle
 * @param[in]   int  iAngleID
 * @param[out]  int
 * @retval:     n
 */
static int _ubt_getAngle(char *pcAllAngle, int iAngleID)
{
    int iLen, iValue = 0xff;

    iLen = strlen(pcAllAngle);
    if (iLen == 0)
    {
        printf("get Angle null");
        return iValue;
    }
    else if (iLen < iAngleID * 2)
    {
        printf("get %d Angle null", iAngleID);
        return iValue;
    }

    iValue = _ubtTranslat(*(pcAllAngle + (iAngleID - 1) * 2)) * 16;
    iValue += _ubtTranslat(*(pcAllAngle + (iAngleID - 1) * 2 + 1));

    return iValue;
}

/**
 * @brief:      ubtSetServosAngle
 * @details:    Set the servo's acAngle with speed
 * @param[in]   int iIndexMask  bit0 - 16 Servo's index.
 * @param[in]   char *pcAngle   The angle for the servos
 * @param[in]   int iTime       It is the time for servo, the value is smaller, the speed is faster.
 * @param[out]  None
 * @retval:
 */
UBT_RC_T _ubtSetServosAngle(int iIndexMask, char *pcAngle, int iTime)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char     acAllAngle[MAX_SERVO_NUM * 2 + 1];
    int      i;

    if (strlen(pcAngle) <= 0)
    {
        printf("ERR: angle buffer is null!\r\n");
        return UBT_RC_WRONG_PARAM;
    }

    memset(acAllAngle, 'F', sizeof(acAllAngle));  // null is "FF"
    acAllAngle[MAX_SERVO_NUM * 2] = '\0';

    for (i = 0; i < MAX_SERVO_NUM; i++)
    {
        if ((iIndexMask >> i) & 0x01)
        {
            acAllAngle[i * 2]     = *pcAngle;
            acAllAngle[i * 2 + 1] = *(pcAngle + 1);
            pcAngle += 2;
        }
    }

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_SetRobotServo(g_iRobot2SDKPort, acAllAngle, iTime,
                                               acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SetRobotServo(acSocketBuffer);

    return ubtRet;
}

/**
 * @brief:      _ubtSetServoAngle
 * @details:    Build the robot all Angle para
 * @param[in]   char *pcAllAngle   The  all angle for the robot
 * @param[in]   int iAngle      one servo angle(range:0~180, not include 0)
 * @param[in]   int iServoID    the servo id for iAngle
 * @param[out]  None
 * @retval: int
 */
static int _ubtSetServoAngle(char *pcAllAngle, int iAngle, int iServoID)
{
    char acAngle[4];

    if ((iAngle > 180) || (iAngle < 0))
    {
        printf("ERR: Servo %d value is out of range! \r\n", iServoID);
        return -1;
    }
    else if (iAngle == 0)
    {
        return 1;
    }

    snprintf(acAngle, sizeof(acAngle), "%02x", iAngle);
    *(pcAllAngle + (iServoID - 1) * 2)     = acAngle[0];
    *(pcAllAngle + (iServoID - 1) * 2 + 1) = acAngle[1];

    return 0;
}

/**
 * @brief:      _ubt_CreateMotionFrame
 * @details:    create motion frame for hts file
 * @param[in]   char *pcFrame   Created frame data
 * @param[in]   char *pcAngle   Angle data buffer
 * @param[in]   int iServoMask  to be control servo
 * @param[in]   int iSeqFram    current seq in hts file
 * @param[in]   int iTotalFram  total frame in hts file
 * @param[in]   FILE *fd   hts file fd
 * @param[out]  none
 * @retval:     void
 */
static void
_ubt_CreateMotionFrame(char *pcFrame, const char *pcAngle, int iServoMask, int iSeqFram, int iTotalFram, FILE *fd)
{
    char acServo[3];
    int  i, offset = 0, sum = 0;

    acServo[2] = '\0';
    for (i = 0; i < 20; i++) // 20Bytes angle Value
    {
        if ((iServoMask >> i) & 0x01)
        {
            acServo[0] = *(pcAngle + offset);
            acServo[1] = *(pcAngle + offset + 1);
            offset += 2;
            if (offset > strlen(pcAngle))
            {
                printf("ERR: Servo angle buffer not match\r\n");
                pcFrame[8 + i] = 0xFF;
            }
            else
            {
                pcFrame[8 + i] = (char) strtol(acServo, NULL, 16);
            }
        }
        else
        {
            pcFrame[8 + i] = 0xFF;
        }
    }

    // frame status
    if (1 == iSeqFram)
    {
        pcFrame[3] = 0x01;//start frame
    }
    else if (iTotalFram > iSeqFram)
    {
        pcFrame[3] = 0x02;// middle frame
    }
    else
    {
        pcFrame[3] = 0x03;
    } // end frame

    //total frame
    pcFrame[4] = iTotalFram & 0xff;
    pcFrame[5] = (iTotalFram >> 8) & 0xff;

    //frame sequence
    pcFrame[6] = iSeqFram & 0xff;
    pcFrame[7] = (iSeqFram >> 8) & 0xff;

    for (i = 2; i <= 30; i++)
    {
        sum += pcFrame[i];
    }
    pcFrame[31] = sum & 0xff;

    fwrite(pcFrame, 33, 1, fd);
}

/**
 * @brief:      ubtGetSWVersion
 * @details:    Get the robot versions including embedded system, raspberry,
             SDK and servos
 * @param[in]   UBT_ROBOT_SOFTVERSION_TYPE_E eType
 *                                      Please see the defination UBT_ROBOT_SOFTVERSION_TYPE_E
 * @param[out]  char *pcVersion
 *                                      The output buffer for versions.
 *                                      In most cases, the version length is no more than 20 bytes.
 * @param[in]   int iVersionLen
 *                                      The max output buffer for versions length.
 * @param[out]  None
 * @retval:     UBT_RC_T
 */
UBT_RC_T ubtGetSWVersion(UBT_ROBOT_SOFTVERSION_TYPE_E eType, char *pcVersion, int iVersionLen)
{
    UBT_RC_T ubtRet   = UBT_RC_FAILED;
    char     *pcParam = NULL;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char     acParam[16];

    acSocketBuffer[0] = -'\0';
    if (eType == UBT_ROBOT_SOFTVERSION_TYPE_SDK)   //sdk version
    {
        strncpy(pcVersion, UBT_SDK_SW_VER, iVersionLen);
        return UBT_RC_SUCCESS;
    }
    switch (eType)
    {
        case UBT_ROBOT_SOFTVERSION_TYPE_STM32: //stm32
            pcParam = pcStr_Msg_Param_Query_Version_STM32;
            break;

        case UBT_ROBOT_SOFTVERSION_TYPE_RASPI: //raspberry
            pcParam = pcStr_Msg_Param_Query_Version_RaspPi;
            break;

        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS1: //servo
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS2:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS3:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS4:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS5:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS6:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS7:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS8:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS9:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS10:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS11:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS12:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS13:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS14:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS15:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS16:
        case UBT_ROBOT_SOFTVERSION_TYPE_SERVOS17:
            sprintf(acParam, "%s%d", pcStr_Msg_Param_Query_Version_Servos, eType - UBT_ROBOT_SOFTVERSION_TYPE_STM32);
            pcParam = acParam;
            break;
        default:
            return UBT_RC_WRONG_PARAM;
    }


    ubtRet = ubtRobot_Msg_Encode_SWVersion(pcParam,
                                           g_iRobot2SDKPort,
                                           acSocketBuffer,
                                           sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SWVersion(acSocketBuffer, pcVersion, iVersionLen);

    return ubtRet;
}


/**
 * @brief      ubtGetPowerStatus
 * @details    Get the power status
 * @param[in]   N/A
 * @param[out]  piVoltage       Voltage(mv)
 *              piRechargeStatus    Recharge(0 No,1 Yes,2 No battery)
 *              piCapacity          Percent(Range 0~64)
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtGetPowerStatus(int *piVoltage, int *piRechargeStatus, int *piCapacity)
{
    UBT_RC_T            ubtRet   = UBT_RC_FAILED;
    char                *pcType  = NULL;
    char                *pcParam = NULL;
    char                acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    UBT_ROBOT_BATTERY_T stPowerStatus;

    /* Check the parameters */
    if ((NULL == piVoltage) || (NULL == piRechargeStatus) ||
        (NULL == piCapacity))
    {
        return UBT_RC_WRONG_PARAM;
    }

    acSocketBuffer[0] = '\0';
    pcType  = pcStr_Msg_Type_Battery;
    pcParam = NULL;

    ubtRet = ubtRobot_Msg_Encode_RobotStatus(pcStr_Msg_Cmd_Query,
                                             pcType,
                                             pcParam,
                                             g_iRobot2SDKPort,
                                             acSocketBuffer,
                                             sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    memset(&stPowerStatus, 0, sizeof(stPowerStatus));
    ubtRet = ubtRobot_Msg_Decode_RobotStatus(pcType, acSocketBuffer, &stPowerStatus);
    if (UBT_RC_SUCCESS == ubtRet)
    {
        *piVoltage        = stPowerStatus.iVoltage;
        *piRechargeStatus = stPowerStatus.iChargeStatus;
        *piCapacity       = stPowerStatus.iCapacity;
    }

    return ubtRet;
}

/**
 * @brief:      ubtRecordMotion
 * @details:    Read all servo's angle 
 * @param[in]   UBT_ROBOTSERVO_T *pstServosAngle
 * @param[out]  None
 * @retval: UBT_RC_T
 */
UBT_RC_T ubtRecordMotion(UBT_ROBOTSERVO_T *pstServosAngle)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char     ucAllAngle[MAX_SERVO_NUM * 2 + 1];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_ReadRobotServo(g_iRobot2SDKPort, acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_ReadRobotServo(acSocketBuffer, ucAllAngle, sizeof(ucAllAngle));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    pstServosAngle->SERVO1_ANGLE  = _ubt_getAngle(ucAllAngle, 1);
    pstServosAngle->SERVO2_ANGLE  = _ubt_getAngle(ucAllAngle, 2);
    pstServosAngle->SERVO3_ANGLE  = _ubt_getAngle(ucAllAngle, 3);
    pstServosAngle->SERVO4_ANGLE  = _ubt_getAngle(ucAllAngle, 4);
    pstServosAngle->SERVO5_ANGLE  = _ubt_getAngle(ucAllAngle, 5);
    pstServosAngle->SERVO6_ANGLE  = _ubt_getAngle(ucAllAngle, 6);
    pstServosAngle->SERVO7_ANGLE  = _ubt_getAngle(ucAllAngle, 7);
    pstServosAngle->SERVO8_ANGLE  = _ubt_getAngle(ucAllAngle, 8);
    pstServosAngle->SERVO9_ANGLE  = _ubt_getAngle(ucAllAngle, 9);
    pstServosAngle->SERVO10_ANGLE = _ubt_getAngle(ucAllAngle, 10);
    pstServosAngle->SERVO11_ANGLE = _ubt_getAngle(ucAllAngle, 11);
    pstServosAngle->SERVO12_ANGLE = _ubt_getAngle(ucAllAngle, 12);
    pstServosAngle->SERVO13_ANGLE = _ubt_getAngle(ucAllAngle, 13);
    pstServosAngle->SERVO14_ANGLE = _ubt_getAngle(ucAllAngle, 14);
    pstServosAngle->SERVO15_ANGLE = _ubt_getAngle(ucAllAngle, 15);
    pstServosAngle->SERVO16_ANGLE = _ubt_getAngle(ucAllAngle, 16);
    pstServosAngle->SERVO17_ANGLE = _ubt_getAngle(ucAllAngle, 17);

    return ubtRet;
}

/**
 * @brief:      ubtGetServosAngle
 * @details:    Read all servo's angle
 * @param[in]   UBT_ROBOTSERVO_T *pstServosAngle
 * @param[out]  None
 * @retval: UBT_RC_T
 */
UBT_RC_T ubtGetServosAngle(UBT_ROBOTSERVO_T *pstServosAngle)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char     ucAllAngle[MAX_SERVO_NUM * 2 + 1];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_ReadRobotServoHold(g_iRobot2SDKPort, acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_ReadRobotServo(acSocketBuffer, ucAllAngle, sizeof(ucAllAngle));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    pstServosAngle->SERVO1_ANGLE  = _ubt_getAngle(ucAllAngle, 1);
    pstServosAngle->SERVO2_ANGLE  = _ubt_getAngle(ucAllAngle, 2);
    pstServosAngle->SERVO3_ANGLE  = _ubt_getAngle(ucAllAngle, 3);
    pstServosAngle->SERVO4_ANGLE  = _ubt_getAngle(ucAllAngle, 4);
    pstServosAngle->SERVO5_ANGLE  = _ubt_getAngle(ucAllAngle, 5);
    pstServosAngle->SERVO6_ANGLE  = _ubt_getAngle(ucAllAngle, 6);
    pstServosAngle->SERVO7_ANGLE  = _ubt_getAngle(ucAllAngle, 7);
    pstServosAngle->SERVO8_ANGLE  = _ubt_getAngle(ucAllAngle, 8);
    pstServosAngle->SERVO9_ANGLE  = _ubt_getAngle(ucAllAngle, 9);
    pstServosAngle->SERVO10_ANGLE = _ubt_getAngle(ucAllAngle, 10);
    pstServosAngle->SERVO11_ANGLE = _ubt_getAngle(ucAllAngle, 11);
    pstServosAngle->SERVO12_ANGLE = _ubt_getAngle(ucAllAngle, 12);
    pstServosAngle->SERVO13_ANGLE = _ubt_getAngle(ucAllAngle, 13);
    pstServosAngle->SERVO14_ANGLE = _ubt_getAngle(ucAllAngle, 14);
    pstServosAngle->SERVO15_ANGLE = _ubt_getAngle(ucAllAngle, 15);
    pstServosAngle->SERVO16_ANGLE = _ubt_getAngle(ucAllAngle, 16);
    pstServosAngle->SERVO17_ANGLE = _ubt_getAngle(ucAllAngle, 17);

    return ubtRet;
}


/**
 * @brief:      ubtSetServosAngle
 * @details:    Set servo's acAngle with speed
 * @param[in]   UBT_ROBOTSERVO_T *pstServosAngle
 * @param[in]   int iTime   It is the time for servo, the value is smaller, the speed is faster.
 * @param[out]  None
 * @retval: UBT_RC_T
 */
UBT_RC_T ubtSetServosAngle(UBT_ROBOTSERVO_T *pstServosAngle, int iTime)
{
    UBT_RC_T ubtRet = UBT_RC_WRONG_PARAM;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    char     acAllAngle[MAX_SERVO_NUM * 2 + 1];

    memset(acAllAngle, 'F', sizeof(acAllAngle)); // null is "FF"
    acAllAngle[MAX_SERVO_NUM * 2] = '\0';

    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO1_ANGLE, 1) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO2_ANGLE, 2) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO3_ANGLE, 3) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO4_ANGLE, 4) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO5_ANGLE, 5) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO6_ANGLE, 6) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO7_ANGLE, 7) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO8_ANGLE, 8) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO9_ANGLE, 9) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO10_ANGLE, 10) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO11_ANGLE, 11) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO12_ANGLE, 12) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO13_ANGLE, 13) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO14_ANGLE, 14) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO15_ANGLE, 15) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO16_ANGLE, 16) < 0)
    {
        return ubtRet;
    }
    if (_ubtSetServoAngle(acAllAngle, pstServosAngle->SERVO17_ANGLE, 17) < 0)
    {
        return ubtRet;
    }

    ubtRet = UBT_RC_FAILED;
    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_SetRobotServo(g_iRobot2SDKPort, acAllAngle, iTime,
                                               acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SetRobotServo(acSocketBuffer);

    return ubtRet;
}


/**
 * @brief:      ubtSetVolume
 * @details:    Set the volume for the Robot
 * @param[in]   int iVolume  [0-100] Volume percent
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtSetVolume(int iVolume)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_SetRobotVolume(g_iRobot2SDKPort, iVolume,
                                                acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SetRobotVolume(acSocketBuffer);

    return ubtRet;
}

/**
 * @brief      ubtGetVolume
 * @details    Get the volume for the Robot
 * @param[in]   iVolume  [0-100] Volume percent
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtGetVolume(int *piVolume)
{
    int      iVolume  = 0;
    UBT_RC_T ubtRet   = UBT_RC_FAILED;
    char     *pcType  = NULL;
    char     *pcParam = NULL;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    /* Check the parameters */
    if (NULL == piVolume)
    {
        return UBT_RC_WRONG_PARAM;
    }

    acSocketBuffer[0] = '\0';
    pcType  = pcStr_Msg_Type_Volume;
    pcParam = NULL;

    ubtRet = ubtRobot_Msg_Encode_RobotStatus(pcStr_Msg_Cmd_Query,
                                             pcType,
                                             pcParam,
                                             g_iRobot2SDKPort,
                                             acSocketBuffer,
                                             sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_RobotStatus(pcType, acSocketBuffer, &iVolume);
    if (UBT_RC_SUCCESS == ubtRet)
    {
        *piVolume = iVolume;
    }

    return ubtRet;
}

/**
 * @brief:      ubtSetMotion
 * @details:    Set the robot's action
 * @param[in]   char *pcType    crouch
  *                             raise
  *                             stretch
  *                             come on
  *                             wave
  *                             bend
  *                             walk
  *                             turn around
  *                             bow
 * @param[in]   char *pcDirect  left
  *                             right
  *                             both
  *                             front
  *                             back
 * @param[in]   int iSpeed      1/2/3/4/5  The default value is 3
 * @param[in]   int iRepeat     Repeat times. 0 means infinite
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtSetMotion(char *pcType, char *pcDirect, int iSpeed, int iRepeat)
{
    FILE           *fd            = NULL;
    char           acFullName[64] = "\0";
    char           acFrame[33];
    int            i, speed, iTotalTime;
    int            iTotalFrame, iLoop;
    struct timeval tpstart;
    char           acAngle[3];

    if (pcType == NULL)
    {
        printf("SetRobotMotion pcType is null!\r\n");
        return UBT_RC_WRONG_PARAM;
    }

    if (iSpeed <= 0 || iSpeed > 6)
    {
        printf("SetRobotMotion iSpeed is outrange(1~6)\r\n");
        return UBT_RC_WRONG_PARAM;
    }
    speed = SERVO_MAX_LOW_SPEED / iSpeed;

    gettimeofday(&tpstart, NULL);
    srand(tpstart.tv_usec);
    snprintf(acFullName, sizeof(acFullName), "/mnt/1xrobot/tmp/motion%d.hts", (int) (50.0 * rand() / (RAND_MAX + 1.0)));
    if ((fd = fopen(acFullName, "wb+")) == NULL)
    {
        printf("SetRobotMotion create file failed!\r\n");
        return UBT_RC_NORESOURCE;
    }

    memset(acFrame, 0, sizeof(acFrame));
    fwrite(acFrame, sizeof(acFrame), 1, fd);

    acFrame[0]  = 0xFB;//LEAD1
    acFrame[1]  = 0xBF;//LEAD2
    acFrame[2]  = 0x01;//reserve
    acFrame[28] = speed;//run time
    acFrame[29] = 0x00; //time high byte
    acFrame[30] = speed;//time low byte
    acFrame[32] = 0xED;//END
    if (!strcmp(pcType, "crouch"))
    {
        iLoop       = 3;
        iTotalFrame = iRepeat * iLoop;
        for (i      = 0; i < iRepeat; i++)
        {
            _ubt_CreateMotionFrame(acFrame, "5A3A4A6E595A78684559", SERVO_RANG_INDEX(7, 16), i * iLoop + 1, iTotalFrame,
                                   fd);
            _ubt_CreateMotionFrame(acFrame, "5E2E00A5555C86B40E5A", SERVO_RANG_INDEX(7, 16), i * iLoop + 2, iTotalFrame,
                                   fd);
            _ubt_CreateMotionFrame(acFrame, "5A3A4A6E595A78684559", SERVO_RANG_INDEX(7, 16), i * iLoop + 3, iTotalFrame,
                                   fd);
        }
    }
    else if (!strcmp(pcType, "raise"))
    {
        iLoop       = 3;
        iTotalFrame = iRepeat * iLoop;
        if (!strcmp(pcDirect, "left"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(4, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5bad5a", SERVO_RANG_INDEX(4, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(4, 6), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b075a", SERVO_RANG_INDEX(1, 3), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "both"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5b5a5a5a5859", SERVO_RANG_INDEX(1, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b075a5aaf5e", SERVO_RANG_INDEX(1, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b5a5a5a5859", SERVO_RANG_INDEX(1, 6), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else
        {
            printf("raise direct[%s]  error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, "stretch"))
    {
        iLoop       = 3;
        iTotalFrame = iRepeat * iLoop;
        if (!strcmp(pcDirect, "left"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "59585a", SERVO_RANG_INDEX(4, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "b30059", SERVO_RANG_INDEX(4, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "59585a", SERVO_RANG_INDEX(4, 6), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45a", SERVO_RANG_INDEX(1, 3), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b5a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "both"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "595a5a5a5959", SERVO_RANG_INDEX(1, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45ab30059", SERVO_RANG_INDEX(1, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "595a5a5a5959", SERVO_RANG_INDEX(1, 6), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else
        {
            printf("stretch direct:%s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, "come on"))
    {
        iLoop       = 6;
        iTotalFrame = iRepeat * iLoop;
        if (!strcmp(pcDirect, "left"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "59585a", SERVO_RANG_INDEX(4, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "ae0016", SERVO_RANG_INDEX(4, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "ae005c", SERVO_RANG_INDEX(4, 6), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "ad000a", SERVO_RANG_INDEX(4, 6), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "ad005e", SERVO_RANG_INDEX(4, 6), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "59585a", SERVO_RANG_INDEX(4, 6), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "585a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b4ae", SERVO_RANG_INDEX(1, 3), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45e", SERVO_RANG_INDEX(1, 3), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b4a8", SERVO_RANG_INDEX(1, 3), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45c", SERVO_RANG_INDEX(1, 3), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "585a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "both"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "585a5a59585a", SERVO_RANG_INDEX(1, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b4aeae0016", SERVO_RANG_INDEX(1, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45eae005c", SERVO_RANG_INDEX(1, 6), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b4a8ad000a", SERVO_RANG_INDEX(1, 6), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "00b45cad005e", SERVO_RANG_INDEX(1, 6), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "585a5a59585a", SERVO_RANG_INDEX(1, 6), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else
        {
            printf("come on direct:%s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, "wave"))
    {
        iLoop       = 6;
        iTotalFrame = iRepeat * iLoop;
        if (!strcmp(pcDirect, "left"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "595859", SERVO_RANG_INDEX(4, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "591e59", SERVO_RANG_INDEX(4, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "59825a", SERVO_RANG_INDEX(4, 6), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "591d59", SERVO_RANG_INDEX(4, 6), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "59865a", SERVO_RANG_INDEX(4, 6), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "595859", SERVO_RANG_INDEX(4, 6), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "585a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58985a", SERVO_RANG_INDEX(1, 3), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58315a", SERVO_RANG_INDEX(1, 3), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "588f5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58335a", SERVO_RANG_INDEX(1, 3), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "585a5a", SERVO_RANG_INDEX(1, 3), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "both"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "585a5a595859", SERVO_RANG_INDEX(1, 6), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58985a591e59", SERVO_RANG_INDEX(1, 6), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58315a59825a", SERVO_RANG_INDEX(1, 6), i * iLoop + 3, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "588f5a591d59", SERVO_RANG_INDEX(1, 6), i * iLoop + 4, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "58335a59865a", SERVO_RANG_INDEX(1, 6), i * iLoop + 5, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "585a5a595859", SERVO_RANG_INDEX(1, 6), i * iLoop + 6, iTotalFrame, fd);
            }
        }
        else
        {
            printf("wave direct %s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, "bend"))
    {
        iLoop       = 3;
        iTotalFrame = iRepeat * iLoop;
        if (!strcmp(pcDirect, "left"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "597769445a", SERVO_RANG_INDEX(12, 16), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "6788973661", SERVO_RANG_INDEX(12, 16), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "597769445a", SERVO_RANG_INDEX(12, 16), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5a3a4a6e59", SERVO_RANG_INDEX(7, 11), i * iLoop + 1, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5830317252", SERVO_RANG_INDEX(7, 11), i * iLoop + 2, iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5a3a4a6e59", SERVO_RANG_INDEX(7, 11), i * iLoop + 3, iTotalFrame, fd);
            }
        }
        else
        {
            printf("bend direct %s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, "walk"))
    {

        if (!strcmp(pcDirect, "front"))
        {
            iLoop       = 6;
            iTotalFrame = iRepeat * iLoop;
            for (i      = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "602724815565a3904650", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "4932159c664f9a9d3460", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "49160c8962508c8f3367", SERVO_RANG_INDEX(7, 16), i * iLoop + 3,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "4c11207464528c8f3364", SERVO_RANG_INDEX(7, 16), i * iLoop + 4,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "601d247a51667c8e224a", SERVO_RANG_INDEX(7, 16), i * iLoop + 5,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "602724814e699aac264f", SERVO_RANG_INDEX(7, 16), i * iLoop + 6,
                                       iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "back"))
        {
            iLoop       = 6;
            iTotalFrame = iRepeat * iLoop;
            for (i      = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "602a1d8d50649a973b4d", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "602a1d8d506683b10f55", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "612a1d8d5066779a164e", SERVO_RANG_INDEX(7, 16), i * iLoop + 3,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "4e1b1d7a67528e962d66", SERVO_RANG_INDEX(7, 16), i * iLoop + 4,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "4f230c9861528e962d66", SERVO_RANG_INDEX(7, 16), i * iLoop + 5,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "4c371c9967528e962d66", SERVO_RANG_INDEX(7, 16), i * iLoop + 6,
                                       iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "left"))
        {
            iLoop       = 2;
            iTotalFrame = iRepeat * iLoop;
            for (i      = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5a271b8c595c8e9b2855", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "42271b8e67598e9b2855", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, "right"))
        {
            iLoop       = 2;
            iTotalFrame = iRepeat * iLoop;
            for (i      = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "5c271d875c708b972b51", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5c271d875c708b972b51", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
            }
        }
        else
        {
            printf("work direct:%s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, MOTION_TYPE_TURNAROUND))
    {
        iLoop       = 2;
        iTotalFrame = iRepeat * iLoop;
        speed += 5;
        acFrame[28] = speed; //run time
        acFrame[29] = 0x00; //time high byte
        acFrame[30] = speed; //time low byte
        if (!strcmp(pcDirect, MOTION_DIRECTION_LEFT))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "4f10306068636e8e1453", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5a281d895e5c8a972a5a", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
            }
        }
        else if (!strcmp(pcDirect, MOTION_DIRECTION_RIGHT))
        {
            for (i = 0; i < iRepeat; i++)
            {
                _ubt_CreateMotionFrame(acFrame, "54532e9f6369a08a554d", SERVO_RANG_INDEX(7, 16), i * iLoop + 1,
                                       iTotalFrame, fd);
                _ubt_CreateMotionFrame(acFrame, "5b281d875a5a89972a5a", SERVO_RANG_INDEX(7, 16), i * iLoop + 2,
                                       iTotalFrame, fd);
            }
        }
        else
        {
            printf("turn round direct %s error\r\n", pcDirect);
            goto ERR;
        }
    }
    else if (!strcmp(pcType, MOTION_TYPE_HEAD))
    {
        if (fd != NULL)
        { fclose(fd); }
        acAngle[2] = '\0';
        if (!strcmp(pcDirect, MOTION_DIRECTION_LEFT))
        {
            acAngle[0] = '2';
            acAngle[1] = '8';
            return _ubtSetServosAngle(SERVO_HEAD_INDEX, acAngle, iSpeed);
        }
        else if (!strcmp(pcDirect, MOTION_DIRECTION_RIGHT))
        {
            acAngle[0] = '7';
            acAngle[1] = '8';
            return _ubtSetServosAngle(SERVO_HEAD_INDEX, acAngle, iSpeed);
        }
        else if (!strcmp(pcDirect, MOTION_DIRECTION_FRONT))
        {
            acAngle[0] = '5';
            acAngle[1] = 'A';
            return _ubtSetServosAngle(SERVO_HEAD_INDEX, acAngle, iSpeed);
        }
        else
        {
            printf("head direct:%s error\r\n", pcDirect);
            return UBT_RC_WRONG_PARAM;
        }
    }
    else if (!strcmp(pcType, "bow"))
    {
        iLoop       = 5;
        iTotalFrame = iRepeat * iLoop;
        for (i      = 0; i < iRepeat; i++)
        {
            _ubt_CreateMotionFrame(acFrame, "595a5b59585b5a3a4a6e595977674459", SERVO_RANG_INDEX(1, 16), i * iLoop + 1,
                                   iTotalFrame, fd);
            _ubt_CreateMotionFrame(acFrame, "59b45b59005b5a179e3159589d19845b", SERVO_RANG_INDEX(1, 16), i * iLoop + 2,
                                   iTotalFrame, fd);
            _ubt_CreateMotionFrame(acFrame, "59b45b59005a5a179e3259589d18825a", SERVO_RANG_INDEX(1, 16), i * iLoop + 3,
                                   iTotalFrame, fd);
            _ubt_CreateMotionFrame(acFrame, "59b45b59005a5c679e485a5b49186b5a", SERVO_RANG_INDEX(1, 16), i * iLoop + 4,
                                   iTotalFrame, fd);
            _ubt_CreateMotionFrame(acFrame, "595a5b59585b5a3a4a6e595977674459", SERVO_RANG_INDEX(1, 16), i * iLoop + 5,
                                   iTotalFrame, fd);
        }
    }
    else
    {
        printf("The Type[%s] motion is unsurpport!\r\n", pcType);
        goto ERR;
    }

    memset(acFrame, 0, sizeof(acFrame));
    iTotalTime = iTotalFrame * speed * 20;
    acFrame[29] = iTotalTime & 0xff;
    acFrame[30] = (iTotalTime >> 8) & 0xff;
    acFrame[31] = (iTotalTime >> 16) & 0xff;
    acFrame[32] = (iTotalTime >> 24) & 0xff;
    fwrite(acFrame, sizeof(acFrame), 1, fd);
    if (fd != NULL)
    { fclose(fd); }
    return ubtStartAction(acFullName, 1);

    ERR:
    if (fd != NULL)
    { fclose(fd); }
    return UBT_RC_WRONG_PARAM;
}


/**
 * @brief:      ubtReadSensorValue
 * @details:    Read the sensor's value
 * @param[in]   char *pcSensorType  The sensor's type.
 *                                  gryo
 *                                  environment
 *                                  board
 *                                  infrared
 *                                  ultrasonic
 *                                  touch
 *                                  color
 *                                  pressure
 *                                  gas
 * @param[out]  void *pValue        The sensor value. More details please see the defination as below variable type
 *                                  UBT_ROBOTGYRO_SENSOR_T
 *                                  UBT_ROBOTENV_SENSOR_T
 *                                  UBT_ROBOTRASPBOARD_SENSOR_T
 *                                  UBT_ROBOTINFRARED_SENSOR_T
 *                                  UBT_ROBOTULTRASONIC_SENSOR_T
 *                                  UBT_ROBOTTOUCH_SENSOR_T
 *                                  UBT_ROBOTCOLOR_SENSOR_T
 *                                  UBT_ROBOTPRESSURE_SENSOR_T
 *                                  UBT_ROBOTGAS_SENSOR_T
 * @param[in]   int iValueLen       The max length of pValue
 * @retval:
 */
UBT_RC_T ubtReadSensorValue(char *pcSensorType, void *pValue, int iValueLen)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    if ((NULL == pcSensorType) || (NULL == pValue))
    {
        return UBT_RC_WRONG_PARAM;
    }
    acSocketBuffer[0] = '\0';

    ubtRet = ubtRobot_Msg_Encode_ReadSensorValue(pcSensorType, g_iRobot2SDKPort,
                                                 acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_ReadSensorValue(acSocketBuffer, pcSensorType, pValue, iValueLen);

    return ubtRet;
}

/**
 * @brief:      ubtReadSensorValueByAddr
 * @details:    Read the sensor's value by it's type and address
 * @param[in]   char *pcSensorType  The sensor's type.
 *                                  gryo
 *                                  environment
 *                                  board
 *                                  infrared
 *                                  ultrasonic
 *                                  touch
 *                                  color
 *                                  pressure
 *                                  gas
 * @param[in]   int iAddr           The sensor's 7bit I2C address
 * @param[out]  void *pValue        The sensor value. More details please see the defination as below variable type
 *                                  UBT_ROBOTGYRO_SENSOR_T
 *                                  UBT_ROBOTENV_SENSOR_T
 *                                  UBT_ROBOTRASPBOARD_SENSOR_T
 *                                  UBT_ROBOTINFRARED_SENSOR_T
 *                                  UBT_ROBOTULTRASONIC_SENSOR_T
 *                                  UBT_ROBOTTOUCH_SENSOR_T
 *                                  UBT_ROBOTCOLOR_SENSOR_T
 *                                  UBT_ROBOTPRESSURE_SENSOR_T
 *                                  UBT_ROBOTGAS_SENSOR_T
 * @param[in]   int iValueLen       The max length of pValue
 * @retval:
 */
UBT_RC_T ubtReadSensorValueByAddr(char *pcSensorType, int iAddr, void *pValue, int iValueLen)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    if ((NULL == pcSensorType) || (NULL == pValue))
    {
        return UBT_RC_WRONG_PARAM;
    }
    acSocketBuffer[0] = '\0';

    ubtRet = ubtRobot_Msg_Encode_ReadSensorValueByAddr(pcSensorType, iAddr, g_iRobot2SDKPort,
                                                       acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_ReadSensorValue(acSocketBuffer, pcSensorType, pValue, iValueLen);

    return ubtRet;
}

/**
 * @brief:      ubtSetRobotLED
 * @details:    Set the LED mode
 * @param[in]   char *pcType
 *                              botton
 *                              camera
 *                              mic
 * @param[in]   char *pcColor
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
 * @param[in]   char *pcMode
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
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtSetRobotLED(char *pcType, char *pcColor, char *pcMode)
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    if ((NULL == pcColor) || (NULL == pcMode))
    {
        return UBT_RC_WRONG_PARAM;
    }
    if ((0 != strcmp(pcStr_Msg_Type_Recognition_Button, pcType)) &&
        (0 != strcmp(pcStr_Msg_Param_Led_Type_Camera, pcType)) &&
        (0 != strcmp(pcStr_Msg_Param_Led_Type_Mic, pcType)))
    {
        return UBT_RC_WRONG_PARAM;
    }

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_SetRobotLED(g_iRobot2SDKPort, pcType, pcColor, pcMode,
                                             acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SetRobotLED(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief:      ubtStartAction
 * @details:    Let the robot play an action
 * @param[in]   char *pcName  The action file's name For
 *                              example: push up, bow
 * @param[in]   int iRepeat   Repeat times. 0 means infinite
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtStartAction(char *pcName, int iRepeat)
{
    int      iTime  = 0;
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    if (NULL == pcName)
    {
        return UBT_RC_WRONG_PARAM;
    }
    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_StartRobotAction(g_iRobot2SDKPort, pcName, iRepeat,
                                                  acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_StartRobotAction(acSocketBuffer, &iTime);

    while (iRepeat--)
    {
        sleep(iTime / 1000);
    }
    sleep(1);
    return ubtRet;
}


/**
  * @brief      Stop to run the robot action file
  *
  * @return  UBT_RC_T 0 Success,  Others    Failed
  *
  */
UBT_RC_T ubtStopAction()
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_StopRobotAction(g_iRobot2SDKPort,
                                                 acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_StopRobotAction(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief   Start voice recognition
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 *
 */
UBT_RC_T ubtStartAsr()
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_VoiceStart(g_iRobot2SDKPort,
                                            acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_VoiceStart(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief   Stop voice recognition
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 *
 */
UBT_RC_T ubtStopAsr()
{
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_VoiceStop(g_iRobot2SDKPort,
                                           acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_VoiceStop(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief   Play the TTS voice
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 * @param   int isInterrputed   Interrupt the previous TTS, if it is not finished.
 *                              0   Not interrupt the previous TTS
 *                              1   Interrupt the previous TTS, start the current TTS immediately
 * @param   char* pcTTS The message to be sent to TTS
 *
 */
UBT_RC_T ubtStartTts(int isInterrputed, char *pcTTS)
{
    UBT_RC_T       ubtRet = UBT_RC_FAILED;
    struct timeval tsock  = {30, 0};
    char           acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    if (NULL == pcTTS)
    {
        return UBT_RC_WRONG_PARAM;
    }

    ubtRet = ubtRobot_Msg_Encode_VoiceTTS(g_iRobot2SDKPort, isInterrputed, pcTTS,
                                          acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), tsock.tv_sec);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_VoiceTTS(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief   Stop tts
 *
 * @return  UBT_RC_T 0 Success,  Others    Failed
 * @param   
 *
 */
UBT_RC_T ubtStopTts()
{
    UBT_RC_T       ubtRet        = UBT_RC_FAILED;
    int            isInterrputed = 1;
    char           *pcTTS        = " ";
    struct timeval tsock         = {30, 0};
    char           acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    if (NULL == pcTTS)
    {
        return UBT_RC_WRONG_PARAM;
    }

    ubtRet = ubtRobot_Msg_Encode_VoiceTTS(g_iRobot2SDKPort, isInterrputed, pcTTS,
                                          acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), tsock.tv_sec);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_VoiceTTS(acSocketBuffer);
    return ubtRet;
}


/**
 * @brief:  ubtEventDetect
 * @details:    Detect some event include Power button etc.
 * @param[in]   pcEventType
 * @param[in]   iTimeout [range: 10~600 s]
 * @param[out]  pcValue
 * @retval:
 */
UBT_RC_T ubtEventDetect(char *pcEventType, char *pcValue, int iTimeout)
{
    UBT_RC_T       ubtRet = UBT_RC_FAILED;
    char           acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    struct timeval tsock  = {30, 0};

    DebugTrace("ubtEventDetect called! iTimeout = %d ", iTimeout);

    if (NULL == pcEventType)
    {
        return UBT_RC_WRONG_PARAM;
    }

    if ((iTimeout >= 10) && (iTimeout <= 600))
    {
        tsock.tv_sec = iTimeout;
    }

    acSocketBuffer[0] = '\0';

    ubtRet = ubtRobot_Msg_Encode_EventDetect(pcEventType, g_iRobot2SDKPort,
                                             acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), tsock.tv_sec);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_EventDetect(acSocketBuffer, pcValue);
    return ubtRet;
}


/**
 * @brief:      ubtConnectRobot
 * @details:    Connect to Robot
 * @param[in]   char *pcVersion  	The SDK version
 * @param[in]   char *pcRobotName   Robot's name
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtConnectRobot(char *pcIpAddr)
{
    int            iRet   = 0;
    int            iPort  = 0;
    UBT_RC_T       ubtRet = UBT_RC_FAILED;
    char           acRobotName[MSG_CMD_STR_MAX_LEN];
    char           acSocketBuffer[SDK_MESSAGE_MAX_LEN];
    pthread_attr_t attr;
    pthread_t      pid;


    if (NULL == pcIpAddr)
    {
        return UBT_RC_WRONG_PARAM;
    }

    if (!strcmp(pcIpAddr, SDK_LOCAL_IP))
    {
        ubtRet = UBT_RC_SUCCESS;
        strncpy(g_stConnectedRobotInfo.acIpAddr, pcIpAddr, sizeof(g_stConnectedRobotInfo.acIpAddr));
        return ubtRet;
    }

    acRobotName[0]    = '\0';
    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_ConnectRobot(g_iRobot2SDKPort,
                                              acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }
    pthread_mutex_lock(&stMutex);
    iPort = g_iSDK2RobotPort;
    pthread_mutex_unlock(&stMutex);

    iRet = _ubtMsgSend2Robot(g_iSDK2Robot, pcIpAddr,
                             iPort, acSocketBuffer, strlen(acSocketBuffer));
    if (iRet != strlen(acSocketBuffer))
    {
        return UBT_RC_SOCKET_SENDERROR;
    }

    /* Please note, acSocketBuf has already been written when ubtMsgRecvFromRo-
    bot */
    memset(&acSocketBuffer, 0, sizeof(acSocketBuffer));

    do
    {
        iRet = _ubtMsgRecvFromRobot(g_iRobot2SDK, acSocketBuffer, sizeof(acSocketBuffer));
        if (iRet != strlen(acSocketBuffer))
        {
            return UBT_RC_SOCKET_SENDERROR;
        }

        ubtRet = ubtRobot_Msg_Decode_ConnectRobot(acSocketBuffer, acRobotName,
                                                  sizeof(acRobotName));
    } while (UBT_RC_SUCCESS != ubtRet);

    if (UBT_RC_SUCCESS == ubtRet)
    {
        g_iConnectingStatus = 1;
        pthread_mutex_lock(&stMutex);
        strncpy(g_stConnectedRobotInfo.acIpAddr, pcIpAddr, sizeof(g_stConnectedRobotInfo.acIpAddr));
        pthread_mutex_unlock(&stMutex);
        pthread_attr_init(&attr);
        iRet = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (iRet != 0)
        {
            printf("pthread_attr_setdetachstate error \n");
            return UBT_RC_FAILED;
        }
        /* Start the heart beat timer every 5 seconds */
        iRet = pthread_create(&pid, &attr, _ubtTimerTimeout, NULL);
        if (iRet < 0)
        {
            printf("pthread_create failed \n");
            return UBT_RC_FAILED;
        }
    }

    return ubtRet;
}

/**
 * @brief:      ubtDisconnectRobot
 * @details:    Disconnect from the robot
 * @param[in]   char *pcVersion  The SDK version
 * @param[in]   char *pcIPAddr   Robot IP address
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtDisconnectRobot(char *pcIpAddr)
{
    int      iRet   = 0;
    int      iPort  = 0;
    UBT_RC_T ubtRet = UBT_RC_FAILED;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];


    if (NULL == pcIpAddr)
    {
        return UBT_RC_WRONG_PARAM;
    }

    if (!strcmp(pcIpAddr, SDK_LOCAL_IP))
    {
        ubtRet = UBT_RC_SUCCESS;
        g_stConnectedRobotInfo.acIpAddr[0] = '\0';
        return ubtRet;
    }

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_DisconnectRobot(g_iRobot2SDKPort,
                                                 acSocketBuffer, sizeof(acSocketBuffer));

    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }
    pthread_mutex_lock(&stMutex);
    iPort = g_iSDK2RobotPort;
    iRet  = _ubtMsgSend2Robot(g_iSDK2Robot, pcIpAddr,
                              iPort, acSocketBuffer, strlen(acSocketBuffer));
    pthread_mutex_unlock(&stMutex);
    if (iRet != strlen(acSocketBuffer))
    {
        return UBT_RC_SOCKET_SENDERROR;
    }

    //Please note, acSocketBuf has already been written when ubtMsgRecvFromRobot
    iRet = _ubtMsgRecvFromRobot(g_iRobot2SDK, acSocketBuffer, sizeof(acSocketBuffer));
    if (iRet != strlen(acSocketBuffer))
    {
        return UBT_RC_SOCKET_SENDERROR;
    }


    ubtRet = ubtRobot_Msg_Decode_DisconnectRobot(acSocketBuffer);

    if (UBT_RC_SUCCESS == ubtRet)
    {
        g_iConnectingStatus = 0;
        /* Stop the heart beat timer */
        g_stConnectedRobotInfo.acIpAddr[0] = '\0';
    }

    return ubtRet;
}


/**
 * @brief       ubtSearchExtendSensor
 * @details     Search all extend sensor include infrared ultrsonic touch environment press
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtSearchExtendSensor(void)
{
    UBT_RC_T ubtRet = UBT_RC_SUCCESS;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_SearchSensor(g_iRobot2SDKPort, acSocketBuffer, sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 5);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_SearchSensor(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief       ubtModifyExtendSensorID
 * @details     Modify   Yanshee's extend sensor ID
 * @param[in]   pcType   Sensor type
 * @param[in]   iCurrID  Sensor ID
 * @param[in]   iDstID   modify id value 
 * @retval      UBT_RC_T
 */
UBT_RC_T ubtModifyExtendSensorID(char *pcType, int iCurrID, int iDstID)
{
    UBT_RC_T ubtRet = UBT_RC_SUCCESS;
    char     acSocketBuffer[SDK_MESSAGE_MAX_LEN];

    acSocketBuffer[0] = '\0';
    ubtRet = ubtRobot_Msg_Encode_ModifySensorID(g_iRobot2SDKPort, pcType, iCurrID, iDstID, acSocketBuffer,
                                                sizeof(acSocketBuffer));
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = _ubtCommWithRobot(g_stConnectedRobotInfo.acIpAddr, acSocketBuffer, sizeof(acSocketBuffer), 0);
    if (UBT_RC_SUCCESS != ubtRet)
    {
        return ubtRet;
    }

    ubtRet = ubtRobot_Msg_Decode_ModifySensorID(acSocketBuffer);
    return ubtRet;
}

/**
 * @brief:      ubtInitialize
 * @details:    Init the SDK for 1x
 * @param[in]   None
 * @param[out]  None
 * @retval:
 */
UBT_RC_T ubtInitialize()
{
    int ret       = UBT_RC_SUCCESS;
    int iPort     = -1;
    int iSocketFd = -1;

    pthread_mutex_init(&stMutex, NULL);
    memset(&g_stSDK2RobotSockAddr, 0, sizeof(g_stSDK2RobotSockAddr));
    iSocketFd = _udpServerInit(&iPort, 0);
    if (iSocketFd < 0)
    {
        printf("Create robot to SDK socket failed!\r\n");
        return UBT_RC_SOCKET_FAILED;
    }
    g_iRobot2SDKPort = iPort;
    g_iRobot2SDK     = iSocketFd;


    iSocketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (iSocketFd < 0)
    {
        printf("Create socket to robot failed!\r\n");
        return UBT_RC_SOCKET_FAILED;
    }
    g_iSDK2Robot = iSocketFd;
    pthread_mutex_lock(&stMutex);
    g_iSDK2RobotPort = SDK_REMOTE_SOCKET_PORT;
    pthread_mutex_unlock(&stMutex);

    memset(&g_stConnectedRobotInfo, 0, sizeof(g_stConnectedRobotInfo));

    return ret;
}

/**
 * @brief:      ubtDeinitialize
 * @details:    Destroy the SDK for 1x
 * @param[in]   None
 * @param[out]  None
 * @retval:
 */
void ubtDeinitialize()
{

    if (-1 != g_iRobot2SDK)
    {
        close(g_iRobot2SDK);
        g_iRobot2SDK = -1;
    }
    g_iRobot2SDKPort = -1;

    if (-1 != g_iSDK2Robot)
    {
        close(g_iSDK2Robot);
        g_iSDK2Robot = -1;
    }
    g_iSDK2RobotPort = -1;

    memset(&g_stSDK2RobotSockAddr, 0, sizeof(g_stSDK2RobotSockAddr));
    memset(&g_stConnectedRobotInfo, 0, sizeof(g_stConnectedRobotInfo));
    pthread_mutex_destroy(&stMutex);

    return;
}





