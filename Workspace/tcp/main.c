//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - TCP Socket
// Application Overview - This particular application illustrates how this
//                        device can be used as a client or server for TCP
//                        communication.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_TCP_Socket_Application
// or
// docs\examples\CC32xx_TCP_Socket_Application.pdf
//
//*****************************************************************************


//****************************************************************************
//
//! \addtogroup tcp_socket
//! @{
//
//****************************************************************************

// Standard includes
#include <stdlib.h>
#include <string.h>

// simplelink includes 
#include "simplelink.h"
#include "wlan.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_apps_rcm.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "timer.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"

// common interface includes 
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "pinmux.h"


/* Config for the TCP */
#define APPLICATION_NAME        "TCP Socket"
#define APPLICATION_VERSION     "1.1.1"
#define DBG_PRINT               Report

#define IP_ADDR             0xc0a8006E /* 192.168.0.110 */
#define PORT_NUM            5001
#define BUF_SIZE            1400
#define TCP_PACKET_COUNT    1000

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

/***************************END OF TCP CONFIG ************************************/


/**************************CONFIG FOR PWM ****************************************/
/*
// The PWM works based on the following settings:
//     Timer reload interval -> determines the time period of one cycle
//     Timer match value -> determines the duty cycle
// The computation for PWM is as described below:
// System frequency = 80 Mhz.
// For a time period of 20 ms (Servo), 
//    Timer reload value = 80,000,000/(1/20ms) = 1,600,000 cycles
//    Cannot store this in 16-bit reload register, so must use prescaler
//    1,600,000 = 0x186A00. Store lower 16 bits into Load, higher 8 bits into prescale
//    PRESCALE = 0x18, RELOAD = 0x6A00
// Servos rotate using duty cycle, 0.5ms for 0 degrees, 2.5ms for 180 degrees.
//    Timer Match for 0.5ms: 80,000,000/(1/0.5ms) = 40,000 cycles
//      40,000 = 0x9C40, MatchPrescale = 0x0, MatchReload = 0x9C40
//    Timer Match for 2.5ms: 80,000m000/(1/2.5ms) = 200,000 cycles
//      200,000 = 0x30D40, MatchPrescale = 0x3, MatchReload = 0x0D40
*/

#define PWM_PRESCALE 0x18
#define PWM_INTERVAL_RELOAD 0x6A00
#define PWM_0_DEGREES 0x9C40UL          // Actual may be 0xC000
#define PWM_180_DEGREES 0x30D54UL       // Actual may be 0x31400
#define PWM_MATCH_PER_DEGREE (0x30D54UL - 0x9C40UL)/180  // Equates to 0x379 or 889
// Defines for fingers
#define FINGER_THUMB 1
#define FINGER_INDEX 2
#define FINGER_MIDDLE 3
/*********************************** END OF PWM CONFIG ***********************/

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int BsdTcpClient(unsigned short usPort);
int BsdTcpServerSetup(unsigned short usPort);
int BsdTcpServerReceive();
static long WlanConnect();
static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();
/* for PWM */
void SetupTimerPWMMode(unsigned long ulBase, unsigned long ulTimer,
                       unsigned long ulConfig, unsigned char ucInvert);
void InitPWMModules();
void DeInitPWMModules();
void UpdatePWM_Finger(int usDegrees, uint8_t finger);

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM;
volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];

#if defined(ccs) || defined (gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

int ServerSockID; //Hold Server Socket ID
int ServerNewSockID; // Hold New Server Socket ID for Client
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
        return;
    }

    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
                        " BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s,"
                            "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(!pNetAppEvent)
    {
        return;
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
            g_ulIpAddr = pEventData->ip;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                        "Gateway=%d.%d.%d.%d\n\r",
                            SL_IPV4_BYTE(g_ulIpAddr,3),
                            SL_IPV4_BYTE(g_ulIpAddr,2),
                            SL_IPV4_BYTE(g_ulIpAddr,1),
                            SL_IPV4_BYTE(g_ulIpAddr,0),
                            SL_IPV4_BYTE(g_ulGatewayIP,3),
                            SL_IPV4_BYTE(g_ulGatewayIP,2),
                            SL_IPV4_BYTE(g_ulGatewayIP,1),
                            SL_IPV4_BYTE(g_ulGatewayIP,0));
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(!pSock)
    {
        return;
    }

    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
        	UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }

}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************



//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param[in]    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_ulDestinationIp = IP_ADDR;
    g_uiPortNum = PORT_NUM;
    g_ulPacketCount = TCP_PACKET_COUNT;
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}



//****************************************************************************
//
//!    \brief Parse the input IP address from the user
//!
//!    \param[in]                     ucCMD (char pointer to input string)
//!
//!    \return                        0 : if correct IP, -1 : incorrect IP
//
//****************************************************************************
int IpAddressParser(char *ucCMD)
{
    volatile int i=0;
    unsigned int uiUserInputData;
    unsigned long ulUserIpAddress = 0;
    char *ucInpString;
    ucInpString = strtok(ucCMD, ".");
    uiUserInputData = (int)strtoul(ucInpString,0,10);
    while(i<4)
    {
        //
       // Check Whether IP is valid
       //
       if((ucInpString != NULL) && (uiUserInputData < 256))
       {
           ulUserIpAddress |= uiUserInputData;
           if(i < 3)
               ulUserIpAddress = ulUserIpAddress << 8;
           ucInpString=strtok(NULL,".");
           uiUserInputData = (int)strtoul(ucInpString,0,10);
           i++;
       }
       else
       {
           return -1;
       }
    }
    g_ulDestinationIp = ulUserIpAddress;
    return SUCCESS;
}
//****************************************************************************
//
//! \brief Opening a TCP client side socket and sending data
//!
//! This function opens a TCP socket and tries to connect to a Server IP_ADDR
//!    waiting on port PORT_NUM.
//!    If the socket connection is successful then the function will send 1000
//! TCP packets to the server.
//!
//! \param[in]      port number on which the server will be listening on
//!
//! \return    0 on success, -1 on Error.
//
//****************************************************************************
int BsdTcpClient(unsigned short usPort)
{
    int             iCounter;
    short           sTestBufLen;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    long            lLoopCount = 0;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    sTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);       
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

    // sending multiple packets to the TCP server
    while (lLoopCount < g_ulPacketCount)
    {
        // sending packet
        iStatus = sl_Send(iSockID, g_cBsdBuf, sTestBufLen, 0 );
        if( iStatus < 0 )
        {
            // error
            sl_Close(iSockID);
            ASSERT_ON_ERROR(SEND_ERROR);
        }
        lLoopCount++;
    }

    Report("Sent %u packets successfully\n\r",g_ulPacketCount);

    iStatus = sl_Close(iSockID);
    //closing the socket after sending 1000 packets
    ASSERT_ON_ERROR(iStatus);

    return SUCCESS;
}

//****************************************************************************
//
//! \brief Opening a TCP server side socket
//!
//! This function opens a TCP socket in Listen mode
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//****************************************************************************
int BsdTcpServerSetup(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iStatus;
    long            lNonBlocking = 1;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    ServerSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( ServerSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(ServerSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(ServerSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(ServerSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(ServerSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(ServerSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 )
    {
        sl_Close(ServerSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    ServerNewSockID = SL_EAGAIN;

    // waiting for an incoming TCP connection
    while( ServerNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
    	ServerNewSockID = sl_Accept(ServerSockID, ( struct SlSockAddr_t *)&sAddr,
                                (SlSocklen_t*)&iAddrSize);
        if( ServerNewSockID == SL_EAGAIN )
        {
           MAP_UtilsDelay(10000);
        }
        else if( ServerNewSockID < 0 )
        {
            // error
            sl_Close(ServerNewSockID);
            sl_Close(ServerSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
    }

    return SUCCESS;
}
//****************************************************************************
//
//! \brief Recieving data from the client
//!
//! Waiting for cmd from the client and Turn the PWM
//!
//! \return     0 on success, -1 on error.
//!
//! Loop forever and wait for package coming from the client
//****************************************************************************
int BsdTcpServerReceive()
{
	int iStatus;
    /* TODO: Delete once new function PWM_update has been verified
	int zeroDegrees_MatchPrescale = 0x0;
	int zeroDegrees_Match = 0xC000;
	int oneEightyDegrees_MatchPrescale = 0x3;
	int oneEightyDegrees_Match = 0x1400; */

	int cmd;
	while (1)
	{
	   iStatus = sl_Recv(ServerNewSockID, g_cBsdBuf, BUF_SIZE, 0);
	   if( iStatus <= 0 )
	   {
	   // error
		   sl_Close(ServerNewSockID);
	       sl_Close(ServerSockID);
	       ASSERT_ON_ERROR(RECV_ERROR);
	   }
	   else
	   {
		   cmd = atoi(g_cBsdBuf);
		   UART_PRINT("BUFF:%s\n", g_cBsdBuf);
           UpdatePWM_Finger(cmd, FINGER_INDEX);
           //MAP_UtilsDelay(13000000);        // Delay for approx 0.5 second

           /* TODO: Delete once new function PWM_update has been verified
		   if (cmd)
		   {
			   UART_PRINT("Receive cmd: %d - Turn 180 degrees.\n",cmd);               
			   MAP_TimerMatchSet(TIMERA2_BASE, TIMER_B, zeroDegrees_Match);
			   MAP_TimerPrescaleMatchSet(TIMERA2_BASE, TIMER_B, zeroDegrees_MatchPrescale);
			   MAP_UtilsDelay(80000000);
		   }
		   else
		   {
			   UART_PRINT("Receive cmd: %d - Turn 0 degrees.\n",cmd);
			   MAP_TimerMatchSet(TIMERA2_BASE, TIMER_B, oneEightyDegrees_Match);
			   MAP_TimerPrescaleMatchSet(TIMERA2_BASE, TIMER_B, oneEightyDegrees_MatchPrescale);
			   MAP_UtilsDelay(80000000);
		   }
           */
	   }

	}

	/*
	// close the connected socket after receiving from connected TCP client
	iStatus = sl_Close(iNewSockID);
	ASSERT_ON_ERROR(iStatus);
	// close the listening socket
	iStatus = sl_Close(iSockID);
	ASSERT_ON_ERROR(iStatus);
	*/
}

//****************************************************************************
//
//!  \brief Connecting to a WLAN Accesspoint
//!
//!   This function connects to the required AP (SSID_NAME) with Security
//!   parameters specified in te form of macros at the top of this file
//!
//!   \param[in]              None
//!
//!   \return     Status value
//!
//!   \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* Wait */
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        // Wait for WLAN Event
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//****************************************************************************
//
//! Setup the timer in PWM mode
//!
//! \param ulBase is the base address of the timer to be configured
//! \param ulTimer is the timer to be setup (TIMER_A or  TIMER_B)
//! \param ulConfig is the timer configuration setting
//! \param ucInvert is to select the inversion of the output
//!
//! This function
//!    1. The specified timer is setup to operate as PWM
//!
//! \return None.
//
//****************************************************************************
void SetupTimerPWMMode(unsigned long ulBase, unsigned long ulTimer,
                       unsigned long ulConfig, unsigned char ucInvert)
{
    // Set GPT - Configured Timer in PWM mode.
    MAP_TimerConfigure(ulBase,ulConfig);

    // Inverting the timer output if required
    MAP_TimerControlLevel(ulBase,ulTimer,ucInvert);

    // Set the prescaler and load to obtain 20ms
    MAP_TimerPrescaleSet(ulBase,ulTimer,PWM_PRESCALE);
    MAP_TimerLoadSet(ulBase,ulTimer,PWM_INTERVAL_RELOAD);

    // Match value set so as to output level 0
    MAP_TimerPrescaleMatchSet(ulBase,ulTimer,PWM_PRESCALE);
    MAP_TimerMatchSet(ulBase,ulTimer,PWM_INTERVAL_RELOAD);
}

//****************************************************************************
//
//! Sets up the identified timers as PWM to drive the peripherals
//!
//! \param none
//!
//! This function sets up the folowing
//!    1. TIMERA2 (TIMER B) as Index finger (also RED of RGB light)
//!    2. TIMERA3 (TIMER A) as Thumb (also YELLOW of RGB light)
//!    3. TIMERA3 (TIMER B) as Middle finger (also GREEN of RGB light)
//!
//! \return None.
//
//****************************************************************************
void InitPWMModules()
{
    // Initialization of timers to generate PWM output
    // Already done in pinmux so commented out
    //MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    //MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);

    // TIMERA2 (TIMER B), Pin 64 as Index finger
    // Also as RED of RGB light. GPIO 9 --> PWM_5
    SetupTimerPWMMode(TIMERA2_BASE, TIMER_B,
            (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM), 1);

    // TIMERA3 (TIMER B), Pin 01 as Thumb
    // Also as YELLOW of RGB light. GPIO 10 --> PWM_6
    SetupTimerPWMMode(TIMERA3_BASE, TIMER_A,
            (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM), 1);

    // TIMERA3 (TIMER A), Pin 02 as Middle finger
    // Also as GREEN of RGB light. GPIO 11 --> PWM_7
    SetupTimerPWMMode(TIMERA3_BASE, TIMER_B,
            (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM), 1);

    // Enable the timers
    MAP_TimerEnable(TIMERA2_BASE,TIMER_B);
    MAP_TimerEnable(TIMERA3_BASE,TIMER_A);
    MAP_TimerEnable(TIMERA3_BASE,TIMER_B);
}

//****************************************************************************
//
//! Disables the timer PWMs
//!
//! \param none
//!
//! This function disables the timers used
//!
//! \return None.
//
//****************************************************************************
void DeInitPWMModules()
{
    // Disable the peripherals
    MAP_TimerDisable(TIMERA2_BASE, TIMER_B);
    MAP_TimerDisable(TIMERA3_BASE, TIMER_A);
    MAP_TimerDisable(TIMERA3_BASE, TIMER_B);
    MAP_PRCMPeripheralClkDisable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkDisable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
}

//****************************************************************************
//
//! Updates the Duty Cycle of PWM output for approprite finger servo
//
//! \param Degrees -> 0 to 180, finger -> finger according to define
//
//! \return None.
//
//****************************************************************************
void UpdatePWM_Finger(int usDegrees, uint8_t finger)
{
    uint32_t degrees_to_match;
    uint16_t lower_16_bits;
    uint8_t upper_8_bits;

    degrees_to_match = PWM_0_DEGREES + usDegrees * PWM_MATCH_PER_DEGREE;
    lower_16_bits = degrees_to_match & 0xFFFF;
    upper_8_bits = (degrees_to_match & 0xFF0000) >> 16;

    switch(finger) {
        case FINGER_THUMB:
            MAP_TimerMatchSet(TIMERA3_BASE, TIMER_A, lower_16_bits);
            MAP_TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A, upper_8_bits);
            break;
        case FINGER_INDEX:
            MAP_TimerMatchSet(TIMERA2_BASE, TIMER_B, lower_16_bits);
            MAP_TimerPrescaleMatchSet(TIMERA2_BASE, TIMER_B, upper_8_bits);
            break;
        case FINGER_MIDDLE:
            MAP_TimerMatchSet(TIMERA3_BASE, TIMER_B, lower_16_bits);
            MAP_TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B, upper_8_bits);
            break;
        default:
            UART_PRINT("[UpdatePWM_IndexFinger] Invalid Finger input\n");
    }
}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs) || defined (gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long lRetVal = -1;

    //
    // Board Initialization
    //
    BoardInit();

    //
    // Initialize the uDMA
    //
    UDMAInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    // Initialize the PWMs used for the Servo
    //
    InitPWMModules();
    //

    // Display banner
    //
    DisplayBanner(APPLICATION_NAME);
    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();

    if(lRetVal < 0)
    {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
         UART_PRINT("Failed to configure the device in its default state \n\r");

      LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \n\r");

    //
    // Asumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Device started as STATION \n\r");


    UART_PRINT("Connecting to AP: %s ...\r\n",SSID_NAME);

    // Connecting to WLAN AP - Set with static parameters defined at common.h
    // After this call we will be connected and have IP address
    lRetVal = WlanConnect();
    if(lRetVal < 0)
    {
        UART_PRINT("Connection to AP failed \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Connected to AP: %s \n\r",SSID_NAME);

    UART_PRINT("Device IP: %d.%d.%d.%d\n\r\n\r",
                      SL_IPV4_BYTE(g_ulIpAddr,3),
                      SL_IPV4_BYTE(g_ulIpAddr,2),
                      SL_IPV4_BYTE(g_ulIpAddr,1),
                      SL_IPV4_BYTE(g_ulIpAddr,0));

    /* Set up the Board as the TCP - SERVER */
    lRetVal = BsdTcpServerSetup(PORT_NUM);
    if(lRetVal < 0)
    {
        UART_PRINT("TCP Server failed\n\r");
        LOOP_FOREVER();
    }
    /* TCP - SERVER Recieve Data From to Client and Turn the Servo */
    BsdTcpServerReceive();

    UART_PRINT("Exiting Application ...\n\r");

    //
    // power of the Network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
