// compile : 
//     sudo g++ -o test test.cpp COjw_37_Protocol2.cpp COjw_37_Protocol2.h -lpthread
// Run :
//     sudo ./test0

//#define _TEST_RECEIVE
/**
 * @author      Jinwook On ojw5014@hanmail.net
 * @modified    2020.09.--
 * @version     01.00.00 Released
 */

// #define EN_WIRINGPI

#include <iostream>
#include <sys/ioctl.h> // ioctl

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>

#include "COjw_37_Protocol2.h"

// Jinwook, On
#include "stdlib.h"
#include <sys/timeb.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <string.h> // memset, memmove,wmemset, fill, fill_n

#include <list>
#include <algorithm> // 범위 복사에 필요

#ifdef EN_WIRINGPI
#include <wiringPi.h>
#include <wiringSerial.h>
#endif

using namespace std;

float m_afMot[256];
float m_afMot_Pose[256];
int m_nWait_Time = 0;
bool m_bEms = false; // emergency switch
bool m_bProgEnd = false;
bool IsEms() { return (((m_bEms == true) || (m_bProgEnd == true)) ? true : false); }
void Reset() { m_bEms = false; }
bool is_open();
void wait_send();
bool play_frame_string(const char *buff, bool bNoWait);
void command_clear();
void command_set(int nID, float fValue);
void command_set_rpm(int nID, float fRpm);
float calc_evd2angle(int nID, int nValue);
int calc_angle2evd(int nID, float fValue);
int calc_position_time(int nAxis, int nTime, int nDelay, float fAngle);
float calc_raw2rpm(int nID, int nValue);
int calc_rpm2raw(int nID, float fRpm);
float calc_time2rpm(float fDeltaAngle, float fTime);
void move(int nTime_ms, int nDelay);
void move(int nTime_ms, int nDelay, bool bContinue);
void move(int nTime_ms, int nDelay, bool bContinue, int nCommandSize, CCommand_t *aCCommands);
void move_no_wait(int nTime_ms, int nDelay);
void move_no_wait(int nTime_ms, int nDelay, int nCommandSize, CCommand_t *aCCommands);

void setposition_speed();
void setposition();
void sync_clear();
void sync_push_byte(int nID, int nData);
void sync_push_word(int nID, int nData);
void sync_push_dword(int nID, int nData);
void sync_push_angle(int nID, float fAngle);
void sync_push(int nID, byte *pbyDatas, int nDataLength);
void sync_flush(int nAddress);
void send(int nMotorRealID, int nCommand, int nAddress, const byte *pbyDatas, int nDataLength);
int updateCRC(byte *data_blk_ptr, int data_blk_size);
int MakeStuff(byte *pBuff, int nLength);

#define _SIZE_BUFFER 1024
int m_nClientMotionFd = -1;
pthread_t m_thSocket = 0;
int m_nSocketPort = 5000;
void* Thread_Socket(void* arg);

CCommand_t *ListCmdToArray(list<CCommand_t> lst);
bool IsStr(const char *strSrc, const char *strFindWord);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
CParam_t m_aCParam[256];

list<int> m_lstRequestMotors;

int m_nRequest_Address = 0;
int m_nRequest_Address_Size = 0;

bool m_abMot[256];
int m_anMot[256];

int m_anMot_Pose[256];

#ifdef _CMD_LIST
std::list<CCommand_t> m_lstCmdIDs;
#else
CCommand_t m_aCCommand[256];
int m_nCommand = 0;
#endif

bool m_bWaitSend = true;
void WaitTime_In_Protocol2(int nMs);

void send_packet(byte *buffer, int nLength);
void socket_bypass_mode(bool bBypass);
bool is_socket_bypass_mode();
bool socket_open(int nPort);
void sock_close();


#ifdef EN_WIRINGPI
#define _PIN_DIR 1 // gpio18
#define _PIN_LEVEL_LOW  0
#define _PIN_LEVEL_HIGH 1
void PinSetup();
void PinTxEnable();
void PinTxDisable();
#endif

void WaitTime_In_Protocol2(int nMs)
{
	struct timeval tv;
	tv.tv_sec = nMs / 1000;
	tv.tv_usec = (nMs % 1000) * 1000;
	select(0, NULL, NULL, NULL, &tv);
}
void CProtocol2::Wait(int nTime)
{
    if (IsOpen() == false) return;
    if (m_bEms == true) return;

    CTimer_t CTmr;
    CTmr.start();
    
    int nWait = ((nTime < 0) ? m_nWait_Time : nTime);
    m_nWait_Time = 0;
    while (true) { if (CTmr.get() >= nWait) break; WaitTime_In_Protocol2(0); }
}
bool IsStr(const char *strSrc, const char *strFindWord)
{
    const char *str = strstr(strSrc, strFindWord);
    if (str == NULL) return false;
    return true;
}
#ifdef EN_WIRINGPI
void PinSetup()
{    
    if (wiringPiSetup () == -1)
    {
        printf("Unable to start wiringPi: %s\n", strerror (errno)) ;
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        return;
    }
    pinMode(_PIN_DIR, OUTPUT);
}
void PinTxEnable()
{
    digitalWrite( _PIN_DIR, _PIN_LEVEL_HIGH );// RX Disable
}
void PinTxDisable()
{
    digitalWrite( _PIN_DIR, _PIN_LEVEL_LOW );// RX Enable
}
#endif

int m_nTty = 0;
CProtocol2::CProtocol2()									// Initialize
{
    m_nTty = 0;				//file description
    int nCnt = sizeof(m_aCParam) / sizeof(CParam_t);
    printf("nCnt = %d\r\n", nCnt);
    for (int i = 0; i < nCnt; i++)
    {
        m_aCParam[i].SetParam(false);
    }
    //////////////////////////
    #if 0
    std::list<CCommand_t> lstCmd;
    CCommand_t CCmd(10, 100.0f);

    ///////////////////////////////////////////////
    for (int j = 0; j < 1000; j++)
    {
        lstCmd.clear();

        lstCmd.push_back(CCmd);
        CCmd.nID = 11;
        CCmd.fVal = 101.0f;
        lstCmd.push_back(CCmd);
        CCmd.nID = 12;
        CCmd.fVal = 102.0f;
        lstCmd.push_back(CCmd);
               
        printf("Empty = %d, Size=%d\r\n", lstCmd.empty(), lstCmd.size());    
        sleep(1);
    }
    lstCmd.clear();
    printf("[Clear]Empty = %d, Size=%d\r\n", lstCmd.empty(), lstCmd.size());    
    #endif
}
CProtocol2::~CProtocol2()		 							// Destroy
{
    m_bProgEnd = true;
	usleep(100);				
	if (IsOpen() == true) Close();
	if (IsOpen_Socket() == true) Close_Socket();
}

bool CProtocol2::IsOpen() { return is_open(); }
bool is_open() { return ((m_nTty != 0) ? true : false); }
bool CProtocol2::Open(const char  *pcDevice, int nBaudrate, bool bWaitSend)// = -1)//, int nMode = 0)	//serial port open // 
{
    if (IsOpen() == false)
	{
        m_bWaitSend = bWaitSend;

#ifdef EN_WIRINGPI     
        printf("PinSetup()\r\n");
        PinSetup();
        PinTxDisable();
#endif

		// Port Open		
		struct termios newtio;

		m_nTty = open(pcDevice, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);
		if(m_nTty == -1) {
			printf("Failed to open TTY Serial Device\r\n");
		    return false;
		}
        fcntl(m_nTty, F_SETFL, O_RDWR);

		printf("Init Serial[%s]\r\n", pcDevice);
#if 0
		// Thread
		int nRet = pthread_create(&m_thReceive, NULL, Thread_Receive, NULL);
		if (nRet != 0)
		{
			printf("Thread Init Error\r\n");
			return false;
		}
		printf("Init Thread\r\n");
#endif
		memset( &newtio, 0, sizeof(newtio) );

		newtio.c_cflag = (
            (nBaudrate == 9600) ? B9600 : (
                (nBaudrate == 19200) ? B19200 : (
                    (nBaudrate == 38400) ? B38400 : (
                        (nBaudrate == 57600) ? B57600 : (
                            (nBaudrate == 115200) ? B115200 : (
                                (nBaudrate == 1000000) ? B1000000 : (
                                    (nBaudrate == 2000000) ? B2000000 : (
                                        (nBaudrate == 3000000) ? B3000000 : (
                                            (nBaudrate == 4000000) ? B4000000 : (
                                                (nBaudrate == 4500000) ? B2000000 : B57600
                                            )
                                        )
                                    )
                                )
                            )
                        )
                    )
                )
            )
        );//B115200;
		newtio.c_cflag |= CS8;
		newtio.c_cflag |= CLOCAL;
		newtio.c_cflag |= CREAD;
        newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0;
		newtio.c_cc[VMIN]  = 0;


		tcflush (m_nTty, TCIFLUSH );			//reset modem
		tcsetattr(m_nTty, TCSANOW, &newtio );	//save setting
#ifdef EN_WIRINGPI     
        printf("PinSetup()\r\n");
        PinSetup();
        PinTxDisable();
#endif

	}
	if (IsOpen() == false)
	{
		printf("[Open()][Error] Connection Fail - Comport %s, %d\r\n", pcDevice, nBaudrate);
		return false;
	}
    
    return true;
}

void CProtocol2::Close()								//serial port close
{
	if (IsOpen() == true) 
	{
		close(m_nTty);
		printf("Serial Port -> Closed\r\n");
	}
	m_nTty = 0;
}
void    CProtocol2::SetParam(int nID, bool bDirReverse, float fMulti, bool bSetDynamixelPro) { printf("SetParam(%d, %d, %f, %d)\r\n", nID, bDirReverse?true:false, fMulti, bSetDynamixelPro?true:false); m_aCParam[nID].SetParam(bSetDynamixelPro); m_aCParam[nID].m_bDirReverse = bDirReverse; m_aCParam[nID].m_fMulti = fMulti; }
void    CProtocol2::SetParam_Dir(int nID, bool bDirReverse) { printf("SetParam_Dir(%d, %d)\r\n", nID, bDirReverse?true:false); m_aCParam[nID].m_bDirReverse = bDirReverse; }
void    CProtocol2::SetParam_DynamixelPro(int nID, bool bSetDynamixelPro) { printf("SetParam_DynamixelPro(%d, %d)\r\n", nID, bSetDynamixelPro?true:false); m_aCParam[nID].SetParam(bSetDynamixelPro); }
void    CProtocol2::SetParam_Multi(int nID, float fMulti) { printf("SetParam_Multi(%d, %f)\r\n", nID, fMulti); m_aCParam[nID].m_fMulti = fMulti; }

// Calc ////////////////////////////////////
float CProtocol2::CalcEvd2Angle(int nID, int nValue) { return calc_evd2angle(nID, nValue); }
int CProtocol2::CalcAngle2Evd(int nID, float fValue) { return calc_angle2evd(nID, fValue); }
int CProtocol2::CalcPosition_Time(int nAxis, int nTime, int nDelay, float fAngle) { return calc_position_time(nAxis, nTime, nDelay, fAngle); }
float CProtocol2::CalcRaw2Rpm(int nID, int nValue) { return calc_raw2rpm(nID, nValue); }
int CProtocol2::CalcRpm2Raw(int nID, float fRpm) { return calc_rpm2raw(nID, fRpm); }
float CProtocol2::CalcTime2Rpm(float fDeltaAngle, float fTime) { return calc_time2rpm(fDeltaAngle, fTime); }

float calc_evd2angle(int nID, int nValue)
{
    float fMul = m_aCParam[nID].m_fMulti * ((m_aCParam[nID].m_bDirReverse == false) ? 1 : -1);
    if (fMul == 0) fMul = 1;
    float fMechMove = m_aCParam[nID].m_fMechMove;//4096.0;
    float fCenterPos = m_aCParam[nID].m_fCenter;//2048.0;
    float fMechAngle = m_aCParam[nID].m_fMechAngle;//360.0;

    return (((fMechAngle * ((float)nValue - fCenterPos)) / fMechMove) * fMul);
}
int calc_angle2evd(int nID, float fValue)
{
    float fMul = m_aCParam[nID].m_fMulti * ((m_aCParam[nID].m_bDirReverse == false) ? 1 : -1);
    if (fMul == 0) fMul = 1;

    float fMechMove = m_aCParam[nID].m_fMechMove;//4096.0;
    float fCenterPos = m_aCParam[nID].m_fCenter;//2048.0;
    float fMechAngle = m_aCParam[nID].m_fMechAngle;//360.0;
    return (int)Roundf(((fMechMove * fValue) / fMechAngle * fMul + fCenterPos));
}
int calc_position_time(int nAxis, int nTime, int nDelay, float fAngle)
{
    float fRpm = (float)abs(calc_time2rpm(abs(fAngle - m_afMot_Pose[nAxis]), (float)nTime));
    return calc_rpm2raw(nAxis, fRpm);
}
float calc_raw2rpm(int nID, int nValue) { return (float)nValue * m_aCParam[nID].m_fJointRpm; }
int calc_rpm2raw(int nID, float fRpm) { return (int)Roundf(fRpm / m_aCParam[nID].m_fJointRpm); }
float calc_time2rpm(float fDeltaAngle, float fTime) { return (60.0f * fDeltaAngle * 1000.0f) / (360.0f * fTime); }

//////////////////////////////////// Calc //

//////////////////////////////////////////////////////
// Protocol - basic(updateCRC, MakeStuff, SendPacket)
//////////////////////////////////////////////////////
int m_anCrcTable[] = {
   0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2, 
   0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 
   0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132, 
   0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 
   0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 
   0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 
   0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};
void CProtocol2::WaitSend()
{
    wait_send();
}
void wait_send()
{
    if (m_bWaitSend == false) return;
    // if (m_nControlMode == 1) return;
    int txemptystate;
    while( 1 )
    {
        ioctl( m_nTty, TIOCSERGETLSR, &txemptystate);
        if( txemptystate ) break;
    }
}
void CProtocol2::SendPacket(byte *buffer, int nLength)
{
	send_packet(buffer, nLength);
}
void send_packet(byte *buffer, int nLength)
{
	if (is_open() == true)
	{
#ifdef EN_WIRINGPI        
        PinTxEnable();
#endif
#if 0
        for (int i = 0; i < nLength; i++)
        {
            write (m_nTty, &buffer[i], 1) ;
            TxWait();
        }
#else
	    write(m_nTty, buffer, sizeof(byte) * (nLength));// + 2));
#endif
		
        tcflush(m_nTty, TCIFLUSH);
        
        wait_send(); // ttyUSB 일때는 잘 동작
        

#ifdef EN_WIRINGPI     
        PinTxDisable();
#endif
#if 0
		printf("[SendPacket()[%d]]\r\n", nLength);
		for (int nMsg = 0; nMsg < nLength; nMsg++)
		{
			printf("0x%02X, ", buffer[nMsg]);
		}
		printf("\r\n");
#endif
	}
}
void CProtocol2::Send(int nMotorRealID, int nCommand, int nAddress, const byte *pbyDatas, int nDataLength) { send(nMotorRealID, nCommand, nAddress, pbyDatas, nDataLength); }
void send(int nMotorRealID, int nCommand, int nAddress, const byte *pbyDatas, int nDataLength)
{
    int i;
    i = 0;
    int nSize = nDataLength;
    int nLength = 3 + ((pbyDatas != NULL) ? nSize + 2 : 0);
    int nDefaultSize = 7;
    byte *pbyteBuffer = (byte *)malloc(nDefaultSize + nLength);
    pbyteBuffer[i++] = 0xff;
    pbyteBuffer[i++] = 0xff;
    // Packet 2.0 -->
    pbyteBuffer[i++] = 0xfd;
    pbyteBuffer[i++] = 0x00;
    // --> Packet 2.0
    pbyteBuffer[i++] = (byte)(nMotorRealID & 0xff);
    pbyteBuffer[i++] = (byte)(nLength & 0xff);
    pbyteBuffer[i++] = (byte)((nLength >> 8) & 0xff);
    pbyteBuffer[i++] = (byte)(nCommand & 0xff);
    if (pbyDatas != NULL)
    {
        pbyteBuffer[i++] = (byte)(nAddress & 0xff);
        pbyteBuffer[i++] = (byte)((nAddress >> 8) & 0xff);
        for (int j = 0; j < nSize; j++) pbyteBuffer[i++] = pbyDatas[j];
    }
    i = MakeStuff(pbyteBuffer, nDefaultSize + nLength);
    int nCrc = updateCRC(pbyteBuffer, i - 2);
    pbyteBuffer[i - 2] = (byte)(nCrc & 0xff);
    pbyteBuffer[i - 1] = (byte)((nCrc >> 8) & 0xff);
    send_packet(pbyteBuffer, i);
    free(pbyteBuffer);
}

int updateCRC(byte *data_blk_ptr, int data_blk_size)
{
    int nCrc_accum = 0;
    for (int i = 0; i < data_blk_size; i++) nCrc_accum = (nCrc_accum << 8) ^ m_anCrcTable[(((nCrc_accum >> 8) ^ data_blk_ptr[i]) & 0xFF)];
    return nCrc_accum;
}
int MakeStuff(byte *pBuff, int nLength)
{
    int nStuff = 0;
    int nSize = nLength;
    byte *pnIndex = (byte *)malloc(sizeof(byte) * nSize);
    memset(pnIndex, 0, sizeof(byte) * nSize);
    int nCnt = 0;
    // (0)0xff, (1)0xff, (2)0xfd, (3)0x00,     
    // (4)ID != 0xff 이니 검사할 필요 없다.
    for (int i = 5; i < nSize; i++)
    {
        switch (nStuff)
        {
            case 0: { if (pBuff[i] == 0xff) nStuff++; } break;
            case 1: { if (pBuff[i] == 0xff) nStuff++; else nStuff = 0; } break;
            case 2: { nStuff = 0; if (pBuff[i] == 0xfd) { pnIndex[nCnt++] = i; } } break;
        }
    }
    if (nCnt > 0)
    {
        byte *pBuff2 = (byte *)malloc(sizeof(byte) * nSize);
        memcpy(&pBuff2, &pBuff, sizeof(byte) * nSize);

        // Resize
        byte *tmp = (byte *)realloc(pBuff, (nSize + nCnt) * sizeof(byte));
        if (!tmp) {
            printf("insufficient memory\n");
            free(pBuff2);
            free(pnIndex);
            return 0;
        }
        pBuff = tmp;
        
        int nIndex = 0;
        int nPos = 0;
        int i = 5;   
        
        // 내부의 패킷길이값 재 설정 
        pBuff[i++] = (byte)((nSize + nCnt - 7) & 0xff);
        pBuff[i++] = (byte)(((nSize + nCnt - 7) >> 8) & 0xff);
        
        for (i = 7; i < nSize; i++)
        {
            pBuff[i + nPos] = pBuff2[i];
            if (i == pnIndex[nPos])
            {
                pBuff[nIndex + nPos + 1] = 0xfd;
                nPos++;
            }
        }
        free(pBuff2);
    }
    free(pnIndex);
    return nSize + nCnt;
}
////////////////////////////////////////////////////// => Protocol

//////////////////////////////////////////////////////
// Sync Write
//////////////////////////////////////////////////////
//#define _SYNC_LIST
// Sync Write //
int m_nSync_Length = 0;
bool m_IsSync_Error = false;
#ifdef _SYNC_LIST
std::list<byte> m_lstSync;
#else
byte m_abyteSync[1024];
int m_nSync_ByteSize = 0;
#endif

// // Sync_Set, Sync_Push, Sync_Flush
void CProtocol2::Sync_Clear()
{
#ifdef _SYNC_LIST
    m_lstSync.clear();
#else
    m_nSync_ByteSize = 0;
#endif
    m_nSync_Length = 0;
    m_IsSync_Error = false;
}
void CProtocol2::Write(int nID, int nAddress, int nSize, int nValue)
{
    byte abyDatas[nSize];
    for (int i = 0; i < nSize; i++) abyDatas[i] = (byte)((nValue >> (8 * i)) & 0xff);
    Send(nID, 0x03, nAddress, abyDatas, nSize);
}
void CProtocol2::Write_Byte(int nID, int nAddress, int nValue) { Write(nID, nAddress, 1, nValue); }
void CProtocol2::Write_Word(int nID, int nAddress, int nValue) { Write(nID, nAddress, 2, nValue); }
void CProtocol2::Write_DWord(int nID, int nAddress, int nValue) { Write(nID, nAddress, 4, nValue); }

void CProtocol2::Sync_Push_Byte(int nID, int nData)
{
    byte abyDatas[1];
    abyDatas[0] = (byte)(nData & 0xff);
    Sync_Push(nID, abyDatas, 1);
}
void CProtocol2::Sync_Push_Word(int nID, int nData)
{
    byte abyDatas[2];
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    Sync_Push(nID, abyDatas, 2);
}
void CProtocol2::Sync_Push_Dword(int nID, int nData)
{
    byte abyDatas[4];
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    abyDatas[2] = (byte)((nData >> 16) & 0xff);
    abyDatas[3] = (byte)((nData >> 24) & 0xff);
    Sync_Push(nID, abyDatas, 4);
}
void CProtocol2::Sync_Push_Angle(int nID, float fAngle)
{
    byte abyDatas[4];
    int nData = CalcAngle2Evd(nID, fAngle);
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    abyDatas[2] = (byte)((nData >> 16) & 0xff);
    abyDatas[3] = (byte)((nData >> 24) & 0xff);
    Sync_Push(nID, abyDatas, 4);
}
void CProtocol2::Sync_Push(int nID, byte *pbyDatas, int nDataLength)
{
    if (nDataLength > 0)
    {     
        if (m_nSync_Length == 0)
        {
            m_nSync_Length = nDataLength;

            #ifdef _SYNC_LIST
            m_lstSync.push_back((byte)(nDataLength & 0xff));
            m_lstSync.push_back((byte)((nDataLength >> 8) & 0xff));
            #else
            m_abyteSync[m_nSync_ByteSize++] = (byte)(nDataLength & 0xff);
            m_abyteSync[m_nSync_ByteSize++] = (byte)((nDataLength >> 8) & 0xff);
            #endif
        }
        else if (m_nSync_Length != nDataLength)
        {
            printf("Error(Sync_Push) - ID:%d\r\n",nID);
            m_IsSync_Error = true;
            return;
        }

        #ifdef _SYNC_LIST
        m_lstSync.push_back((byte)(nID & 0xff));
        for (int i = 0; i < nDataLength; i++)
        {
            m_lstSync.push_back((byte)pbyDatas[i]);
        }
        #else
        m_abyteSync[m_nSync_ByteSize++] = (byte)(nID & 0xff);
        for (int i = 0; i < nDataLength; i++)
        {
            m_abyteSync[m_nSync_ByteSize++] = (byte)pbyDatas[i];
        }
        #if 0
        printf("m_nSync_ByteSize=%d\r\n", m_nSync_ByteSize);
        for (int i = 0; i < m_nSync_ByteSize; i++)
        {
            printf("%d ", m_abyteSync[i]);
        }
        printf("\r\n");
        #endif
        #endif
    }
}
void CProtocol2::Sync_Flush(int nAddress) { sync_flush(nAddress); }

void sync_clear()
{
#ifdef _SYNC_LIST
    m_lstSync.clear();
#else
    m_nSync_ByteSize = 0;
#endif
    m_nSync_Length = 0;
    m_IsSync_Error = false;
}
void sync_push_byte(int nID, int nData)
{
    byte abyDatas[1];
    abyDatas[0] = (byte)(nData & 0xff);
    sync_push(nID, abyDatas, 1);
}
void sync_push_word(int nID, int nData)
{
    byte abyDatas[2];
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    sync_push(nID, abyDatas, 2);
}
void sync_push_dword(int nID, int nData)
{
    byte abyDatas[4];
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    abyDatas[2] = (byte)((nData >> 16) & 0xff);
    abyDatas[3] = (byte)((nData >> 24) & 0xff);
    sync_push(nID, abyDatas, 4);
}
void sync_push_angle(int nID, float fAngle)
{
    byte abyDatas[4];
    int nData = calc_angle2evd(nID, fAngle);
    abyDatas[0] = (byte)(nData & 0xff);
    abyDatas[1] = (byte)((nData >> 8) & 0xff);
    abyDatas[2] = (byte)((nData >> 16) & 0xff);
    abyDatas[3] = (byte)((nData >> 24) & 0xff);
    sync_push(nID, abyDatas, 4);
}
void sync_push(int nID, byte *pbyDatas, int nDataLength)
{
    if (nDataLength > 0)
    {     
        if (m_nSync_Length == 0)
        {
            m_nSync_Length = nDataLength;

            #ifdef _SYNC_LIST
            m_lstSync.push_back((byte)(nDataLength & 0xff));
            m_lstSync.push_back((byte)((nDataLength >> 8) & 0xff));
            #else
            m_abyteSync[m_nSync_ByteSize++] = (byte)(nDataLength & 0xff);
            m_abyteSync[m_nSync_ByteSize++] = (byte)((nDataLength >> 8) & 0xff);
            #endif
        }
        else if (m_nSync_Length != nDataLength)
        {
            printf("Error(sync_push) - ID:%d\r\n",nID);
            m_IsSync_Error = true;
            return;
        }

        #ifdef _SYNC_LIST
        m_lstSync.push_back((byte)(nID & 0xff));
        for (int i = 0; i < nDataLength; i++)
        {
            m_lstSync.push_back((byte)pbyDatas[i]);
        }
        #else
        m_abyteSync[m_nSync_ByteSize++] = (byte)(nID & 0xff);
        for (int i = 0; i < nDataLength; i++)
        {
            m_abyteSync[m_nSync_ByteSize++] = (byte)pbyDatas[i];
        }
        #if 0
        printf("m_nSync_ByteSize=%d\r\n", m_nSync_ByteSize);
        for (int i = 0; i < m_nSync_ByteSize; i++)
        {
            printf("%d ", m_abyteSync[i]);
        }
        printf("\r\n");
        #endif
        #endif
    }
}
void sync_flush(int nAddress)
{
    if (m_IsSync_Error == false)
    {
        #ifdef _SYNC_LIST
        if (m_lstSync.size() > 0)
        {
            byte *arr = (byte *)malloc(sizeof(byte) * m_lstSync.size());
            if (arr != NULL)
            {
                if (m_lstSync.size() > 0)
                {
                    std::copy(m_lstSync.begin(), m_lstSync.end(), arr);
                    Send(254, 0x83, nAddress, arr, m_lstSync.size());
                }
                free(arr);
            }
        }

        #else
        if (m_nSync_ByteSize > 0)
        {
            send(254, 0x83, nAddress, m_abyteSync, m_nSync_ByteSize);
        }
        #endif
    }
    sync_clear();
}



////////////////////////////////////////////////////// => Sync Write

//////////////////////////////////////////////////////
// Read
//////////////////////////////////////////////////////
const int _WAIT_TIME = 500; // ms
int m_nShowReturnPacket = 0;
void CProtocol2::ShowPacketReturn(int nPacket_0_Disable_1_Enable) { m_nShowReturnPacket = nPacket_0_Disable_1_Enable; }
class CReceive_t{
public:
    int nID = 0;
    int nCmd = 0;
    int nLength_Data = 0;
    int nError = 0;
    std::list<int> lstDatas;
};


int m_nRequestMotors = 0;
#if 1
bool CProtocol2::WaitReceive()
{
    CTimer_t CTmr;
    CTmr.start();

    byte buf[256];
    while (true)
    {
        int nSize = read(m_nTty, buf, 256);
        if (nSize > 0)
        {
            if (is_socket_bypass_mode() == true)
            {
                if (m_nClientMotionFd >= 0) write(m_nClientMotionFd, buf, _SIZE_BUFFER);
            }
            ReceivedPacket(buf, nSize);
            if (m_nRequestMotors <= 0) 
            {
                m_nRequestMotors = 0;
                return true;
            }
        }
        if (CTmr.get() >= _WAIT_TIME)
        {
            printf("Timeover > %d\r\n", _WAIT_TIME);
            break;
        }
        WaitTime_In_Protocol2(1);
    }
    m_nRequestMotors = 0;
    return false;
}

int m_nReceive_Header = 0;
int m_nReceive_Index = 0;
int m_nReceive_ID = 0;
int m_nReceive_Length = 0;
int m_nReceive_Cmd = 0;
int m_nReceive_Length_Check = 0;
int m_nReceive_Error = 0;
std::list<int> m_anReceive_Datas;
void CProtocol2::ReceivedPacket(byte *buffer, int nBufferSize)
{
    bool bShow_StrLetter = false;
    bool bShow_Str = (m_nShowReturnPacket == 0) ? false : true;
    int nPaketLength = nBufferSize;
    string str = "";
    string strLetter = "";
    int nHeader = 0;
    byte val;
    for (int i = 0; i < nPaketLength; i++) {
        val = buffer[i];
        if (bShow_StrLetter)
        {
            if ((val >= 0x20) && (val <= 127))
            {
            }
            else{
            }
        }
        if (bShow_Str)
        {
        }
        byte byData = val;
        int nTmp = m_nReceive_Header % 100;
        if (byData == 0xff) 
        {
            m_nReceive_Header++;
            if ((nTmp > 2) && (nTmp < 10))
            {
                if (m_nReceive_Header >= 100) m_nReceive_Header = 102;
                else m_nReceive_Header = 2;
            }
        }
        else if (nTmp == 2)
        {
            if (byData == 0xfd) 
            {
                if (m_nReceive_Header >= 100) m_nReceive_Header = 110;
                else m_nReceive_Header = 10;
            }
        }
        else if (nTmp == 10)
        {
            if (byData == 0x00) 
            {
                m_nReceive_Header = 100;
                m_nReceive_Index = 1;
                m_nReceive_ID = 0;
                m_nReceive_Length = 0;
                m_nReceive_Cmd = 0;
                m_nReceive_Length_Check = 0;
                m_nReceive_Error = 0;
                m_anReceive_Datas.clear();
            }
        }
        else
        {
            if (m_nReceive_Header >= 100) m_nReceive_Header = 100;
            else m_nReceive_Header = 0;
        } 

        if (m_nReceive_Header >= 100)
        {
            switch(m_nReceive_Index)
            {     
                case 1:
                    m_nReceive_Index++;
                    break;
                case 2:
                    m_nReceive_ID = byData;
                    m_nReceive_Index++;
                    break;       
                case 3:
                    m_nReceive_Length = byData;
                    m_nReceive_Index++;
                    break;
                case 4:
                    m_nReceive_Length += byData * 256;
                    m_nReceive_Index++;
                    m_nReceive_Length_Check = 0;
                    break;
                case 5:
                    m_nReceive_Length_Check++;
                    m_nReceive_Cmd = byData;
                    m_nReceive_Index++;
                    break;
                case 6:
                    m_nReceive_Length_Check++;
                    if (m_nReceive_Length > 3)
                    {
                        m_nReceive_Error = byData;
                        if (m_nReceive_Error != 0) 
                        {
                            printf("[ID:%d]Received: ++++++Error 발생[Code:%d]+++++++\r\n", m_nReceive_ID, m_nReceive_Error);
                            ShowError(m_nReceive_Error);
                            m_nReceive_Header = 0;
                            m_nReceive_Index = 0;
                            break;
                        }
                    }
                    m_nReceive_Index++;
                    break;
                case 7:
                
                    m_nReceive_Length_Check++;
                    if (m_nReceive_Length_Check <= m_nReceive_Length - 2)
                    {
                        m_anReceive_Datas.push_back(byData);
                    }
                    else
                    {
                        CReceive_t CReceive;
                
                        CReceive.nID = m_nReceive_ID;
                        CReceive.nCmd = m_nReceive_Cmd;
                        CReceive.nLength_Data = m_anReceive_Datas.size();
                        int nSize = m_anReceive_Datas.size();
                        byte pbyData[nSize];

                        if (m_nRequestMotors > 0)
                        {
                            if ((m_nReceive_ID >= 0) && (m_nReceive_ID < 253))
                            {
                                
                                std::list<int>::iterator iter; // 반복만을 위한 변수 생성
                                int nBuffer = 0;
                                int nCnt = m_anReceive_Datas.size();
                                for (int i = 0; i < nCnt; i++)
                                { 
                                    pbyData[nBuffer++] = (byte)(m_anReceive_Datas.front() & 0xff);
                                    m_anReceive_Datas.pop_front();
                                }

                                int nVal = 0;
                                switch (CReceive.nLength_Data)
                                {
                                    case 1: nVal = (byte)(pbyData[0]); break;
                                    case 2: nVal = BytesToShort(pbyData, 0); break;
                                    case 4: nVal = BytesToInt(pbyData, 0); break;
                                }

                                if (m_nRequest_Address == m_aCParam[m_nReceive_ID].m_nSet_Position_Address)
                                {
                                    m_anMot[m_nReceive_ID] = nVal;
                                    m_afMot[m_nReceive_ID] = CalcEvd2Angle(CReceive.nID, nVal);
                                    printf("[Receive]Set:%d번 -> %d(%.2f)도)\r\n", m_nReceive_ID, nVal, m_afMot[m_nReceive_ID]);
                                }
                                else if (m_nRequest_Address == m_aCParam[m_nReceive_ID].m_nGet_Position_Address)
                                {
                                    m_anMot_Pose[m_nReceive_ID] = nVal;
                                    m_afMot_Pose[m_nReceive_ID] = CalcEvd2Angle(CReceive.nID, nVal);

                                    m_anMot[m_nReceive_ID] = m_anMot_Pose[m_nReceive_ID];
                                    m_afMot[m_nReceive_ID] = m_afMot_Pose[m_nReceive_ID];
                                    printf("[Receive]Get:%d번 -> %d(%.2f도)\r\n", m_nReceive_ID, nVal, m_afMot[m_nReceive_ID]);
                                }
                                else if (m_nRequest_Address == m_aCParam[m_nReceive_ID].m_nSet_Torq_Address)
                                {
                                    m_abMot[m_nReceive_ID] = ((nVal == 0) ? false:true);
                                }
                            }
                            m_nRequestMotors--;
                            if (m_nRequestMotors <= 0)
                            {
                                
                            }
                        }
                        else
                        {
                        }
                        CReceive.nError = m_nReceive_Error;
                        for (int i = 0; i < nSize; i++)
                        {
                            CReceive.lstDatas.push_back(pbyData[i]);
                        }
                        // to the next...
                        m_nReceive_Index++;
                    }
                    break;
                case 8:               
                    m_nReceive_Length_Check++;
                    if (m_nReceive_Length_Check >= m_nReceive_Length)
                    {
                        if (byData == 0xff) m_nReceive_Header = 1;
                        else m_nReceive_Header = 0;
                        m_nReceive_Index = 0;
                    }
                    break;
            }
        }
    } // for
    if (bShow_StrLetter)
        printf("[˘︹˘ ][Letter] : %s\r\n", strLetter);
    if (bShow_Str)
        printf("[˘︹˘ ] : %s\r\n", str);
}

void CProtocol2::ShowError(int nErrorNumber)
{
    char strRes[256];
    printf("ErrNum[%d]", nErrorNumber);
    switch(nErrorNumber)
    {
        case 0x00:  break;
        case 0x01:  printf("[Result Fail] 전송된 Instruction Packet 을 처리하는데 실패한 경우\0"); 
                    break;
        case 0x02:  printf("[Instruction Error]  정의되지 않은 Instruction 을 사용한 경우\0");
                    printf("Reg Write 없이 Action 을 사용한 경우\0"); 
                    break;
        case 0x03:  printf("[CRC Error]          전송된 Packet 의 CRC 값이 맞지 않는 경우\0"); 
                    break;
        case 0x04:  printf("[Data Range Error]   해당 Address 에 쓰려는 Data 가 최소/최대값의 범위를 벗어난 경우\0"); 
                    break;
        case 0x05:  printf("[Data Length Error]  해당 Address 의 데이터 길이보다 짧은 데이터를 적으려고 한 경우\0");
                    printf("(예: 4 byte로 정의된 항목의 2 byte 만 쓰려고 하는 경우)\0"); 
                    break;
        case 0x06:  printf("[Data Limit Error]   해당 Address 에 쓰려는 Data 가 Limit 값을 벗어난 경우\0"); 
                    break;
        case 0x07:  printf("[Access Errer]       Read Only 혹은 정의되지 않은 Address 에 값을 쓰려고 한 경우\0"); 
                    printf("Write Only 혹은 정의되지 않은 Address 에 값을 읽으려고 한 경우\0"); 
                    printf("Torque Enable(ROM Lock) 상태에서 ROM 영역에 값을 쓰려고 한 경우\0"); 
                    break;   
    }
    printf("\r\n");
    // return strRes;
}
/*
0x01: [Result Fail]        전송된 Instruction Packet 을 처리하는데 실패한 경우
0x02: [Instruction Error]  정의되지 않은 Instruction 을 사용한 경우
                        Reg Write 없이 Action 을 사용한 경우
0x03: [CRC Error]          전송된 Packet 의 CRC 값이 맞지 않는 경우
0x04: [Data Range Error]   해당 Address 에 쓰려는 Data 가 최소/최대값의 범위를 벗어난 경우
0x05: [Data Length Error]  해당 Address 의 데이터 길이보다 짧은 데이터를 적으려고 한 경우
                        (예: 4 byte로 정의된 항목의 2 byte 만 쓰려고 하는 경우)
0x06: [Data Limit Error]   해당 Address 에 쓰려는 Data 가 Limit 값을 벗어난 경우
0x07: [Access Errer]       Read Only 혹은 정의되지 않은 Address 에 값을 쓰려고 한 경우
                        Write Only 혹은 정의되지 않은 Address 에 값을 읽으려고 한 경우
                        Torque Enable(ROM Lock) 상태에서 ROM 영역에 값을 쓰려고 한 경우
*/
#endif
////////////////////////////////////////////////////// => Read

//////////////////////////////////////////////////////
// Read Command
//////////////////////////////////////////////////////
void CProtocol2::SyncRead_With_Address(int nAddress, int nSize, int *anIDs)
{
    Request_Clear();
    int nCnt = sizeof(anIDs) / sizeof(int);
    for (int i = 0; i < nCnt; i++) Request_Push(anIDs[i]);
    Request_Flush(nAddress, nSize);
    WaitReceive();
}

void CProtocol2::SyncRead(int nMotor_Cnt, int *anIDs)
{
    list<int> lstSecond;
    while (true)
    {
        Request_Clear();
        int nCnt = nMotor_Cnt;
        if (lstSecond.size() > 0) nCnt = lstSecond.size();
        //else nCnt = sizeof(anIDs) / sizeof(int);
        int anIDsCurr[nCnt];
        if (lstSecond.size() > 0)
        {
            std::list<int>::iterator iter; // 반복만을 위한 변수 생성
            int i = 0;
            // for(iter=lstSecond.begin(); iter!=lstSecond.end(); iter++) 
            int nLen = lstSecond.size();
            for(i = 0; i < nLen; i++) 
            { 
                anIDsCurr[i] = lstSecond.front();
                lstSecond.pop_front();
            }
        }
        else
        {
            memcpy(anIDsCurr, anIDs, sizeof(int) * nCnt);
        }
        lstSecond.clear();
        if (nCnt > 0)
        {
            for (int i = 0; i < nCnt; i++)
            {
                if (m_aCParam[anIDsCurr[0]].m_nGet_Position_Address != m_aCParam[anIDsCurr[i]].m_nGet_Position_Address)
                {
                    lstSecond.push_back(anIDsCurr[i]);
                }
                else Request_Push(anIDsCurr[i]);
            }
            Request_Flush(m_aCParam[anIDsCurr[0]].m_nGet_Position_Address, m_aCParam[anIDsCurr[0]].m_nGet_Position_Size);
            WaitReceive();
        }
        else break;
        if (lstSecond.size() == 0) break;
    }
    Command_Clear();
}
void CProtocol2::Request(int nMotor, int nCommand, int nData_Size, byte *pbyDatas) { Request_with_RealID(nMotor, nCommand, nData_Size, pbyDatas); }
void CProtocol2::Request_Push(int nMotor) { m_lstRequestMotors.push_back(nMotor); }
void CProtocol2::Request_Clear() { m_lstRequestMotors.clear(); }
void CProtocol2::Request_Flush(int nAddress, int nSize)
{
    byte pbyDatas[4 + m_lstRequestMotors.size()];
    int nPos = 0;
    pbyDatas[nPos++] = (byte)(nAddress & 0xff);
    pbyDatas[nPos++] = (byte)(((nAddress >> 8) & 0xff));
    pbyDatas[nPos++] = (byte)(nSize & 0xff);
    pbyDatas[nPos++] = (byte)(((nSize >> 8) & 0xff));
    m_nRequestMotors = m_lstRequestMotors.size();
    for (int i = 0; i < m_nRequestMotors; i++)
    {
        pbyDatas[nPos++] = (byte)(m_lstRequestMotors.front() & 0xff);
        m_lstRequestMotors.pop_front();
    }

    m_nRequest_Address = nAddress;
    m_nRequest_Address_Size = nSize;

    Request(254, 0x82, nPos, pbyDatas);
    Request_Clear();
}
void CProtocol2::Request_with_RealID(int nMotorRealID, int nCommand, int nData_Size, byte *pbyDatas)
{
    int i = 0;
    int nDataLength = 0;
    if (pbyDatas != NULL)
    {
        int nCnt = nData_Size;
        if (nCnt > 0)
            nDataLength = nCnt;
    }
    int nLength = 3;
    nLength += ((nDataLength > 0) ? nDataLength + 0 : 0);
    int nDefaultSize = 7;
    int nSize_All = nDefaultSize + nLength;
    byte pbyteBuffer[nSize_All];
    pbyteBuffer[i++] = 0xff;
    pbyteBuffer[i++] = 0xff;
    // region Packet 2.0
    pbyteBuffer[i++] = 0xfd;
    pbyteBuffer[i++] = 0x00;
    // endregion Packet 2.0
    pbyteBuffer[i++] = (byte)(nMotorRealID & 0xff);
    pbyteBuffer[i++] = (byte)(nLength & 0xff);
    pbyteBuffer[i++] = (byte)((nLength >> 8) & 0xff);
    pbyteBuffer[i++] = (byte)(nCommand & 0xff);
    if (nDataLength > 0)
    {
        for (int j = 0; j < nDataLength; j++)
            pbyteBuffer[i++] = pbyDatas[j];
    }

    i = MakeStuff(pbyteBuffer, nSize_All);
    int nCrc = updateCRC(pbyteBuffer, i - 2);
    pbyteBuffer[i - 2] = (byte)(nCrc & 0xff);
    pbyteBuffer[i - 1] = (byte)((nCrc >> 8) & 0xff);
    SendPacket(pbyteBuffer, nSize_All);
}
////////////////////////////////////////////////////// => Read Command

//////////////////////////////////////////////////////
// Command
//////////////////////////////////////////////////////
#ifdef _CMD_LIST
void CProtocol2::Command_Clear() { m_lstCmdIDs.clear(); }
void CProtocol2::Command_Set(int nID, float fValue) { CCommand_t CCmd(nID, fValue); m_lstCmdIDs.push_back(CCmd); }
void CProtocol2::Command_Set_Rpm(int nID, float fRpm) { CCommand_t CCmd(nID, CalcRpm2Raw(nID, fRpm)); m_lstCmdIDs.push_back(CCmd); }
#else
void CProtocol2::Command_Clear() { command_clear(); }
void CProtocol2::Command_Set(int nID, float fValue) { command_set(nID, fValue); }
void CProtocol2::Command_Set_Rpm(int nID, float fRpm) { command_set_rpm(nID, fRpm); }
#endif

float CProtocol2::Get(int nID) { return m_afMot[nID]; }

void command_clear() { m_nCommand = 0; }
void command_set(int nID, float fValue) { m_aCCommand[m_nCommand].nID = nID; m_aCCommand[m_nCommand].fVal = fValue; m_nCommand++; }
void command_set_rpm(int nID, float fRpm) { m_aCCommand[m_nCommand].nID = nID; m_aCCommand[m_nCommand].fVal = calc_rpm2raw(nID, fRpm); m_nCommand++; }

void CProtocol2::SetTorq()
{
#ifdef _CMD_LIST
#else
#endif
#ifdef _CMD_LIST
    list<CCommand_t> lstSecond;
    while (true)
    {
        Sync_Clear();
        int nSize = m_lstCmdIDs.size();
        CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : NULL);
        Command_Clear();

        if (lstSecond.size() > 0)
        {
            nSize = lstSecond.size();
            pCCmd = ListCmdToArray(lstSecond);
            lstSecond.clear();
        }
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Torq_Address != m_aCParam[pCCmd[i].nID].m_nSet_Torq_Address)
                {
                    lstSecond.push_back(CCommand_t(pCCmd[i].nID, pCCmd[i].fVal));
                }
                else 
                {
                    Sync_Push_Byte(pCCmd[i].nID, (int)Round(pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Torq_Address);
        }
        if (nSize == 0) break;
    }
#else
    CCommand_t aCSecond[256];
    int nSecond = 0;
    while (true)
    {
        Sync_Clear();
        int nSize = m_nCommand;
        CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : NULL);
        if (nSecond > 0)
        {
            nSize = nSecond;
            pCCmd2 = aCSecond;
            nSecond = 0;
        }
        CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nSize);
        memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nSize);
        
        Command_Clear();

        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Torq_Address != m_aCParam[pCCmd[i].nID].m_nSet_Torq_Address)
                {
                    aCSecond[nSecond].nID = pCCmd[i].nID;
                    aCSecond[nSecond].fVal = pCCmd[i].fVal;
                    nSecond++;
                }
                else 
                {
                    Sync_Push_Byte(pCCmd[i].nID, (int)Round(pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Torq_Address);
        }
        free(pCCmd);

        if (nSize == 0) break;
    }
#endif
    
    Command_Clear();
}
#if 0

int m_nRun_Time = 0;
int m_nRun_Delay = 0;
CCommand_t[] m_aCRun_Commands;
void CProtocol2::FThread_Run()
{
    try
    {
        printf("FThread_Run -> Start\r\n");

        Ojw.CTimer CTmr = new CTimer();
        CTmr.Set();

        int nTime_ms = m_nRun_Time;
        int nDelay = m_nRun_Delay;
        CCommand_t [] aCCommands = m_aCRun_Commands;

        CCommand_t[] CCmd = ((aCCommands.Length > 0) ? aCCommands : ((m_lstCmdIDs.Count > 0) ? m_lstCmdIDs.ToArray() : null));
        Command_Clear();
        if (CCmd.Length > 0)
        {
            float[] afMot = new float[m_afMot.Length];
            float[] afRes = new float[m_afMot.Length];
            Array.Copy(m_afMot, afMot, m_afMot.Length);
            m_bBreak = false;
            while (true)
            {
                if ((IsEms() == true) || (m_bBreak == true))
                {
                    if (m_bBreak == true) m_bBreak = false;
                    for (int i = 0; i < CCmd.Length; i++) { m_afMot_Pose[CCmd[i].nID] = m_afMot[CCmd[i].nID] = afRes[CCmd[i].nID]; }
                    return;
                }

                List<int> lstIDs = new List<int>();
                lstIDs.Clear();
                for (int i = 0; i < CCmd.Length; i++) { lstIDs.Add(CCmd[i].nID); Command_Set(CCmd[i].nID, 0); } //CalcPosition_Time(CCmd[i].nID, nTime_ms, nDelay, CCmd[i].fVal)); }
                SetPosition_Speed();

                float fGet = CTmr.Get();
                float fTmr = (fGet / (float)nTime_ms);
                if (fTmr > 1f) fTmr = 1f;

                // -Delay 탈출
                if (m_bBreak2 == true)
                {
                    m_bBreak2 = false;
                    m_bBreak = true;
                }
                if (m_bBreak == true)
                {
                    float fTmr_Sub = (nDelay >= 0) ? 0 : (fGet / (float)(nTime_ms + nDelay));
                    if (fTmr_Sub >= 1f) break;
                }

                for (int i = 0; i < CCmd.Length; i++) { afRes[CCmd[i].nID] = afMot[CCmd[i].nID] + (CCmd[i].fVal - afMot[CCmd[i].nID]) * fTmr; Command_Set(CCmd[i].nID, afRes[CCmd[i].nID]); }
                SetPosition();

                if (fTmr >= 1f) break;
                Ojw.CTimer.DoEvent();
            }
            for (int i = 0; i < CCmd.Length; i++) { m_afMot_Pose[CCmd[i].nID] = m_afMot[CCmd[i].nID] = afRes[CCmd[i].nID]; }

            while (true)
            {
                if (CTmr.Get() >= (nTime_ms + nDelay)) break;
                Ojw.CTimer.DoEvent();
            }
        }
    }
    catch (Exception ex)
    {
    }
}

bool m_bNext = false;
void CProtocol2::PlayNext() { m_bNext = true; }
void CProtocol2::Play(string strFileName)
{
    if (IsOpen() == false) return;
    
    printf("Play - %s\r\n", strFileName);

    Ojw.CFile CFile = new Ojw.CFile();
    if (CFile.Load(strFileName) == 0) return;

    int nCnt = CFile.Get_Count();
    int nMax = 0;
    for (int i = 0; i < nCnt; i++) 
    {
        string str = CFile.Get(i);
        string[] pstr = str.Split(',');
        int nEnable = Ojw.CConvert.StrToInt(pstr[0]); 
        if (nEnable > 0) nMax = i; 
    }
    for (int i = 0; i <= nMax; i++)
    {
        string str = CFile.Get(i);
        string[] pstr = str.Split(',');

        int nEnable = Ojw.CConvert.StrToInt(pstr[0]);
        int nTime = Ojw.CConvert.StrToInt(pstr[1]);
        int nDelay = Ojw.CConvert.StrToInt(pstr[2]);

        Command_Clear();
        for (int nIndex = 3; nIndex < pstr.Length; nIndex++)
        {
            string[] pstrDatas = pstr[nIndex].Split(':');
            if (pstrDatas.Length > 1)
            {
                int nID = Ojw.CConvert.StrToInt(pstrDatas[0]);
                int nEvd = Ojw.CConvert.StrToInt(pstrDatas[1]);
                Command_Set(nID, (float)Math.Round(CalcEvd2Angle(nID, nEvd)));
            }
        }
        bool bContinue = (i < nMax) ? true : false;
        if (bContinue == true)
        {
            if (m_bNext == true)
            {
                m_bNext = false;
                bContinue = true;
            }
        }
        Move(nTime, nDelay, bContinue);
    }
    printf("Done - %s\r\n", strFileName);
}


#endif

#ifdef _CMD_LIST
// 마지막 모션이 아니라면 bContinue = false
void CProtocol2::Move(int nTime_ms, int nDelay) { list<CCommand_t> lst; lst.clear(); Move(nTime_ms, nDelay, false, lst); }
void CProtocol2::Move(int nTime_ms, int nDelay, bool bContinue) { list<CCommand_t> lst; lst.clear(); Move(nTime_ms, nDelay, bContinue, lst); }
void CProtocol2::Move(int nTime_ms, int nDelay, bool bContinue, list<CCommand_t> aCCommands)
{
    if (IsOpen() == false) return;
    if (IsEms() == true) return;

    CTimer_t CTmr;
    CTmr.start();

    int nCnt_Motors = ((m_lstCmdIDs.size() > 0) ? m_lstCmdIDs.size() : aCCommands.size());
    if (m_lstCmdIDs.size() > 0) nCnt_Motors = m_lstCmdIDs.size();
    CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : ListCmdToArray(aCCommands));
    
    Command_Clear();
    
    if (nCnt_Motors > 0)
    {
        float fGet, fTmr, fTmr_Sub;
        int i;


        int nSize = sizeof(m_afMot) / sizeof(float);
        float afMot[nSize];
        float afRes[nSize];
        memcpy(afMot, m_afMot, sizeof(float) * nSize);
        while (true)
        {
            if (IsEms() == true)
            {
                for (int i = 0; i < nCnt_Motors; i++) { m_afMot_Pose[pCCmd[i].nID] = m_afMot[pCCmd[i].nID] = afRes[pCCmd[i].nID]; }
                return;
            }
            fGet = (float)CTmr.get();
            fTmr = (fGet / (float)nTime_ms);
            if (fTmr > 1.0f) fTmr = 1.0f;
            // -Delay 탈출
            if (bContinue == true)
            {
                fTmr_Sub = (nDelay >= 0) ? 0 : (fGet / (float)(nTime_ms + nDelay));
                if (fTmr_Sub >= 1.0f) break;
            }
            for (int i = 0; i < nCnt_Motors; i++) { Command_Set(pCCmd[i].nID, 0); }
            SetPosition_Speed();
            for (i = 0; i < nCnt_Motors; i++) { afRes[pCCmd[i].nID] = afMot[pCCmd[i].nID] + (pCCmd[i].fVal - afMot[pCCmd[i].nID]) * fTmr; Command_Set(pCCmd[i].nID, afRes[pCCmd[i].nID]); }
            SetPosition();
            if (fTmr >= 1.0f) break;
            WaitTime_In_Protocol2(1);
        }
        for (i = 0; i < nCnt_Motors; i++) { m_afMot_Pose[pCCmd[i].nID] = m_afMot[pCCmd[i].nID] = afRes[pCCmd[i].nID]; }

        while (true)
        {
            if (CTmr.get() >= (nTime_ms + nDelay)) break;
            WaitTime_In_Protocol2(1);
        }
    }
}

void CProtocol2::Wheel(float fRpm)
{
    if (IsOpen() == false) return;
    if (IsEms() == true) return;
    
    int nSize = m_lstCmdIDs.size();
    CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : NULL);
    Command_Clear();
    if (nSize > 0)
    {
        list<int> lstIDs;
        lstIDs.clear();
        for (int i = 0; i < nSize; i++) { lstIDs.push_back(pCCmd[i].nID); }

        for (int i = 0; i < nSize; i++) { Command_Set(pCCmd[i].nID, 0); CalcRpm2Raw(pCCmd[i].nID, fRpm); }
        SetSpeed();
    }
}

void CProtocol2::SetSpeed()
{
    list<CCommand_t> lstSecond;
    while (true)
    {
        Sync_Clear();
        int nSize = m_lstCmdIDs.size();
        CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : NULL);
        Command_Clear();
        if (lstSecond.size() > 0)
        {
            nSize = lstSecond.size();
            pCCmd = ListCmdToArray(lstSecond);
            lstSecond.clear();
        }
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Speed_Address != m_aCParam[pCCmd[i].nID].m_nSet_Speed_Address)
                {
                    lstSecond.push_back(CCommand_t(pCCmd[i].nID, pCCmd[i].fVal));
                }
                else
                {
                    Sync_Push_Dword(pCCmd[i].nID, (int)Round(pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Speed_Address);
        }
        if (lstSecond.size() == 0) break;
    }
}

void CProtocol2::SetPosition_Speed()
{
    list<CCommand_t> lstSecond;
    while (true)
    {
        Sync_Clear();
        int nSize = m_lstCmdIDs.size();
        CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : NULL);
        if (lstSecond.size() > 0)
        {
            nSize = lstSecond.size();
            pCCmd = ListCmdToArray(lstSecond);
            lstSecond.clear();
        }
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Position_Speed_Address != m_aCParam[pCCmd[i].nID].m_nSet_Position_Speed_Address)
                {
                    lstSecond.push_back(CCommand_t(pCCmd[i].nID, pCCmd[i].fVal));
                }
                else
                {
                    Sync_Push_Dword(pCCmd[i].nID, (int)Roundf(pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Position_Speed_Address);
        }
        if (lstSecond.size() == 0) break;
    }
    Command_Clear();
}

void CProtocol2::SetPosition()
{
    list<CCommand_t> lstSecond;
    while (true)
    {
        Sync_Clear();
        int nSize = m_lstCmdIDs.size();
        CCommand_t *pCCmd = ((m_lstCmdIDs.size() > 0) ? ListCmdToArray(m_lstCmdIDs) : NULL);
        if (lstSecond.size() > 0)
        {
            nSize = lstSecond.size();
            pCCmd = ListCmdToArray(lstSecond);
            lstSecond.clear();
        }
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Position_Address != m_aCParam[pCCmd[i].nID].m_nSet_Position_Address)
                {
                    lstSecond.push_back(CCommand_t(pCCmd[i].nID, pCCmd[i].fVal));
                }
                else
                {
                    m_afMot[pCCmd[i].nID] = pCCmd[i].fVal;
                    m_afMot_Pose[pCCmd[i].nID] = pCCmd[i].fVal;
                    Sync_Push_Dword(pCCmd[i].nID, CalcAngle2Evd(pCCmd[i].nID, pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Position_Address);
        }
        if (lstSecond.size() == 0) break;
    }
    Command_Clear();
}
////////////////////////////////////////////////////// => Command

CCommand_t *ListCmdToArray(list<CCommand_t> lst)
{
    if (lst.size() > 0)
    {
        CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * lst.size());
        std::list<CCommand_t>::iterator iter; // 반복만을 위한 변수 생성
        int i = 0;
        for (CCommand_t const &cmd: lst)
        {
            pCCmd[i].nID = cmd.nID;
            pCCmd[i].fVal = cmd.fVal;
            i++;
        }
        return pCCmd;
    }
    return NULL;
}

#else

// 마지막 모션이 아니라면 bContinue = false
void CProtocol2::Move(int nTime_ms, int nDelay) { move(nTime_ms, nDelay, false, 0, NULL); }
void CProtocol2::Move(int nTime_ms, int nDelay, bool bContinue) { move(nTime_ms, nDelay, bContinue, 0, NULL); }
void CProtocol2::Move(int nTime_ms, int nDelay, bool bContinue, int nCommandSize, CCommand_t *aCCommands) { move(nTime_ms, nDelay, bContinue, nCommandSize, aCCommands); }

void move(int nTime_ms, int nDelay) { move(nTime_ms, nDelay, false, 0, NULL); }
void move(int nTime_ms, int nDelay, bool bContinue) { move(nTime_ms, nDelay, bContinue, 0, NULL); }
void move(int nTime_ms, int nDelay, bool bContinue, int nCommandSize, CCommand_t *aCCommands)
{
    m_nWait_Time = 0;
    if (is_open() == false) return;
    if (IsEms() == true) return;

    CTimer_t CTmr;
    CTmr.start();

    int nCnt_Motors = ((m_nCommand > 0) ? m_nCommand : nCommandSize);
    if (m_nCommand > 0) nCnt_Motors = m_nCommand;
    CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : aCCommands);
    CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nCnt_Motors);
    memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nCnt_Motors);
    
    command_clear();
    
    if (nCnt_Motors > 0)
    {
        int nSize = sizeof(m_afMot) / sizeof(float);
        float afMot[nSize];
        float afRes[nSize];
        
        memcpy(afMot, m_afMot, sizeof(float) * nSize);

        while (true)
        {
            float fGet = (float)CTmr.get();
            float fTmr = (fGet / (float)nTime_ms);
            if (fTmr > 1.0f) fTmr = 1.0f;
            
            // -Delay 탈출
            if (bContinue == true)
            {
                float fTmr_Sub = (nDelay >= 0) ? 0 : (fGet / (float)(nTime_ms + nDelay));
                if (fTmr_Sub >= 1.0f) break;
            }

            command_clear();
            for (int j = 0; j < nCnt_Motors; j++) { 
                command_set(pCCmd[j].nID, 0);
            }
            setposition_speed();
            
            command_clear();
            for (int i = 0; i < nCnt_Motors; i++) { 
                afRes[pCCmd[i].nID] = afMot[pCCmd[i].nID] + (pCCmd[i].fVal - afMot[pCCmd[i].nID]) * fTmr; 
                command_set(pCCmd[i].nID, afRes[pCCmd[i].nID]); 
            }
            if (IsEms() == true)
            {
                free(pCCmd);
                return;
            }
            setposition();

            if (fTmr >= 1.0f) break;
            WaitTime_In_Protocol2(10);
        }

        while (true)
        {
            if (CTmr.get() >= (nTime_ms + nDelay)) break;
        }
    }
    free(pCCmd);
}

void CProtocol2::Move_NoWait(int nTime_ms, int nDelay) { Move_NoWait(nTime_ms, nDelay, 0, NULL); }
void CProtocol2::Move_NoWait(int nTime_ms, int nDelay, int nCommandSize, CCommand_t *aCCommands) { move_no_wait(nTime_ms, nDelay, nCommandSize, aCCommands); }
void move_no_wait(int nTime_ms, int nDelay) { move_no_wait(nTime_ms, nDelay, 0, NULL); }
void move_no_wait(int nTime_ms, int nDelay, int nCommandSize, CCommand_t *aCCommands)
{
    if (is_open() == false) return;
    if (IsEms() == true) return;

    m_nWait_Time = (nTime_ms + nDelay);

    int nCnt_Motors = ((m_nCommand > 0) ? m_nCommand : nCommandSize);
    if (m_nCommand > 0) nCnt_Motors = m_nCommand;
    CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : aCCommands);
    CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nCnt_Motors);
    memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nCnt_Motors);
    
    command_clear();
    
    if (nCnt_Motors > 0)
    {
        int nSize = sizeof(m_afMot) / sizeof(float);  
        float afMot[nSize];
        memcpy(afMot, m_afMot, sizeof(float) * nSize);
        command_clear();
        
        for (int j = 0; j < nCnt_Motors; j++) { 
            command_set(pCCmd[j].nID, calc_position_time(pCCmd[j].nID, nTime_ms, nDelay, pCCmd[j].fVal));
        }
        setposition_speed();
        
        command_clear();
        for (int i = 0; i < nCnt_Motors; i++) { 
            command_set(pCCmd[i].nID, pCCmd[i].fVal); 
        }
        if (IsEms() == true)
        {
            free(pCCmd);
            return;
        }
        setposition();

    }
    free(pCCmd);
}

void CProtocol2::Wheel(float fRpm)
{
    if (IsOpen() == false) return;
    if (IsEms() == true) return;
    
    int nSize = m_nCommand;
    
    CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : NULL);
    CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nSize);
    memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nSize);

    Command_Clear();
    if (nSize > 0)
    {
        for (int i = 0; i < nSize; i++) { Command_Set(pCCmd[i].nID, 0); CalcRpm2Raw(pCCmd[i].nID, fRpm); }
        SetSpeed();
    }
    free(pCCmd);
}

void CProtocol2::SetSpeed()
{
    CCommand_t aCSecond[256];
    int nSecond = 0;
    while (true)
    {
        Sync_Clear();
        int nSize = m_nCommand;
        CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : NULL);
        if (nSecond > 0)
        {
            nSize = nSecond;
            pCCmd2 = aCSecond;
            nSecond = 0;
        }
        
        CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nSize);
        memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nSize);
        
        Command_Clear();
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Speed_Address != m_aCParam[pCCmd[i].nID].m_nSet_Speed_Address)
                {
                    aCSecond[nSecond].nID = pCCmd[i].nID;
                    aCSecond[nSecond].fVal = pCCmd[i].fVal;
                    nSecond++;
                }
                else
                {
                    Sync_Push_Dword(pCCmd[i].nID, (int)Round(pCCmd[i].fVal));
                }
            }
            Sync_Flush(m_aCParam[pCCmd[0].nID].m_nSet_Speed_Address);
        }
        free(pCCmd);
        if (nSecond == 0) break;
    }
}

void CProtocol2::SetPosition_Speed() { setposition_speed(); }
void setposition_speed()
{
    CCommand_t aCSecond[256];
    int nSecond = 0;
    while (true)
    {
        sync_clear();
        int nSize = m_nCommand;
        CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : NULL);
        if (nSecond > 0)
        {
            nSize = nSecond;
            pCCmd2 = aCSecond;
            nSecond = 0;
        }
        CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nSize);
        memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nSize);

        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Position_Speed_Address != m_aCParam[pCCmd[i].nID].m_nSet_Position_Speed_Address)
                {
                    aCSecond[nSecond].nID = pCCmd[i].nID;
                    aCSecond[nSecond].fVal = pCCmd[i].fVal;
                    nSecond++;
                }
                else
                {
                    sync_push_dword(pCCmd[i].nID, (int)Roundf(pCCmd[i].fVal));
                }
            }
            sync_flush(m_aCParam[pCCmd[0].nID].m_nSet_Position_Speed_Address);
        }
        free(pCCmd);
        if (nSecond == 0) break;
    }
    command_clear();
}

void CProtocol2::SetPosition() { setposition(); }
void setposition()
{
    CCommand_t aCSecond[256];
    int nSecond = 0;
    while (true)
    {
        sync_clear();
        int nSize = m_nCommand;
        CCommand_t *pCCmd2 = ((m_nCommand > 0) ? m_aCCommand : NULL);
        if (nSecond > 0)
        {
            nSize = nSecond;
            pCCmd2 = aCSecond;
            nSecond = 0;
        }
        CCommand_t *pCCmd = (CCommand_t *)malloc(sizeof(CCommand_t) * nSize);
        memcpy(pCCmd, pCCmd2, sizeof(CCommand_t) * nSize);
        
        if (pCCmd == NULL) break;
        if (nSize > 0)
        {
            for (int i = 0; i < nSize; i++)
            {
                if (m_aCParam[pCCmd[0].nID].m_nSet_Position_Address != m_aCParam[pCCmd[i].nID].m_nSet_Position_Address)
                {
                    aCSecond[nSecond].nID = pCCmd[i].nID;
                    aCSecond[nSecond].fVal = pCCmd[i].fVal;
                    nSecond++;
                }
                else
                {
                    m_afMot[pCCmd[i].nID] = pCCmd[i].fVal;
                    m_afMot_Pose[pCCmd[i].nID] = pCCmd[i].fVal;
                    sync_push_dword(pCCmd[i].nID, calc_angle2evd(pCCmd[i].nID, pCCmd[i].fVal));
                }
            }
            sync_flush(m_aCParam[pCCmd[0].nID].m_nSet_Position_Address);
        }
        free(pCCmd);
        if (nSecond == 0) break;
    }
    command_clear();
}
////////////////////////////////////////////////////// => Command
#endif

bool CProtocol2::Play(const char *strMotionFile, bool bOneshot_Style)
{
    if (IsOpen() == false) return false;
    printf("Play - %s\r\n", strMotionFile);

    bool bFileOpened = false;
	FILE *pfileMotion;
	char buff[1024];
	char szFilename[256];
	sprintf(szFilename, "%s", strMotionFile);
	if ((pfileMotion = fopen(szFilename, "rb")) == NULL)
	{
		printf("File[%s] open error", szFilename);
		return false;
	}
	bFileOpened = true;

    int nData = 0;
    int nSize_Line = 0;
    int nTestIndex = 0;

    // 파일의 크기를 ISize 에 저장한다.
	fseek(pfileMotion, 0, SEEK_END);
	long lSize = ftell(pfileMotion);
	rewind(pfileMotion);
	
    int nCnt_Frame = 0;
    while (feof(pfileMotion) == 0)    // 파일의 끝이면 1 반환
	{
        fgets(buff, 1024, pfileMotion);
        
        nSize_Line = strlen(buff);
        printf("nSize_Line = %d\r\n", nSize_Line);
        lSize -= nSize_Line;
        if (nSize_Line > 0)
        {
            
            if (bOneshot_Style)
            {
                if (PlayFrameString(buff, true) == true)
                {
                    Wait();
                    nCnt_Frame++;
                }
            }
            else
            {
                if (PlayFrameString(buff) == true)
                {
                    nCnt_Frame++;
                    printf("[%d]%s\r\n", nCnt_Frame, buff);
                }
            }
        }
        if (lSize <= 2)//2)
        {
            break;
        } 
        memset(buff, 0, nSize_Line);
	}
    printf("Done : Frame=%d\r\n", nCnt_Frame);

	fclose(pfileMotion);    // 파일 포인터 닫기
    return bFileOpened;
}

bool CProtocol2::PlayFrameString(const char *buff) { return PlayFrameString(buff, false); }
bool CProtocol2::PlayFrameString(const char *buff, bool bNoWait) { return play_frame_string(buff, bNoWait); }
bool play_frame_string(const char *buff, bool bNoWait)
{
    int nTime = 0;
    int nDelay = 0;
    int anDatas[258];
    int nCol = 0;
    char strSplit0[] = ",";
    char strSplit1[] = ":";
    char strBuff[256];
    sprintf(strBuff, buff);
    char *ptr = (char *)strtok(((nCol == 0)?strBuff : NULL), strSplit0);
    int nLen = strlen(ptr);
    bool bEn = false;
    if (nLen > 1)
    {
        if ((strBuff[1] == '1') || (strBuff[1] == '2')) // Enable
        {
            bool bAngle = false;
            if (strBuff[1] == '2') bAngle = true;
            bEn = true;
            int nTime = 0;
            int nDelay = 0;
            command_clear();
            while(ptr=strtok(NULL, strSplit0))
            {
                bool bEnd = false;
                
                if ((ptr[0] == '\r') || (ptr[0] == '\n')) 
                {
                    bEnd = true;
                    break;
                }
                else if (nLen > 1)
                {
                    if ((ptr[1] == '\r') || (ptr[1] == '\n')) 
                    {
                        bEnd = true;
                        break;
                    }
                }
            
                if (bEnd == false) 
                {
                    if (nCol == 0)
                    {
                        nTime = atoi(ptr);
                    }
                    else if (nCol == 1)
                    {
                        nDelay = atoi(ptr);
                    }
                    else
                    {
                        int nIndex = (int)(strchr(ptr, ':') - ptr);
                        if (nIndex > 0)
                        {
                            nLen = strlen(ptr);
                            char *pcID = (char *)malloc(sizeof(char) * (nIndex + 1));
                            memset(pcID, 0, sizeof(char) * (nIndex + 1));
                            memcpy(pcID, ptr, nIndex);
                            int nLenEvd = nLen - nIndex - 1;
                            char *pcEvd = (char *)malloc(sizeof(char) * nLenEvd);
                            memset(pcEvd, 0, sizeof(char) * (nLenEvd + 1));
                            memcpy(pcEvd, ptr + nIndex + 1, nLenEvd);
                     
                            int nID = atoi(pcID);
                            if (bAngle) 
                            {
                                command_set(nID, atof(pcEvd));
                            }
                            else command_set(nID, calc_evd2angle(nID, atoi(pcEvd)));


                            free(pcID);
                            free(pcEvd);
                        }
                    }
                }
                nCol++;
            }
            if (bEn == true) 
            {
                if (bNoWait == false) move(nTime, nDelay);
                else 
                {
                    move_no_wait(nTime, nDelay);
                }
            }

            nCol = 0;
            return true;
        }
    }
    return false;
}


////////////////////////////////////////////////////////////////
int m_nSocket_Mode = 0; // 0 : Normal Mode, 1 : Bypass Mode
void CProtocol2::Socket_BypassMode(bool bBypass) { socket_bypass_mode(bBypass); }
bool CProtocol2::IsSocket_BypassMode() { return is_socket_bypass_mode(); }
bool CProtocol2::Open_Socket(int nPort)//const char * pcIp)//, int nPort)
{
	return socket_open(nPort);
}

void socket_bypass_mode(bool bBypass) 
{
    m_nSocket_Mode = (bBypass) ? 1 : 0;
}
bool is_socket_bypass_mode() { return (m_nSocket_Mode == 1) ? true : false; }
bool socket_open(int nPort)//const char * pcIp)//, int nPort)
{
	// Thread
    if (nPort > 0) m_nSocketPort = nPort;
	int nRet = pthread_create(&m_thSocket, NULL, Thread_Socket, NULL);
	if (nRet != 0)
	{
		printf("[Protocol2]Thread(Socket) Init Error\r\n");
		return false;
	}
	printf("[Protocol2]Init Thread(Socket)\r\n");	
    return true;
}

void CProtocol2::Close_Socket()
{
	sock_close();
}

bool CProtocol2::IsOpen_Socket()
{
	return ((m_nClientMotionFd >= 0) ? true : false);
}

void sock_close()
{
    printf("[Protocol2]Close Socket\n");
	close(m_nClientMotionFd);
	m_nClientMotionFd = -1;
}

#define _SIZE_QUE 3
#define _SIZE_QUE_LENGTH 100
int m_nQue_Index_Next = 0;
int m_nQue_Index = 0;
int m_nQue_Count = 0;
unsigned char m_abyteQue[_SIZE_QUE][_SIZE_QUE_LENGTH];
void* Thread_Socket(void* arg)
{
	// Socket	
	int nSize;
	unsigned char buf[256];
	int nSockFd;
	int nPort = m_nSocketPort;
	struct sockaddr_in SServerAddr, SClientAddr;
	printf("[Protocol2]Server Started, nPort=%d\n", nPort);
	nSockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (nSockFd < 0)
	{
		printf("[Protocol2]Socket Error\n");
		exit(-1);
	}
	
    memset(&SServerAddr, 0, sizeof(SServerAddr));
	SServerAddr.sin_family = AF_INET;
	SServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	SServerAddr.sin_port = htons(nPort);

	int nLength_Client;
	if (bind(nSockFd, (struct sockaddr *)&SServerAddr, sizeof(SServerAddr)) < 0)
    {
		printf("Socket Bind() Error\r\n");
        exit(-1);
    }
	
    // 연결 대기열 1개 생성
    if (listen(nSockFd, 1) < 0)
    {
        printf("Socket listen() Error\r\n");
		exit(-1);
    }
	while(m_bProgEnd == false)
	{
        printf("[Protocol2]Wait Client Connection\n");	
		
        nLength_Client = sizeof(m_nClientMotionFd);    // connection socket
		m_nClientMotionFd = accept(nSockFd, (struct sockaddr *)&SClientAddr, (socklen_t*)&nLength_Client);
        printf("m_nClientMotionFd=%d\r\n", m_nClientMotionFd);
		if (m_nClientMotionFd < 0)
		{
			printf("[Protocol2]Client Connection Error\n");
			break;
		}
		printf("[Protocol2]Client Connected\r\n");

		while ((m_nClientMotionFd >= 0) && (m_bProgEnd == false))
		{	
			if ((nSize = read(m_nClientMotionFd, buf, _SIZE_BUFFER)) < 0)
			{		
				printf("[Protocol2]Receive Error\n");
				sock_close();
				break;
			}
			if (nSize <= 0)
			{
				printf("[Protocol2]Disconnected\n");
				sock_close();
				break;
			}			
			else
			{
                if (is_socket_bypass_mode() == true)	
				    send_packet(buf, nSize);
                else
                {
                    bool bStart = false;
                    bool bContinue = false;
                    //int nSize = 0;
                    int nSize2 = m_abyteQue[m_nQue_Index_Next][0] + m_abyteQue[m_nQue_Index_Next][1] * 256;
                    if (nSize2 < 0) nSize2 = 0;
                    else if (nSize2 > _SIZE_QUE_LENGTH) nSize2 = _SIZE_QUE_LENGTH;
                    for (int i = 0; i < nSize; i++)
                    {
                        if (buf[i] == 0x02)
                        {
                            bStart = true;
                            nSize2 = 0;
                            bContinue = false;
                        }
                        else if (buf[i] == 0x03)
                        {
                            bStart = false;
                            bContinue = true;
                            m_abyteQue[m_nQue_Index_Next][0] = (nSize2 & 0xff);
                            m_abyteQue[m_nQue_Index_Next][1] = ((nSize2 >> 8) & 0xff);
                            m_nQue_Index = m_nQue_Index_Next;
                            m_nQue_Index_Next++;
                            m_nQue_Count++;
                        }
                        else
                        {
                            m_abyteQue[m_nQue_Index_Next][2 + nSize2++] = buf[i];
                        }
                    }
                    if (m_nQue_Index_Next >= _SIZE_QUE)
                    {
                        m_nQue_Index_Next = 0;
                    }
                    if (m_nQue_Count > _SIZE_QUE)
                    {
                        m_nQue_Count = _SIZE_QUE;
                    }
                }
			}
            if (m_nQue_Count > 0)
            {
                int nLen = m_abyteQue[m_nQue_Index][0] + m_abyteQue[m_nQue_Index][1] * 256;
                    
                char str[nLen + 1];
                memset(str, 0, sizeof(char) * (nLen + 1));
                memcpy(str, &m_abyteQue[m_nQue_Index][2], sizeof(char) * nLen);
                
                play_frame_string(str, true);
                m_nQue_Count--;
            }
			usleep(0);
		}
		usleep(0);
	}
	printf("[Protocol2][Thread] Closed Thread\r\n");
    return (void *)NULL;
}

