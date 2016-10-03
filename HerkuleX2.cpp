//#define _TEST_RECEIVE
/**
 * HerkuleX2
 * A library for DST HerkuleX Servo
 *
 * Copyright 2016 DST Robot
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; 
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc..
 * 
 * @author      Jinwook On ojw5014@hanmail.net
 * @modified    2016.06.23
 * @version     01.00.00 Released
 */
 
#include <iostream>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>

#include "HerkuleX2.h"

// Jinwook, On
#include "stdlib.h"
#include <sys/timeb.h>
#include "math.h"
#include <stdarg.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// add : 20160621 Jinwook On in DST Robot
#define Round(dValue)				(((double)dValue > 0)?floor((double)dValue + 0.5):ceil((double)dValue - 0.5))
#define Roundf(fValue)				(((float)fValue > 0)?floor((float)fValue + 0.5f):ceil((float)fValue - 0.5f))

#define _TIME_DELAY	0//10
#define _TIME_DELAY_THREAD	0

#define _CNT_RETRIEVE	5
int m_nRetrieve = 0;

bool m_bProgEnd;
char m_acRam[_SIZE_MOTOR_MAX][_SIZE_MEMORY];
char m_acRom[_SIZE_MOTOR_MAX][_SIZE_MEMORY];

char m_anPos[_SIZE_MOTOR_MAX];
char m_acStatus1[_SIZE_MOTOR_MAX];
char m_acStatus2[_SIZE_MOTOR_MAX];
int m_anAxis_By_ID[_SIZE_MOTOR_MAX];

bool m_bMultiTurn = false;

int m_nSeq_Receive = 0;	
int m_nSeq_Receive_Back = 0;

int GetAxis_By_ID(int nID) { return (nID == 0xfe) ? 0xfe : m_anAxis_By_ID[nID]; }
int m_nTty;	

#define _SIZE_BUFFER 1024
int m_nClientMotionFd;
pthread_t m_thSocket;

bool m_abReceivedPos[_SIZE_MOTOR_MAX];

bool m_bShowMessage;
#ifdef _TEST_RECEIVE
bool m_bError = false;
#endif
void CMotor::Sync_Seq() { m_nSeq_Receive_Back= m_nSeq_Receive; }
void CMotor::Init()
{
	m_bShowMessage = false;
#ifdef _TEST_RECEIVE
	m_nTimeout = 5000;	
#else
	m_nTimeout = 10;	
#endif
	//m_nSeq_Motor = 0;
	//m_nSeq_Motor_Back = 0;
	m_ulDelay = 0;

	
	m_bIgnoredLimit = false;
	m_nTty = 0;				//file description
	m_nModel = 0;
	m_thReceive = 0;
	m_thSocket = 0;

	m_nClientMotionFd = -1;
		
	m_nSeq_Receive = 0;

	//m_bOpen = false;
	m_bStop = false;
	m_bEms = false;

	m_nMotorCnt_Back = m_nMotorCnt = 0;
	memset(m_acRam, 0, sizeof(char) * _SIZE_MEMORY);
	memset(m_acRom, 0, sizeof(char) * _SIZE_MEMORY);
	memset(m_aSParam_Axis, 0, sizeof(SParam_Axis_t) * _SIZE_MOTOR_MAX);
	memset(m_aSMot, 0, sizeof(SMot_t) * _SIZE_MOTOR_MAX);
	memset(m_aSMot_Prev, 0, sizeof(SMot_t) * _SIZE_MOTOR_MAX);

	memset(m_acStatus1, 0, _SIZE_MOTOR_MAX);
	memset(m_acStatus2, 0, _SIZE_MOTOR_MAX);
	memset(m_anAxis_By_ID, 0, _SIZE_MOTOR_MAX);
	
	memset(m_acEn, 0, _SIZE_MOTOR_MAX);
	memset(m_aSRead, 0, sizeof(SRead_t) * _SIZE_MOTOR_MAX);
	m_nReadCnt = 0;

	//m_nSeq_Receive = 0;
	//m_nSeq_Receive_Back = 0;
	Sync_Seq();

	memset(m_abReceivedPos, 0, sizeof(bool) * _SIZE_MOTOR_MAX);
	

	for (int i = 0; i < 256; i++) SetParam(i, _MODEL_DRS_0101);
	m_bProgEnd = false;	
}
CMotor::CMotor()									// Initialize
{
	Init();
}
CMotor::~CMotor()		 							// Destroy
{
	m_bProgEnd = true;
	usleep(100);				
	if (IsOpen() == true) Close();
	if (IsOpen_Socket() == true) Close_Socket();
}

void* CMotor::Thread_Socket(void* arg)
{
	// Socket	
	int nSize;
	byte buf[256];
	int nSockFd;
	int nPort = 5002;//m_nPort;//5000;
	struct sockaddr_in SServerAddr, SClientAddr;
	printf("[HerkuleX2]Server Started\n");
	//nSockFd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	nSockFd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (nSockFd < 0)
	{
		printf("[HerkuleX2]Socket Error\n");
		exit(-1);
	}
	
	bzero(&SServerAddr, sizeof(SServerAddr));
	SServerAddr.sin_family = AF_INET;
	SServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	SServerAddr.sin_port = htons(nPort);

	int nLength_Client;
	bind(nSockFd, (struct sockaddr *)&SServerAddr, sizeof(SServerAddr));
	listen(nSockFd, 5); // 연결 대기
	
	while(m_bProgEnd == false)
	{
		printf("[HerkuleX2]Wait Client Connection\n");	
		
		m_nClientMotionFd = accept(nSockFd, (struct sockaddr *)&SClientAddr, (socklen_t*)&nLength_Client);

		if (m_nClientMotionFd < 0)
		{
			printf("[HerkuleX2]Client Connection Error\n");
			//m_bProgEnd = true;
			break;
			//return;// -1;
		}
		printf("[HerkuleX2]Client Connected\r\n");// %s\n", inet_ntoa(SClientAddr.sin_addr));

		while ((m_nClientMotionFd >= 0) && (m_bProgEnd == false))
		{	
		//printf("test\n");
			if ((nSize = read(m_nClientMotionFd, buf, _SIZE_BUFFER)) < 0)
			{		
				printf("[HerkuleX2]Receive Error\n");
				//close(m_nClientMotionFd);
				//m_nClientMotionFd = -1;
				Close_Socket();
				break;
			}
			//printf("Test=%d, nSize=%d\r\n", m_nClientMotionFd, nSize);
			//usleep(1000);
			if (nSize <= 0)
			{
				printf("[HerkuleX2]Disconnected\n");
				Close_Socket();
				//printf("[HerkuleX2]Close Socket\n");
				//close(m_nClientMotionFd);
				//m_nClientMotionFd = -1;
				break;
			}			
			else //if (nSize > 0)
			{				
				//printf("[HerkuleX2][Receive]\n");
				//SendPacket_Socket(buf, nSize);
				SendPacket(buf, nSize);
			}
			usleep(0);
		}
		usleep(0);
	}	
	printf("[HerkuleX2][Thread] Closed Thread\r\n");
}
void* CMotor::Thread_Receive(void* arg)
{
	int nIndex = 0;
	char buf[256];
	byte byCheckSum = 0;
	byte byCheckSum1 = 0;
	byte byCheckSum2 = 0;
	int nPacketSize = 0;
	int nDataLength = 0;
	int nIndexData = 0;
	int nCmd = 0;
	int nId = 0;
	int nData_Address = 0;
	int nData_Length = 0;
	printf("[Thread] Running Thread\r\n");
	while(m_bProgEnd == false)
	{
		int nSize = read(m_nTty, buf, 256);
		if (nSize > 0)
		{
#ifdef _TEST_RECEIVE
			if (m_bError == true) printf("Receive Event - after error\r\n");
#endif		
			//printf("[Receive]");
			if (m_nClientMotionFd >= 0)
			{
				
			}
			for (int i = 0; i < nSize; i++)
			{
				//printf("[Index=%d]", nIndex);
				//printf("0x%02X,", buf[i]);
				
				switch(nIndex)
				{
					case 0 : 
						if (((m_bMultiTurn == true) && ((nIndex == 0) && (buf[i] == 0xff))) || ((m_bMultiTurn == false) && (buf[i] == 0xff)))
						{
							byCheckSum = 0;
							nIndexData = 0;
							
							nData_Address = 0;
							nData_Length = 0;
							
							nIndex++;
						}
						break;
					case 1 :
						if (buf[i] == 0xff) nIndex++;
						else nIndex = 0;//-1;
						break;
					case 2 : // Packet Size
						nPacketSize = buf[i];
						nDataLength = nPacketSize - _SIZE_PACKET_HEADER - 2;
						byCheckSum = buf[i];
						if (nDataLength < 0) 	nIndex = 0;
						else 				nIndex++;
						break;
					case 3 : // ID
						nId = GetAxis_By_ID(buf[i]);
						byCheckSum ^= buf[i];
						nIndex++;
						break;
					case 4 : // Cmd
						nCmd = buf[i];
						byCheckSum ^= buf[i];
						nIndex++;
						break;
					case 5 : // CheckSum1
						byCheckSum1 = buf[i];
						nIndex++;
						break;
					case 6 : // CheckSum2
						byCheckSum2 = buf[i];
						if ((~byCheckSum1 & 0xfe) != byCheckSum2) nIndex = 0;
						else nIndex++;
						break;
					case 7 : // Datas...
						//printf("[DataLength=%d/%d]", nIndexData, nDataLength);
						if (nIndexData < nDataLength)
						{
							if (nIndexData == 0) 		nData_Address = buf[i];
							else if (nIndexData == 1) 	nData_Length =  buf[i];
							else 					m_acRam[nId][nData_Address + nIndexData - 2] = buf[i];




							if (++nIndexData >= nDataLength) nIndex++;
													
							byCheckSum ^= buf[i];	
						}						
						else
						{
							//printf("====== Status1=======\r\n");
							m_acStatus1[nId] = buf[i];
							byCheckSum ^= buf[i];
							nIndex+=2;
						}
						break;
					case 8 : // Status 1		
						//printf("====== Status1=======\r\n");
						m_acStatus1[nId] = buf[i];
						byCheckSum ^= buf[i];
						nIndex++;


/*
						0x01 : Exceed Input Voltage Limit
						0x02 : Exceed allowed POT limit
						0x04 : Exceed Temperature limit
						0x08 : Invalid Packet
						0x10 : Overload detected
						0x20 : Driver fault detected
						0x40 : EEP REG distorted
						0x80 : reserved
*/
						
						break;
					case 9 : // Status 2		
						//printf("====== Status2=======(Chk: 0x%02X , 0x%02X\r\n", byCheckSum, byCheckSum1);
						m_acStatus2[nId] = buf[i];
						byCheckSum ^= buf[i];
						
/*
						0x01 : Moving flag
						0x02 : Inposition flag
						0x04 : Checksum Error
						0x08 : Unknown Command
						0x10 : Exceed REG range
						0x20 : Garbage detected
						0x40 : MOTOR_ON flag
						0x80 : reserved
*/
						







						///////////////////
						// Done
						if ((byCheckSum & 0xFE) == byCheckSum1) 
						{
							// test
							byte abyteData[2];
							abyteData[1] = (byte)(m_acRam[nId][ _ADDRESS_CALIBRATED_POSITION + ((m_bMultiTurn == true) ? 4 : 0)] & 0xff);
							abyteData[0] = (byte)(m_acRam[nId][ _ADDRESS_CALIBRATED_POSITION + ((m_bMultiTurn == true) ? 4 : 0) + 1] & 0xff);
							// 0000 0000  0000 0000
							m_anPos[nId] = (short)(((abyteData[0] & 0x0f) << 8) | (abyteData[1] << 0) | ((abyteData[0] & 0x10) << (3 + 8)) | ((abyteData[0] & 0x10) << (2 + 8)) | ((abyteData[0] & 0x10) << (1 + 8)));
                					if (m_bShowMessage == true) printf("Data Received(<Address(%d)Length(%d)>Pos[%d]=%d, Status1 = %d, Status2 = %d)\r\n", nData_Address, nDataLength, nId, m_anPos[nId], m_acStatus1[nId], m_acStatus2[nId]);

#ifdef _TEST_RECEIVE
							if (m_bError == true) printf("*****Received(after error)*****\r\n");
							m_bError = false;
#endif
							m_abReceivedPos[nId] = true;
							m_nSeq_Receive++;
						}
						
#ifdef _TEST_RECEIVE
						else if (m_bError == true) printf("checksum error - after error\r\n");
#endif		
						nIndex = 0;
						break;
				}

#ifdef _TEST_RECEIVE
				if (m_bError == true) printf("Index [%d] - after error\r\n", nIndex);
#endif		
			}		
			//printf("\r\n");
		}
		usleep(_TIME_DELAY_THREAD);
	}
	
	printf("[Thread] Closed Thread\r\n");
}

void CMotor::Open_Socket()//const char * pcIp)//, int nPort)
{
	// Thread
	int nRet = pthread_create(&m_thSocket, NULL, Thread_Socket, NULL);
	if (nRet != 0)
	{
		printf("[HerkuleX2]Thread(Socket) Init Error\r\n");
		return;// -1;
	}
	printf("[HerkuleX2]Init Thread(Socket)\r\n");	
}

void CMotor::Close_Socket()
{
	printf("[HerkuleX2]Close Socket\n");
	close(m_nClientMotionFd);
	m_nClientMotionFd = -1;
}

bool CMotor::IsOpen_Socket()
{
	return ((m_nClientMotionFd >= 0) ? true : false);
}
	
void CMotor::Open(const char  *pcDevice, int nBaudrate)//, int nModel)		//serial port open
{
	if (IsOpen() == false)
	{
		// Port Open		
		struct termios newtio;

		m_nTty = open(pcDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if(m_nTty == -1) {
			printf("Failed to open TTY Serial Device\r\n");
		    return;
		}
		printf("Init Serial[%s]\r\n", pcDevice);
#if 1
		// Thread
		int nRet = pthread_create(&m_thReceive, NULL, Thread_Receive, NULL);
		if (nRet != 0)
		{
			printf("Thread Init Error\r\n");
			return;
		}
		printf("Init Thread\r\n");
#endif
		memset( &newtio, 0, sizeof(newtio) );

#if 1
		newtio.c_cflag = ((nBaudrate == 9600) ? B9600 : ((nBaudrate == 19200) ? B19200 : ((nBaudrate == 38400) ? B38400 : ((nBaudrate == 57600) ? B57600 : ((nBaudrate == 115200) ? B115200 : B115200)))));//B115200;
		newtio.c_cflag |= CS8;
		newtio.c_cflag |= CLOCAL;
		newtio.c_cflag |= CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0;
		newtio.c_cc[VMIN]  = 0;

		tcflush (m_nTty, TCIFLUSH );			//reset modem
		tcsetattr(m_nTty, TCSANOW, &newtio );	//save setting
#else
		newtio.c_cflag = ((nBaudrate == 9600) ? B9600 : ((nBaudrate == 19200) ? B19200 : ((nBaudrate == 38400) ? B38400 : ((nBaudrate == 57600) ? B57600 : ((nBaudrate == 115200) ? B115200 : B115200)))));//B115200;
//printf("Baudrate=%d, %d\r\n", newtio.c_cflag, B115200);
		newtio.c_cflag |= CS8;
		newtio.c_cflag |= CLOCAL;
		newtio.c_cflag |= CREAD;
		//newtio.c_iflag |= IGNPAR;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;

		newtio.c_cc[VINTR] = 0; /*ctrl-c */
		newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
		newtio.c_cc[VERASE] = 0; /* del */
		newtio.c_cc[VKILL] = 0; // @
		newtio.c_cc[VEOF] = 4; // ctrl-d
		newtio.c_cc[VTIME] = 0; // inter-character timer unused
		newtio.c_cc[VMIN] = 1;// blocking read until 1 character arrives
		newtio.c_cc[VSWTC] = 0; // '\0'
		newtio.c_cc[VSTART] = 0; // Ctrl-q
		newtio.c_cc[VSTOP] = 0; // Ctrl-s
		newtio.c_cc[VSUSP] = 0; // Ctrl-z
		newtio.c_cc[VEOL] = 0; //'\0'
		newtio.c_cc[VREPRINT] = 0; // ctrl-r
		newtio.c_cc[VDISCARD] = 0; // Ctrl-u
		newtio.c_cc[VWERASE] = 0; // Ctrl-w
		newtio.c_cc[VLNEXT] = 0; // Ctrl-v
		newtio.c_cc[VEOL2] = 0; // '\0'
		tcflush(m_nTty, TCIFLUSH);
		tcsetattr(m_nTty, TCSANOW, &newtio);
#endif
		//std::cout << "Serial Open! " << m_nTty << std::endl;

		Clear_Flag();

	}

	if (IsOpen() == false)
	{
		//printf("[Open()][Error] Connection Fail - Comport %s, %d, %s\r\n", pcDevice, nBaudrate, pszModel);
		printf("[Open()][Error] Connection Fail - Comport %s, %d, %s\r\n", pcDevice, nBaudrate);

		//free(pszModel);
		return;// false;
	}
	//m_nModel = nModel;
	
	//free(pszModel);
	//return true;
}
void CMotor::Close()								//serial port close
{
	if (IsOpen() == true) 
	{
		close(m_nTty);
		printf("Serial Port -> Closed\r\n");
	}

	m_nTty = 0;
//	m_nModel = 0;
}

//void CMotor::SetParam(int nId)
//{
//}
void CMotor::SetParam(int nAxis, int nRealID, int nDir, float fLimitUp, float fLimitDn, float fCenterPos, float fOffsetAngle_Display, float fMechMove, float fDegree)
{
    //if ((nAxis >= _CNT_MAX_MOTOR) || (nID >= _MOTOR_MAX)) return false;

    m_aSParam_Axis[nAxis].nID = m_aSMot[nAxis].nID = nRealID;
    m_aSParam_Axis[nAxis].nDir = m_aSMot[nAxis].nDir = nDir;
    m_aSParam_Axis[nAxis].fLimitUp = m_aSMot[nAxis].fLimitUp = fLimitUp;
    m_aSParam_Axis[nAxis].fLimitDn = m_aSMot[nAxis].fLimitDn = fLimitDn;
    m_aSParam_Axis[nAxis].fCenterPos = m_aSMot[nAxis].fCenterPos= fCenterPos;
    m_aSParam_Axis[nAxis].fOffsetAngle_Display = fOffsetAngle_Display;
    m_aSParam_Axis[nAxis].fMechMove = m_aSMot[nAxis].fMechMove = fMechMove;
    m_aSParam_Axis[nAxis].fDegree = m_aSMot[nAxis].fDegree = fDegree;
}
void CMotor::SetParam(int nAxis, int nModel)
{
	//printf("nAxis = %d, nModel = %d\r\n", nAxis, nModel);
	if (nModel == _MODEL_DRS_0603) m_bMultiTurn = true;
	else if (m_bMultiTurn == true) m_bMultiTurn = false;
	switch(nModel)
	{
		case _MODEL_DRS_0101: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 512);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 1024);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0102: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 3196);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 6392);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0201: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 512);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 1024);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0202: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 3196);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 6392);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0401: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 1024);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 2048);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0402: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 16384);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0601: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 1024);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 2048);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0602: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 16384);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
		case _MODEL_DRS_0603: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 0);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
			break;
	}
}

void CMotor::SetParam_RealID(int nAxis, int nRealID) { m_aSParam_Axis[nAxis].nID = m_aSMot[nAxis].nID = nRealID; m_anAxis_By_ID[nRealID] = nAxis; }
void CMotor::SetParam_Dir(int nAxis, int nDir) { m_aSParam_Axis[nAxis].nDir = m_aSMot[nAxis].nDir = nDir; }
void CMotor::SetParam_LimitUp(int nAxis, float fLimitUp) { m_aSParam_Axis[nAxis].fLimitUp = m_aSMot[nAxis].fLimitUp = fLimitUp; }
void CMotor::SetParam_LimitDown(int nAxis, float fLimitDn) { m_aSParam_Axis[nAxis].fLimitDn = m_aSMot[nAxis].fLimitDn = fLimitDn; }
void CMotor::SetParam_CenterEvdValue(int nAxis, float fCenterPos) { m_aSParam_Axis[nAxis].fCenterPos = m_aSMot[nAxis].fCenterPos = fCenterPos; }
void CMotor::SetParam_Display(int nAxis, float fOffsetAngle_Display) { m_aSParam_Axis[nAxis].fOffsetAngle_Display = fOffsetAngle_Display; }
void CMotor::SetParam_MechMove(int nAxis, float fMechMove) { m_aSParam_Axis[nAxis].fMechMove = m_aSMot[nAxis].fMechMove = fMechMove; }
void CMotor::SetParam_Degree(int nAxis, float fDegree) { m_aSParam_Axis[nAxis].fDegree = m_aSMot[nAxis].fDegree = fDegree; }

void CMotor::Stop(int nAxis) // no stop flag setting
{
//	if (Get_Flag_Mode(nAxis) != 0)   // 속도제어
	Set_Turn(nAxis, 0);
//	Set_Flag_Stop(nAxis, true);
	Send_Motor(1000);
}
void CMotor::Stop()
{
//    for (int i = 0; i < _SIZE_MOTOR_MAX; i++) 
//    {
//	 if (Get_Flag_Mode(i) != 0)   // 속도제어
//        	Set(i, 0);
//	Set_Turn(nAxis, 0);
 //       Set_Flag_Stop(i, true);
//    }
    Set_Turn(254, 0);
    Send_Motor(100);
    m_bStop = true;
}	
void CMotor::Ems()
{
    Stop();
    SetTorque(false, false);
    m_bEms = true;
}

bool CMotor::GetErrorCode(int nAxis) { return m_acStatus1[nAxis]; }
bool CMotor::IsError(int nAxis)
{
	if (m_acStatus1[nAxis] != 0)
	{
		return true;		
	}
	return false;
/*
	0x01 : Exceed Input Voltage Limit
	0x02 : Exceed allowed POT limit
	0x04 : Exceed Temperature limit
	0x08 : Invalid Packet
	0x10 : Overload detected
	0x20 : Driver fault detected
	0x40 : EEP REG distorted
	0x80 : reserved
*/
}

bool CMotor::IsWarning(int nAxis)
{	
	// Status 2
	if ((m_acStatus2[nAxis] & 0x43) != 0)
	{
		return true;
	}
	return false;
	
/*
	0x01 : Moving flag
	0x02 : Inposition flag
	0x04 : Checksum Error
	0x08 : Unknown Command
	0x10 : Exceed REG range
	0x20 : Garbage detected
	0x40 : MOTOR_ON flag
	0x80 : reserved
	*/
}
//////////////////////////////////////////////////////////
// Reboot 
void CMotor::Reboot() { Reboot(_ID_BROADCASTING); }
void CMotor::Reboot(int nAxis)
{
	if (nAxis < 0xfe) Clear_Flag(nAxis);
       else
	{
	    for (int i = 0; i < _SIZE_MOTOR_MAX; i++) Clear_Flag(i);
	}

	int nID = m_aSMot[nAxis].nID;
	int nDefaultSize = _CHECKSUM2 + 1;
	byte pbyteBuffer[nDefaultSize];
	//(char *)pbyteBuffer = (char
	// Header
	pbyteBuffer[_HEADER1] = 0xff;
	pbyteBuffer[_HEADER2] = 0xff;
	// ID = 0xFE : 전체명령, 0xFD - 공장출하시 설정 아이디
	pbyteBuffer[_ID] = (byte)(nID & 0xff);
	// Cmd
	pbyteBuffer[_CMD] = 0x09; // Reset

	//Packet Size
	pbyteBuffer[_SIZE] = (byte)((nDefaultSize) & 0xff);

	MakeCheckSum(nDefaultSize, pbyteBuffer);//, out pbyteBuffer[_CHECKSUM1], out pbyteBuffer[_CHECKSUM2]);

	SendPacket(pbyteBuffer, nDefaultSize);

	Clear_Flag();


// Initialize variable
	m_bStop = false;
	m_bEms = false;
	m_nMotorCnt_Back = m_nMotorCnt = 0;

}

///////////////////////////////////
// Motor Control - Reset
void CMotor::Reset() { Reset(_ID_BROADCASTING); }
void CMotor::Reset(int nAxis) 
{
	// Clear Variable
	m_bStop = false;
	m_bEms = false;
	
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	byte pbyteBuffer[3];
	// Data
	pbyteBuffer[i++] = 48 + ((m_bMultiTurn == true) ? 4 : 0);// 48번 레지스터 명령
	////////
	pbyteBuffer[i++] = 0x01;// 이후의 레지스터 사이즈
	pbyteBuffer[i++] = 0x00; // led value

	Make_And_Send_Packet(nID, 0x03, i, pbyteBuffer);
	//pbyteBuffer = null;
}

//void CMotor::Reset_ErrorFlag()
//{
//	m_bStop = m_bEms = false;
//}
//void CMotor::Reset_ErrorFlag(int nAxis)
//{
//	m_bStop = m_bEms = false;
//}

int CMotor::Clip(int nLimitValue_Up, int nLimitValue_Dn, int nData)
{
    if (GetLimitEn() == false) return nData;

    int nRet = ((nData > nLimitValue_Up) ? nLimitValue_Up : nData);
    return ((nRet < nLimitValue_Dn) ? nLimitValue_Dn : nRet);
}
float CMotor::Clip(float fLimitValue_Up, float fLimitValue_Dn, float fData)
{
    if (GetLimitEn() == false) return fData;
    float fRet = ((fData > fLimitValue_Up) ? fLimitValue_Up : fData);
    return ((fRet < fLimitValue_Dn) ? fLimitValue_Dn : fRet);
}

int CMotor::CalcLimit_Evd(int nAxis, int nValue)
{

    if ((Get_Flag_Mode(nAxis) == 0) || (Get_Flag_Mode(nAxis) == 2))
    {
		//if ((m_aSMot[nAxis].fLimit_Down != 0) && (m_aSMot[nMot].fLimit_Down >= fValue)) fValue = m_aSMot[nMot].fLimit_Down;
		//if ((m_aSMot[nAxis].fLimit_Up != 0) && (m_aSMot[nMot].fLimit_Up <= fValue)) fValue = m_aSMot[nMot].fLimit_Up;
		//return fValue;













        int nPulse = nValue;
	 if (m_bMultiTurn == false)
	 {
	 	nValue &= 0x4000;
        	nValue &= 0x3fff;
	 }
 	 //int nLimit = 0x100000;
        int nUp = 0x100000;
        int nDn = -nUp;
        if (m_aSMot[nAxis].fLimitUp != 0) nUp = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitUp);
        if (m_aSMot[nAxis].fLimitDn != 0) nDn = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitDn);
        if (nUp < nDn) { int nTmp = nUp; nUp = nDn; nDn = nTmp; }
        return (Clip(nUp, nDn, nValue) | nPulse);
    }

//	int nDn = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitDn);
//	int nUp = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitUp);
//	if ((nDn != 0) && (nDn >= nValue)) nValue = nDn;
//	if ((nUp != 0) && (nUp <= nValue)) nValue = nUp;    
    return nValue;
}
float CMotor::CalcLimit_Angle(int nAxis, float fValue)
{
    if ((Get_Flag_Mode(nAxis) == 0) || (Get_Flag_Mode(nAxis) == 2))
    {
		if ((m_aSMot[nAxis].fLimitDn != 0) && (m_aSMot[nAxis].fLimitDn >= fValue)) fValue = m_aSMot[nAxis].fLimitDn;
		if ((m_aSMot[nAxis].fLimitUp != 0) && (m_aSMot[nAxis].fLimitUp <= fValue)) fValue = m_aSMot[nAxis].fLimitUp;
    }
    return fValue;
}

int CMotor::CalcTime_ms(int nTime)
{
    // 1 Tick 당 11.2 ms => 1:11.2=x:nTime => x = nTime / 11.2
    return ((nTime <= 0) ? 1 : (int)Roundf((float)nTime / 11.2f));
}
int CMotor::CalcAngle2Evd(int nAxis, float fValue)
{
	fValue *= ((m_aSMot[nAxis].nDir == 0) ? 1.0f : -1.0f);
	int nData = 0;
	if (Get_Flag_Mode(nAxis) != 0)   // 속도제어
	{
	    nData = (int)Roundf(fValue);
		//printf("Speed Turn");
	}
	else
	{
	    // 위치제어
	    nData = (int)Roundf((m_aSMot[nAxis].fMechMove * fValue) / m_aSMot[nAxis].fDegree);
	    nData = nData + (int)Roundf(m_aSMot[nAxis].fCenterPos);
		
	    //printf("[%d]Angle(%.2f), Mech(%.2f), Degree(%.2f), Center(%.2f), nData(%d)\r\n", nAxis, fValue, m_aSMot[nAxis].fMechMove, m_aSMot[nAxis].fDegree, m_aSMot[nAxis].fCenterPos, nData);
	}

	return nData;
}
float CMotor::CalcEvd2Angle(int nAxis, int nValue)
{
	float fValue = ((m_aSMot[nAxis].nDir == 0) ? 1.0f : -1.0f);
	float fValue2 = 0.0f;
	if (Get_Flag_Mode(nAxis) != 0)   // 속도제어
	    fValue2 = (float)nValue * fValue;
	else                                // 위치제어
	    fValue2 = (float)(((m_aSMot[nAxis].fDegree * ((float)(nValue - (int)Roundf(m_aSMot[nAxis].fCenterPos)))) / m_aSMot[nAxis].fMechMove) * fValue);
	return fValue2;
}


void CMotor::SetBaudrate(int nAxis, int nBaud) 	//torque on / Off
{
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	byte byBaud = 0x10;
	byBaud = ((nBaud == 1000000) ? 0x01 : 
		((nBaud == 666666) ? 0x02 : 
			((nBaud == 500000) ? 0x03 : 
				((nBaud == 400000) ? 0x04 : 
					((nBaud == 250000) ? 0x07 : 
						// baudrate : 0x10(16 - 115200) , 0x22(34 - 57600)
						((nBaud == 200000) ? 0x09 : ((nBaud == 57600) ? 0x22 : 0x10)) // 57600(0x22) : 115200(0x10)
					)
				)
			)
		)
	);
	byte pbyteBuffer[50];
	// Data
	pbyteBuffer[i++] = 4; // 4 번 레지스터 명령
	////////
	pbyteBuffer[i++] = 0x01;// 이후의 레지스터 사이즈
	pbyteBuffer[i++] = byBaud;
	Make_And_Send_Packet(nID, 0x01, i, pbyteBuffer);
	//pbyteBuffer = null;
}

void CMotor::SetMotorID(int nAxis, int nNewID) 	//torque on / Off
{
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	
	byte pbyteBuffer[50];
	// Data
	pbyteBuffer[i++] = 6; // 6 번 레지스터 명령
	////////
	pbyteBuffer[i++] = 0x01;// 이후의 레지스터 사이즈
	pbyteBuffer[i++] = (nNewID & 0xff);
	Make_And_Send_Packet(nID, 0x01, i, pbyteBuffer);
	//pbyteBuffer = null;
}
////////////////////////////////////
// Motor Control - Torq On / Off
void CMotor::SetTorque(int nAxis, bool bDrvOn, bool bSrvOn) 	//torque on / Off
{
#if 1
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	byte byOn = 0;
	byOn |= (byte)((bDrvOn == true) ? 0x40 : 0x00);
	byOn |= (byte)((bSrvOn == true) ? 0x20 : 0x00);
	byte pbyteBuffer[50];
	// Data
	pbyteBuffer[i++] = _ADDRESS_TORQUE_CONTROL + ((m_bMultiTurn == true) ? 4 : 0);// 52번 레지스터 명령
	////////
	pbyteBuffer[i++] = 0x01;// 이후의 레지스터 사이즈
	pbyteBuffer[i++] = byOn;
	Make_And_Send_Packet(nID, 0x03, i, pbyteBuffer);
	//pbyteBuffer = null;
#else

	int nID = m_aSMot[nAxis].nID;
	byte byOn = 0;
	byOn |= (byte)((bDrvOn == true) ? 0x40 : 0x00);
	byOn |= (byte)((bSrvOn == true) ? 0x20 : 0x00);

	
	int nDefaultSize = _CHECKSUM2 + 1;
	byte pbyteBuffer[nDefaultSize];
	//(char *)pbyteBuffer = (char
	// Header
	pbyteBuffer[_HEADER1] = 0xff;
	pbyteBuffer[_HEADER2] = 0xff;
	// ID = 0xFE : 전체명령, 0xFD - 공장출하시 설정 아이디
	pbyteBuffer[_ID] = (byte)(nID & 0xff);
	// Cmd
	pbyteBuffer[_CMD] = 0x03; 
	////////
	int i = 0;
	// Data
	pbyteBuffer[nDefaultSize + i++] = _ADDRESS_TORQUE_CONTROL + ((m_bMultiTurn == true) ? 4 : 0);// 52번 레지스터 명령
	////////
	pbyteBuffer[nDefaultSize + i++] = 0x01;// 이후의 레지스터 사이즈
	pbyteBuffer[nDefaultSize + i++] = byOn;
	
	//Packet Size
	pbyteBuffer[_SIZE] = (byte)((nDefaultSize + i) & 0xff);
	MakeCheckSum(nDefaultSize + i, pbyteBuffer);//, out pbyteBuffer[_CHECKSUM1], out pbyteBuffer[_CHECKSUM2]);
#if 0
	printf("[SetTorque(),Value=%d]\r\n", byOn);
	for (int nMsg = 0; nMsg < nDefaultSize + i; nMsg++)
	{
		printf("0x%02x, ", pbyteBuffer[nMsg]);
	}
	printf("\r\n");
#endif
	SendPacket(pbyteBuffer, nDefaultSize + i);
#endif

	
}
void CMotor::SetTorque(bool bDrvOn, bool bSrvOn) { SetTorque((int)_ID_BROADCASTING, bDrvOn, bSrvOn); }

/////////////////////////////////////
// Data Command(No motion) - just setting datas   		=> use with Send_Motor
// ---- Position Control ----
void CMotor::Set(int nAxis, int nEvd)
{
	if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return;
	Push_Id(nAxis);
	Read_Motor_Push(nAxis);
	m_aSMot[nAxis].bEn = true; 
	Set_Flag_Mode(nAxis, false);
	m_aSMot[nAxis].nPos = CalcLimit_Evd(nAxis, nEvd); 
	Set_Flag_NoAction(nAxis, false);
	//Push_Id(nAxis);	
}
int CMotor::Get(int nAxis) { return m_aSMot[nAxis].nPos; }
            
void CMotor::Set_Angle(int nAxis, float fAngle)
{
	if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return;
	Push_Id(nAxis);
	Read_Motor_Push(nAxis);

	m_aSMot_Prev[nAxis].nFlag = m_aSMot[nAxis].nFlag;
	m_aSMot_Prev[nAxis].bEn = m_aSMot[nAxis].bEn;
	m_aSMot_Prev[nAxis].nPos = m_aSMot[nAxis].nPos;
	//m_aSMot_Prev[nAxis].fMechMove = m_aSMot[nAxis].fMechMove;
	//m_aSMot_Prev[nAxis].fDegree = m_aSMot[nAxis].fDegree;

	m_aSMot[nAxis].bEn = true; 
	Set_Flag_Mode(nAxis, false);
	m_aSMot[nAxis].nPos = CalcLimit_Evd(nAxis, CalcAngle2Evd(nAxis, fAngle)); 
	Set_Flag_NoAction(nAxis, false);
	//Push_Id(nAxis);	
}
float CMotor::Get_Angle(int nAxis) { return CalcEvd2Angle(nAxis, m_aSMot[nAxis].nPos); }

// ---- Speed Control ----
void CMotor::Set_Turn(int nAxis, int nEvd)
{
	if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return;
	Push_Id(nAxis);
	Read_Motor_Push(nAxis);
	m_aSMot[nAxis].bEn = true; 
	Set_Flag_Mode(nAxis, true);
	m_aSMot[nAxis].nPos = CalcLimit_Evd(nAxis, nEvd); 
	Set_Flag_NoAction(nAxis, false);
	//Push_Id(nAxis);	
}
int CMotor::Get_Turn(int nAxis) { return m_aSMot[nAxis].nPos; }
int CMotor::Get_Pos_Evd(int nAxis)
{
	int nValue = 0;
	if (m_bMultiTurn == false)
	{
		byte abyteData[2];
		abyteData[1] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + ((m_bMultiTurn == true) ? 4 : 0)] & 0xff);
		abyteData[0] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + ((m_bMultiTurn == true) ? 4 : 0) + 1] & 0xff);
		// 0000 0000  0000 0000
	 	nValue = (int)(((abyteData[0] & 0x0f) << 8) | (abyteData[1] << 0) | ((abyteData[0] & 0x10) << (3 + 8)) | ((abyteData[0] & 0x10) << (2 + 8)) | ((abyteData[0] & 0x10) << (1 + 8)));	
	}
	else
	{
		//bool bMinus = false;
		//	0x10 00 00
		byte abyteData[4];
		abyteData[3] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + 4 + 3] & 0xff);
		abyteData[2] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + 4 + 2] & 0xff);
		/*if (((abyteData[2] & 0x80) != 0) || (abyteData[3] != 0))
		{
			bMinus = true;
			//abyteData[3] |= 0xff;
			//abyteData[2] |= 0xf0;
		}*/
		abyteData[1] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + 4 + 1] & 0xff);
		abyteData[0] = (byte)(m_acRam[nAxis][ _ADDRESS_CALIBRATED_POSITION + 4 + 0] & 0xff);
		// 0000 0000  0000 0000
	 	nValue = (int)(
	 					((abyteData[3] & 0xff) << 24) | 
	 					((abyteData[2] & 0xff) << 16) | 
	 					((abyteData[1] & 0xff) << 8) | 
	 					((abyteData[0] & 0xff) << 0) 
	 					);// | ( ? 0xfffffffffff00000 : 0);
	 	/*if ((bMinus == true) && (nValue > 0))
 		{
 			printf("nValue = %d, %d\r\n", nValue , nValue - 0x100000);
 			nValue -= 0xf00000;
 		}*/
		//printf("0x%02x%02x%02x%02x:0x%08xf\r\n", abyteData[3], abyteData[2], abyteData[1], abyteData[0], nValue);
	}
	return nValue;
}
float CMotor::Get_Pos_Angle(int nAxis) {	return CalcEvd2Angle(nAxis, Get_Pos_Evd(nAxis)); }

///////////////////////////////////// -> Mode & NoAction : no use "PushId()"
// Led Control   									=> use with Send_Motor
void CMotor::Clear_Flag() { for (int i = 0; i < 256; i++) m_aSMot[i].nFlag = _FLAG_NO_ACTION; }
void CMotor::Clear_Flag(int nAxis) { m_aSMot[nAxis].nFlag = _FLAG_NO_ACTION; }//{ m_aSMot[nAxis].nFlag = nFlag; Push_Id(nAxis); }
void CMotor::Set_Flag(int nAxis, bool bStop, bool bMode_Speed, bool bLed_Green, bool bLed_Blue, bool bLed_Red, bool bNoAction) { m_aSMot[nAxis].nFlag = ((bStop == true) ? _FLAG_STOP : 0) | ((bMode_Speed == true) ? _FLAG_MODE_SPEED : 0) | ((bLed_Green == true) ? _FLAG_LED_GREEN : 0) | ((bLed_Blue == true) ? _FLAG_LED_BLUE : 0) | ((bLed_Red == true) ? _FLAG_LED_RED : 0) | ((bNoAction == true) ? _FLAG_NO_ACTION : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Stop(int nAxis, bool bStop) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xfe) | ((bStop == true) ? _FLAG_STOP : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Mode(int nAxis, bool bMode_Speed) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xfd) | ((bMode_Speed == true) ? _FLAG_MODE_SPEED : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Led(int nAxis, bool bGreen, bool bBlue, bool bRed) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xe3) | ((bGreen == true) ? _FLAG_LED_GREEN : 0) | ((bBlue == true) ? _FLAG_LED_BLUE : 0) | ((bRed == true) ? _FLAG_LED_RED : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Led_Green(int nAxis, bool bGreen) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xfb) | ((bGreen == true) ? _FLAG_LED_GREEN : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Led_Blue(int nAxis, bool bBlue) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xf7) | ((bBlue == true) ? _FLAG_LED_BLUE : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
void CMotor::Set_Flag_Led_Red(int nAxis, bool bRed) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xef) | ((bRed == true) ? _FLAG_LED_RED : 0); Push_Id(nAxis); Read_Motor_Push(nAxis); }
// NoAction 은 push 안함
void CMotor::Set_Flag_NoAction(int nAxis, bool bNoAction) { m_aSMot[nAxis].nFlag = (m_aSMot[nAxis].nFlag & 0xdf) | ((bNoAction == true) ? _FLAG_NO_ACTION : 0); }

// 1111 1101
//int 	CMotor::Get_Flag(int nAxis) { return m_aSMot[nAxis].nFlag; }
int	CMotor::Get_Flag_Mode(int nAxis) { return (((m_aSMot[nAxis].nFlag & _FLAG_MODE_SPEED) != 0) ? 1 : 0); }

bool CMotor::Get_Flag_Led_Green(int nAxis) { return (((m_aSMot[nAxis].nFlag & 0x04) != 0) ? true : false); }
bool CMotor::Get_Flag_Led_Blue(int nAxis) { return (((m_aSMot[nAxis].nFlag & 0x08) != 0) ? true : false); }
bool CMotor::Get_Flag_Led_Red(int nAxis) { return (((m_aSMot[nAxis].nFlag & 0x10) != 0) ? true : false); }

//////////////////////////////////////
// Motor Control - Move Motor(Action)

void CMotor::Send_Motor(int nMillisecond)
{
	if ((m_bStop == true) || (m_bEms == true)) return;
	
	m_nMotorCnt_Back = m_nMotorCnt; // 나중에 waitaction 에서 사용

	//Sync_Seq();

	int nID;
	int i = 0;
	////////////////////////////////////////////////

	byte pbyteBuffer[256];//[1 + 4 * m_nMotorCnt];
	int nPos;
	int nFlag;
		
	// region S-Jog Time
	int nCalcTime = CalcTime_ms(nMillisecond);
	pbyteBuffer[i++] = (byte)(nCalcTime & 0xff);
	if (m_bMultiTurn == true) pbyteBuffer[i++] = (byte)((nCalcTime >> 8) & 0xff);
	int nCnt = m_nMotorCnt;//_SIZE_MOTOR_MAX;//m_nMotorCnt;
	for (int nAxis2 = 0; nAxis2 < nCnt; nAxis2++)
	{
		int nAxis = Pop_Id();//nAxis2;// i;//Pop_Id();
		//printf("ID=%d\r\n", nAxis);
		if (m_aSMot[nAxis].bEn == true)
		{
			//printf("ID=%d\r\n", nAxis);
			//nPos |= _JOG_MODE_SPEED << 10;  // 속도제어 
			// Position
			nPos = Get(nAxis);

			if (m_bMultiTurn == false)
			{
				if (nPos < 0)
				{
				    nPos *= -1;
				    nPos |= 0x4000;
				}
			}
			else
			{
				/*if (nPos < 0)
				{
				    nPos *= -1;
				    nPos |= 0x100000;
				}*/
			}
			if (m_bMultiTurn == false) 
			{
				pbyteBuffer[i++] = (byte)(nPos & 0xff);
				pbyteBuffer[i++] = (byte)((nPos >> 8) & 0xff);
			}
			else
			{
				//printf("nPos=%d\r\n", nPos);
				unsigned int unPos = (unsigned int)nPos;
				pbyteBuffer[i++] = (byte)(unPos & 0xff);
				pbyteBuffer[i++] = (byte)((unPos >> 8) & 0xff);
				pbyteBuffer[i++] = (byte)((unPos >> 16) & 0xff);
				pbyteBuffer[i++] = (byte)((unPos >> 24) & 0xff);
			}
			// Set-Flag
			nFlag = Get_Flag(nAxis);
			pbyteBuffer[i++] = (byte)(nFlag & 0xff);
			Set_Flag_NoAction(nAxis, true); // 동작 후 모터 NoAction을 살려둔다.

			// 모터당 아이디(후면에 붙는다)
			nID = GetID_By_Axis(nAxis);
			pbyteBuffer[i++] = (byte)(nID & 0xff);

			m_aSMot[nAxis].bEn = false;
			////////////////////////////////////////////////
		}
		
	}
	Make_And_Send_Packet(0xfe, 0x06, i, pbyteBuffer);
	//pbyteBuffer = null;
	Clear_Flag();
	//m_nSeq_Motor++; // reserve;
	m_ulDelay = nMillisecond;
	Wait_Ready();
}

void CMotor::Wait_Ready()
{
	m_CTmr.Set();
}

int m_nReadMotor_Index = 0;
bool CMotor::Wait_Motor(int nMilliseconds)
{	
	if (Read_Motor_IsReceived() == true) Read_Motor();	

#if 1
	else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
	{
		if (m_CTmr_Timeout.Get() > m_nTimeout)
		{
			Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
			m_CTmr_Timeout.Set();
			m_nRetrieve++;
		}
	}
	else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
	#ifdef _TIMER_ARRAY
		else if (m_aCTmr_Timeout.Get() > m_nTimeout)
	#else
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
	#endif
#endif		
	{
		printf("[Receive-Wait_Motor(ms)*] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
		//printf("[Receive-Wait_Motor(ms)] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
		//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
		m_bError = true;
#endif
		Read_Motor();	
	}
	while(m_CTmr.Get() < nMilliseconds) 
	{
		if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) { Sync_Seq(); return false; }//m_nSeq_Motor_Back = m_nSeq_Motor; return false; }

		if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
		else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
		{
			if (m_CTmr_Timeout.Get() > m_nTimeout)
			{
				Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
				m_CTmr_Timeout.Set();
				m_nRetrieve++;
			}
		}
		else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif	
		{
			printf("[Receive-Wait_Motor(ms)] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//printf("[Receive-Wait_Motor(ms)] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();	
		}
		
		usleep(_TIME_DELAY);
	}
	//m_nSeq_Motor_Back = m_nSeq_Motor; 
	Sync_Seq();
	return true;
}
bool CMotor::Wait_Motor()
{
	if (Read_Motor_IsReceived() == true) Read_Motor();
#if 1
	else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
	{
		if (m_CTmr_Timeout.Get() > m_nTimeout)
		{
			Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
			m_CTmr_Timeout.Set();
			m_nRetrieve++;
		}
	}
	else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
	else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif	
	{
		printf("[Receive-Wait_Motor()*] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);;
		//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
		m_bError = true;
#endif
		Read_Motor();	
	}
	while(m_CTmr.Get() < m_ulDelay) 
	{
		if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) { Sync_Seq(); return false; }//m_nSeq_Motor_Back = m_nSeq_Motor; return false; }		
		if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
		else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
		{
			if (m_CTmr_Timeout.Get() > m_nTimeout)
			{
				Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
				m_CTmr_Timeout.Set();
				m_nRetrieve++;
			}
		}
		else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif	
		{
			printf("[Receive-Wait_Motor()] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();	
		}
		usleep(_TIME_DELAY);
	}
	Sync_Seq();//m_nSeq_Motor_Back = m_nSeq_Motor; 
	return true;
}


bool CMotor::Wait_Position(int nAxis, float fTargetAngle, int nMilliseconds)
{
	if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
	else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
	{
		if (m_CTmr_Timeout.Get() > m_nTimeout)
		{
			Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
			m_CTmr_Timeout.Set();
			m_nRetrieve++;
		}
	}
	else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
	else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif
	{
		printf("[Receive-Wait_Motor()*] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);;
		//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
		m_bError = true;
#endif
		Read_Motor();	
	}
	float afMot[_SIZE_MOTOR_MAX];
	bool abMot[_SIZE_MOTOR_MAX];
	while(m_CTmr.Get() < m_ulDelay) 
	{
		if (abMot[nAxis] == false) 
		{
			if (m_abReceivedPos[nAxis] == true) 
			{
				abMot[nAxis] = true;
				afMot[nAxis] = Get_Pos_Angle(nAxis);
			} 
		}
		else
		{
			if ((Get_Angle(nAxis) - afMot[nAxis]) >= 0)
			{
				if (Get_Pos_Angle(nAxis) >= fTargetAngle)
				{
					//printf("Break\n");
					break;
				}
			}
			else
			{
				if (Get_Pos_Angle(nAxis) <= fTargetAngle)
				{
					//printf("Break\n");
					break;
				}
			}
		}

		if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) { Sync_Seq(); return false; }//m_nSeq_Motor_Back = m_nSeq_Motor; return false; }		
		if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
		else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
		{
			if (m_CTmr_Timeout.Get() > m_nTimeout)
			{
				Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
				m_CTmr_Timeout.Set();
				m_nRetrieve++;
			}
		}
		else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif		
		{
			printf("[Receive-Wait_Motor()] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();	
		}
		usleep(_TIME_DELAY);
	}
	Sync_Seq();//m_nSeq_Motor_Back = m_nSeq_Motor; 
	return true;
}
/*
bool CMotor::Wait_Delay(int nMilliseconds)
{
	m_CTmr.Set();
	if (Read_Motor_IsReceived() == true) Read_Motor();	
	else if (m_CTmr_Timeout.Get() > m_nTimeout)
	{
		printf("[Receive-Wait_Delay(ms)] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
		//printf("[Receive-Wait_Delay()] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
		//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
		m_bError = true;
#endif
		Read_Motor();	
	}
	while(m_CTmr.Get() < nMilliseconds) 
	{
		if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) { Sync_Seq(); return false; }//m_nSeq_Motor_Back = m_nSeq_Motor; return false; }
		if (Read_Motor_IsReceived() == true) Read_Motor();	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
		{
			printf("[Receive-Wait_Delay(ms)] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//printf("[Receive-Wait_Delay()] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();	
		}
		usleep(_TIME_DELAY);
	}
	Sync_Seq();//m_nSeq_Motor_Back = m_nSeq_Motor; 
	return true;
}*/

bool CMotor::Wait_Delay(int nMilliseconds)
{
	m_CTmr.Set();
	if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
	else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
	{
		if (m_CTmr_Timeout.Get() > m_nTimeout)
		{
			Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
			m_CTmr_Timeout.Set();
			m_nRetrieve++;
		}
	}
	else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
	else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif		
	{
		printf("[Receive-Wait_Delay(ms)*] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
		//printf("[Receive-Wait_Delay()] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
		//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
		m_bError = true;
#endif
		Read_Motor();	
	}
	while(m_CTmr.Get() < nMilliseconds) 
	{
		if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) { Sync_Seq(); return false; }//m_nSeq_Motor_Back = m_nSeq_Motor; return false; }
		if (Read_Motor_IsReceived() == true) Read_Motor();	
#if 1
		else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
		{
			if (m_CTmr_Timeout.Get() > m_nTimeout)
			{
				Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
				m_CTmr_Timeout.Set();
				m_nRetrieve++;
			}
		}
		else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif	
		{
			printf("[Receive-Wait_Delay(ms)] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//printf("[Receive-Wait_Delay()] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();	
		}
		usleep(_TIME_DELAY);
	}
	Sync_Seq();//m_nSeq_Motor_Back = m_nSeq_Motor; 
	return true;
}

// 현 시점을 중심으로 타이머를 초기화
char CMotor::WaitAction_SetTimer()
{
	m_lWaitActionTimer = 0;	
	m_CTmr_Motion.Set();	
	return true; // 현시점에서는 TRUE 만을 무조건 리턴. 나중에 필요시 넣도록...
}

// WaitAction_SetTimer 한 시간부터 지정시간이 넘었는지를 체크. 넘기 까지 Wait
char CMotor::WaitAction_ByTimer(long t)
{
	if (t <= 0) return true;	// t 값이 0 보다 작다면 대기문이 필요없으므로 완료를 보냄.
	m_lWaitActionTimer += t;
//	printf("[Action] WaitAction_ByTimer=%d(%ld)\n", t, m_lWaitActionTimer);

	while(
		((m_CTmr_Motion.Get() >= m_lWaitActionTimer) == false) && (m_bEms == false)  && (m_bStop== false)  //&& //(m_bPause == FALSE) && (m_bMotionEnd == FALSE)
		)
	{		
		usleep(_TIME_DELAY);
		if (Read_Motor_IsReceived() == true) Read_Motor();
#if 1
		else if ((m_nRetrieve < _CNT_RETRIEVE) && (m_nReadCnt > 0))
		{
			if (m_CTmr_Timeout.Get() > m_nTimeout)
			{
				Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
				m_CTmr_Timeout.Set();
				m_nRetrieve++;
			}
		}
		else if  (m_CTmr_Timeout.Get() > m_nTimeout)
#else	
		else if (m_CTmr_Timeout.Get() > m_nTimeout)
#endif	
		{
			printf("[Receive-WaitAction_ByTimer(ms)] Time out Error(%d ms)- ID[%d]Seq[%d]Seq_Back[%d]Index[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID, m_nSeq_Receive, m_nSeq_Receive_Back, m_nReadMotor_Index);
			//printf("[Receive-Wait_Delay()] Time out Error(%d ms)- ID[%d]\r\n", m_nTimeout, m_aSRead[m_nReadMotor_Index].nID);
			//m_nSeq_Receive_Back = m_nSeq_Receive;
#ifdef _TEST_RECEIVE
			m_bError = true;
#endif
			Read_Motor();
		}
	}
	return true;
}

void CMotor::Read_Ram(int nAxis, int nAddress, int nLength)
{
	if (IsOpen() == false) return;

	//m_bBusy = true; // Request Data

	int nID = GetID_By_Axis(nAxis);
	int nDefaultSize = _CHECKSUM2 + 1;

	int i = 0;
	// Header
	byte abyteBuffer[256];
	abyteBuffer[_HEADER1] = 0xff;
	abyteBuffer[_HEADER2] = 0xff;
	// ID = 0xFE : 전체명령, 0xFD - 공장출하시 설정 아이디
	abyteBuffer[_ID] = (byte)(nID & 0xff);
	// Cmd
	abyteBuffer[_CMD] = 0x04; // Ram 영역을 읽어온다.

	/////////////////////////////////////////////////////
	// Data
	abyteBuffer[nDefaultSize + i++] = (byte)(nAddress & 0xff);// Register address

	////////
	abyteBuffer[nDefaultSize + i++] = (byte)(nLength & 0xff);// Data Size
	////////
	/////////////////////////////////////////////////////

	//Packet Size
	abyteBuffer[_SIZE] = (byte)((nDefaultSize + i) & 0xff);

	MakeCheckSum(nDefaultSize + i, abyteBuffer);//, out abyteBuffer[_CHECKSUM1], out abyteBuffer[_CHECKSUM2]);

	// 보내기 전에 Tick 을 Set 한다.
	//Tick_Send(nAxis);
	Sync_Seq();
	//usleep(0);
	SendPacket(abyteBuffer, nDefaultSize + i);
#ifdef _TEST_RECEIVE
	if (m_bError == true) printf("Read_Ram - after error\r\n");
#endif
	m_CTmr_Timeout.Set();
}

void CMotor::Read_Motor(int nAxis) { Read_Ram(nAxis, _ADDRESS_TORQUE_CONTROL + ((m_bMultiTurn == true) ? 4 : 0), 16); }//8); }
bool CMotor::Read_Motor_IsReceived()
{
	//if (m_bMultiTurn == true) return true;

	//m_nRetrieve = 0;
	
	if (m_nSeq_Receive_Back != m_nSeq_Receive)
	{
		Sync_Seq();
		//m_nSeq_Receive_Back = m_nSeq_Receive;
		//printf("Read_Motor_IsReceived() == true\r\n");
		return true;
	}
	return false;
}
void CMotor::Read_Motor() 
{	
	if (m_nReadCnt <= 0) return;
	//if (m_bMultiTurn == true) return;

	m_nRetrieve = 0;
	
	m_nReadMotor_Index = (m_nReadMotor_Index + 1) % m_nReadCnt;
	//printf("Read_Motor()-%d, %d\r\n", m_nSeq_Receive, m_nSeq_Receive_Back);
	//m_nSeq_Receive_Back = m_nSeq_Receive;
	//Sync_Seq();
	Read_Motor(m_aSRead[m_nReadMotor_Index].nID);
}
//////////////////////////////////////
// Setting
//void Set_Ram(int nId, 
//void Set_Rom(int nId, 
void CMotor::Push_Id(int nAxis) { if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return; if (IsCmd(nAxis) == false) m_acEn[m_nMotorCnt++] = nAxis;  }
int CMotor::Pop_Id() { if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return -1; if (m_nMotorCnt > 0) return m_acEn[--m_nMotorCnt]; return -1; }

// Push Motor ID for checking(if you set a Motor ID with this function, you can get a feedback data with delay function)
void CMotor::Read_Motor_Push(int nAxis) { if (m_bProgEnd == true) return; if (Read_Motor_Index(nAxis) >= 0) return; if ((nAxis < 0) || (nAxis >= 254)) return; m_aSRead[m_nReadCnt].nID= nAxis;  m_aSRead[m_nReadCnt].bEnable = true; m_aSRead[m_nReadCnt].nAddress_First = _ADDRESS_TORQUE_CONTROL + ((m_bMultiTurn == true) ? 4 : 0); m_aSRead[m_nReadCnt].nAddress_Length= 8; m_nReadCnt++; }
// You can check your Motor ID for feedback, which you set or not.
int CMotor::Read_Motor_Index(int nAxis) { if (m_bProgEnd == true) return -1; for (int i = 0; i < m_nReadCnt; i++) { if (m_aSRead[i].nID == nAxis) return i; } return -1; }

// use this when you don't want to get some motor datas.
void CMotor::Read_Motor_Clear() { m_nReadCnt = 0; memset(m_aSRead, 0, sizeof(SRead_t) * _SIZE_MOTOR_MAX);}

// detail option
void CMotor::Read_Motor_Change_Address(int nAxis, int nAddress, int nLength) { int nIndex = Read_Motor_Index(nAxis); if (nIndex >= 0) {  m_aSRead[nIndex].nAddress_First = nAddress; m_aSRead[nIndex].nAddress_Length= nLength; } }
void CMotor::Read_Motor_ShowMessage(bool bTrue) { m_bShowMessage = bTrue; }
bool CMotor::IsCmd(int nAxis)
{
	for (int i = 0; i < m_nMotorCnt; i++) if (m_acEn[i] == nAxis) return true;
	return false;
}

void CMotor::SendPacket(byte *buffer, int nLength)
{
//	unsigned char *szPacket = (unsigned char *)malloc(sizeof(unsigned char) *( nLength + 2));
	if (IsOpen() == true)
	{
		write(m_nTty, buffer, sizeof(byte) * (nLength));// + 2));
		tcflush(m_nTty, TCIFLUSH);
#if 0
		printf("[SendPacket()]\r\n");
		for (int nMsg = 0; nMsg < nLength; nMsg++)
		{
			printf("0x%02X, ", buffer[nMsg]);
		}
		printf("\r\n");
#endif
	}
}
void CMotor::SendPacket_Socket(byte *buffer, int nLength)
{
	if (IsOpen_Socket() == true)
	{
		write(m_nClientMotionFd, buffer, sizeof(byte) * (nLength));// + 2));
		tcflush(m_nClientMotionFd, TCIFLUSH);
#if 0
		printf("[SendPacket_Socket()]\r\n");
		for (int nMsg = 0; nMsg < nLength; nMsg++)
		{
			printf("0x%02X, ", buffer[nMsg]);
		}
		printf("\r\n");
#endif
	}
}
void CMotor::Make_And_Send_Packet(int nID, int nCmd, int nDataByteSize, byte* pbytePacket)
{	
	int nDefaultSize = _CHECKSUM2 + 1;
	int nSize = nDefaultSize + nDataByteSize;

	byte pbyteBuffer[nSize];//(byte *)malloc(sizeof(byte) * nSize);

	// Header
	pbyteBuffer[_HEADER1] = (byte)0xff;
	pbyteBuffer[_HEADER2] = (byte)0xff;
	// ID = 0xFE : 전체명령, 0xFD - 공장출하시 설정 아이디
	pbyteBuffer[_ID] = (byte)(nID & 0xff);
	// Cmd
	pbyteBuffer[_CMD] = (byte)(nCmd & 0xff);
	/////////////////////////////////////////////////////
	int i = 0;
	for (int j = 0; j < nDataByteSize; j++) pbyteBuffer[nDefaultSize + i++] = pbytePacket[j];
	/////////////////////////////////////////////////////

	//Packet Size
	pbyteBuffer[_SIZE] = (byte)(nSize & 0xff);
	MakeCheckSum(nSize, pbyteBuffer);
#if 0
	printf("[Make_And_Send_Packet()]\r\n");
	for (int nMsg = 0; nMsg < nDefaultSize + i; nMsg++)
	{
		printf("0x%02x, ", pbyteBuffer[nMsg]);
	}
	printf("\r\n");
#endif
	SendPacket(pbyteBuffer, nSize);

	//free(pbyteBuffer);
}

void CMotor::MakeCheckSum(int nAllPacketLength, byte* buffer)
{
	int nHeadSize = _CHECKSUM2 + 1;
	buffer[_CHECKSUM1] = (byte)(buffer[_SIZE] ^ buffer[_ID] ^ buffer[_CMD]);
	for (int j = 0; j < nAllPacketLength - nHeadSize; j++)
	{
	    buffer[_CHECKSUM1] ^= buffer[nHeadSize + j];
	}
	buffer[_CHECKSUM1] = (byte)(buffer[_CHECKSUM1] & 0xfe);
	buffer[_CHECKSUM2] = (byte)(~buffer[_CHECKSUM1] & 0xfe);
}

void CMotor::Motion_Play(const char *strFileName)
{
	if (BinaryFileOpen(strFileName, &m_SMotion) == true)
	{
		if (m_SMotion.nFrameSize > 0)
		{			
			m_bStart = true;

			WaitAction_SetTimer();

			printf("Frame=%d\r\n", m_SMotion.nFrameSize);
			for (int i = 0; i < m_SMotion.nFrameSize; i++)
			{
				printf("bEn=%d\r\n", m_SMotion.pSTable[i].bEn);
				if (m_SMotion.pSTable[i].bEn == true)
				{
					PlayFrame(m_SMotion.pSTable[i]);

					int nDelay = m_SMotion.pSTable[i].nTime + m_SMotion.pSTable[i].nDelay;

					if (nDelay > 0) WaitAction_ByTimer(nDelay);
					printf("nDelay=%d\r\n", nDelay);
				}
			}

			m_bStart = false;
			//m_bMotionEnd = false;
		}
	}
	

	// #region 실제 모션후 메모리 해제
	for(int j = 0; j < m_SMotion.nFrameSize; j++)
	{
		// test
//		printf("Value=[0]%d\n", m_SMotion.pSTable[0].bEn);
		
		free(m_SMotion.pSTable[j].pnMot);
		free(m_SMotion.pSTable[j].pnLed);
		free(m_SMotion.pSTable[j].pbEn);
		free(m_SMotion.pSTable[j].pbType);
	}
	free(m_SMotion.pSTable);
}

void CMotor::PlayFrame(SMotionTable_t STable)
{
	if ((m_bStop == false) && (m_bEms == false))// && (m_bMotionEnd == false))
	{
		//m_CMotor.ResetStop();
		SetTorque(true, true);
		for (int nAxis = 0; nAxis < m_SMotion.nMotorCnt; nAxis++)//.nMotorCnt; nAxis++)
		{
			if (
			    //(m_CHeader.pSMotorInfo[nAxis]. == EType_t._0102) || // 엔코더이거나
			    (m_aSMotorInfo[nAxis].nMotorControlType != 0) // 위치제어가 아니라면 //// Motor Control type => 0: Position, 1: Speed type
			    //(m_abEnc[nAxis] == true) || // 엔코더이거나
			    //(Grid_GetFlag_Type(m_nCurrntCell, nAxis) == true) // 위치제어가 아니라면
			    )
			{
			    // 모드에 따라 계산법이 틀려지기에 모드 셋팅부터 먼저 한다.
			    Set_Flag_Mode(nAxis, m_aSMotorInfo[nAxis].nMotorControlType);
			    SetParam_Dir(nAxis, m_aSMotorInfo[nAxis].nMotorDir);

			    //float fTmpVal = (float)Math.Round(Convert.ToSingle(OjwGrid.GetData(nLine, nAxis)));
			    int nVal = STable.pnMot[nAxis];//CalcAngle2Evd(nAxis, fTmpVal);
			    if (nVal < 0)
			    {
			        nVal *= -1;
			        nVal |= 0x4000;
			    }
			    Set_Turn(nAxis, nVal);

			    Set_Flag_Led(nAxis, 
			        Get_Flag_Led_Green(STable.pnLed[nAxis]),
			        Get_Flag_Led_Blue(STable.pnLed[nAxis]),
			        Get_Flag_Led_Red(STable.pnLed[nAxis])
			        );
			}
			else
			{
			    // 모드에 따라 계산법이 틀려지기에 모드 셋팅부터 먼저 한다.
			    Set_Flag_Mode(nAxis, m_aSMotorInfo[nAxis].nMotorControlType);
			    SetParam_Dir(nAxis, m_aSMotorInfo[nAxis].nMotorDir);

//printf("[PlayFrame][%d]:%d\r\n", nAxis, STable.pnMot[nAxis]);
//SetParam_Dir(nAxis, 0);
			    Set(nAxis, STable.pnMot[nAxis]);
			    printf("[%d]%d, %d, %.2f, %.2f\r\n", nAxis, STable.pnMot[nAxis], Get(nAxis), CalcEvd2Angle(nAxis, STable.pnMot[nAxis]), Get_Angle(nAxis));//Get_Angle(nAxis));
			    Set_Flag_Led(nAxis,
			        Get_Flag_Led_Green(STable.pnLed[nAxis]),
			        Get_Flag_Led_Blue(STable.pnLed[nAxis]),
			        Get_Flag_Led_Red(STable.pnLed[nAxis])
			        );
			}
		}
		Send_Motor(STable.nTime);
		// Sound & Buzz
		//m_CMotor.Mpsu_Play_HeadLed_Buzz(STable.nData4, STable.nData3);
	}
}

bool CMotor::SetParam_with_File(const char *strHeaderFile)
{
	bool bFileOpened = false;
	FILE *pfileDesigner;
	char szFilename[256];
	int nSize = 3;// + 8;
	char szFileType[nSize];
	char *buff2 = NULL;
	char buff[1024];
	int nVersion = 0;
	int i;
	//sprintf(szFilename, "%s/%s.act", "dmt", strFileName);
	sprintf(szFilename, "%s", strHeaderFile);
	if ( ( pfileDesigner = fopen(szFilename, "rb") ) == NULL ) 
	{
		printf("[SetParam_with_File] File[%s] open error\n", strHeaderFile);
		return false;
	}
	bFileOpened = true;


	// header mark & version load
	memset(szFileType, 0, sizeof(char) * nSize);
	fread(szFileType, sizeof(char), nSize, pfileDesigner);

	if ( strncmp(szFileType, "DHF", nSize) == 0 ) 
	{
		printf("Found Header -> DHF, but we can not do anything...\r\n");
	}
	else if ( strncmp(szFileType, "OJW", nSize) == 0 ) 	
	{
		// version(8) - 01.01.00
		fread(buff, sizeof(char), 8, pfileDesigner);
		int nPos;// = 8;
		for (i =0; i < 8; i++) nVersion = ((buff[i] != '.') ? nVersion * 10 + (int)(buff[i] - 0x30) : nVersion);

		if (nVersion > 010000) 
		{
			fread(buff, sizeof(char), 4, pfileDesigner);
			//nPos += 4;		
		}
		
		fread(buff, sizeof(char), 60, pfileDesigner);
		nPos = 0;
		short sData;
		int nData;
		float fData;

		//nModelNum <= short(2)
		nPos += 2;
		
		// title(21)
		nPos += 21;
		// BackColor(4)
		nPos += 4;
		//Motor Count(2)		
		memcpy(&sData, &buff[nPos], sizeof(short));
		nPos += 2;
		//int nMotorCnt = ((buff[nPos + 1] << 8) | buff[nPos]);

		//nPos += 2;
		printf("Found Header -> OJW : Ver %d, Motor[%d]\r\n", nVersion, sData);
		int nMotorCnt = (int)sData;
		// Init Angle(Pan, Tilt, Swing - 4, 4, 4), Init Pos(X, Y, Z - 4, 4, 4)
		nPos += 24;

		// scale(4), 2wheel Counter(1), 3wheel Counter(1), 4Wheel Counter(1)
		nPos += 7;

		nPos = 0;
		nSize = 90;
		if (nVersion >= 010200)
			nSize += 4;
		int nMotorInfoFrameSize = nMotorCnt; // 256
		buff2 = (char *)malloc(sizeof(char) * nMotorInfoFrameSize * nSize);
		fread(buff2, sizeof(char), nMotorInfoFrameSize * nSize, pfileDesigner);
		//SMotorInfo_t SMotor;
		for (i = 0; i < nMotorInfoFrameSize; i++)//256; i++)
		{
			// ID (2)
			sData = (int)((buff2[nPos + 1] << 8) | buff2[nPos]);						
			//memcpy(&sData, &buff2[nPos], sizeof(short));
			m_aSMotorInfo[i].nMotorID = (int)sData;
			SetParam_RealID(i, sData);
			nPos += 2;
			// Direction(forward, backward) (2)
			memcpy(&sData, &buff2[nPos], sizeof(short));
			SetParam_Dir(i, sData);
			m_aSMotorInfo[i].nMotorDir = (int)sData;
			//printf("[%d]DIR=%d\r\n", i, sData);
			nPos += 2;
			// LimitUp(float - 4)
			memcpy(&fData, &buff2[nPos], sizeof(float));
			SetParam_LimitUp(i, 0);//fData);	
			//printf("LimitUp=%.2f\r\n", fData);
			m_aSMotorInfo[i].fLimit_Up = 0;//fData;
			nPos += 4;
			// LimitDown(float - 4)
			memcpy(&fData, &buff2[nPos], sizeof(float));
			SetParam_LimitDown(i, 0);//fData);
			//printf("LimitDown=%.2f\r\n", fData);
			m_aSMotorInfo[i].fLimit_Down = 0;//fData;
			nPos += 4;
			if (nVersion < 010200)
			{
				// Center(2)			
				memcpy(&sData, &buff2[nPos], sizeof(short));
				SetParam_CenterEvdValue(i, (float)sData);
				m_aSMotorInfo[i].nCenter_Evd = (int)sData;
				nPos += 2;
				// Mech Move(2)			
				memcpy(&sData, &buff2[nPos], sizeof(short));
				SetParam_MechMove(i, (float)sData);
				m_aSMotorInfo[i].nMechMove = (int)sData;
				nPos += 2;
			}
			else
			{
				// Center(4)			
				//memcpy(&nData, &buff2[nPos], sizeof(int));
				nData = (int)((buff2[nPos + 3] << 24) | (buff2[nPos + 2] << 16) | (buff2[nPos + 1] << 8) | buff2[nPos]);	
				SetParam_CenterEvdValue(i, (float)nData);
				m_aSMotorInfo[i].nCenter_Evd = nData;
				nPos += 4;	
				// Mech Move(4)	
				memcpy(&nData, &buff2[nPos], sizeof(int));
				SetParam_MechMove(i, (float)nData);
				m_aSMotorInfo[i].nMechMove = nData;
				nPos += 4;				
				//printf("[%d]MechMove:%d, Center: %d\r\n", i, m_aSMotorInfo[i].nMechMove, m_aSMotorInfo[i].nCenter_Evd);
			}
			// Mech Angle
			memcpy(&fData, &buff2[nPos], sizeof(float));
			SetParam_Degree(i, fData);
			m_aSMotorInfo[i].fMechAngle = fData;
			nPos += 4;	
			// Init Angle
			memcpy(&fData, &buff2[nPos], sizeof(float));
			//SetParam_(i, fData);
			m_aSMotorInfo[i].fInitAngle = fData;
			nPos += 4;	
			// Init Angle2			
			memcpy(&fData, &buff2[nPos], sizeof(float));			
			m_aSMotorInfo[i].fInitAngle2 = fData;
			nPos += 4;	

			// reserve ( 2, 4, 4, 4, 4, 4, 4)
			nPos += 2 + 4 + 4 + 4 + 4 + 4 + 4; // 26

			// Nickname(32);
			nPos += 32;

			// Group Number(2)
			nPos += 2;

			// Mirror Axis Number(2)
			nPos += 2;			
			// Motor control type (2)
			nPos += 2;
		}

		free(buff2);
	}
	else
	{
		printf("[SetParam_with_File]header mark not founded \n");
		fclose(pfileDesigner);
		return false;
	}

	fclose(pfileDesigner);
	return true;
}

#define _STR_EXT "dmt"
#define _STR_EXT_UP "DMT"
#define _STR_VER_V_12 "1.2"
#define _STR_VER_V_11 "1.1"
#define _STR_VER_V_10 "1.0"
#define _SIZE_FILE_NAME 21
bool CMotor::BinaryFileOpen(const char *strFileName, SMotion_t * pSMotion)
{
       bool bFileOpened = false;
	FILE *pfileAction;
	char szFilename[256];
	//sprintf(szFilename, "%s/%s.act", "dmt", strFileName);
	sprintf(szFilename, "%s", strFileName);
	if ( ( pfileAction = fopen(szFilename, "rb") ) == NULL ) 
	{
		printf("[BinaryFileOpen] File[%s](Binary) open error\n", szFilename);
		return false;
	}
	bFileOpened = true;

	// header mark & version load
	memset(pSMotion->strVersion, 0, sizeof(char) * 6);
	fread(pSMotion->strVersion, sizeof(char), 6, pfileAction);

	if (
		( strncmp(_STR_EXT, pSMotion->strVersion, strlen(_STR_EXT)) != 0 ) && 
		( strncmp(_STR_EXT_UP, pSMotion->strVersion, strlen(_STR_EXT_UP)) != 0 )
		)
	{
		printf("[BinaryFileOpen]header mark not founded \n");
		fclose(pfileAction);
		return false;
	}

	// version check
	int nVersion = 10;
	if ( strncmp(_STR_VER_V_10, &pSMotion->strVersion[3], strlen(_STR_VER_V_10)) != 0 ) 
	{
		if ( strncmp(_STR_VER_V_11, &pSMotion->strVersion[3], strlen(_STR_VER_V_11)) == 0 )
		{
			int nVersion = 11;
		}
		else
		{			
			if ( strncmp(_STR_VER_V_12, &pSMotion->strVersion[3], strlen(_STR_VER_V_12)) == 0 )
			{
				int nVersion = 12;
			}
			else
			{			
				printf("[Action]version mismatch\n");
				fclose(pfileAction);
				return false;		
			}	
		}
	}

	// title
	char strTitle[21];

	if (nVersion == 10)
	{
		memset(pSMotion->strTableName, 0, sizeof(char) * 21);
		fread(pSMotion->strTableName, sizeof(char), 21, pfileAction);
		printf("Table Name = %s\r\n", pSMotion->strTableName);


		// Start Position(1)
		int nMemorySize = 14;
		byte *pbyTmp = (byte *)malloc(sizeof(byte) * nMemorySize);
		if (pbyTmp == NULL)
		{
			printf("[BinaryFileOpen]memory alloc error(Header)\n");
			fclose(pfileAction);
			return false;					
		}
		fread(pbyTmp, sizeof(byte), nMemorySize, pfileAction);
		int nPos = 0;
		pSMotion->nStartPosition = (int)((pbyTmp[nPos] >= 0) ? pbyTmp[nPos] : 0);
		nPos++;
		// MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1)
		// Size                            
		pSMotion->nFrameSize = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
		pSMotion->nCommentSize = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
		pSMotion->nCnt_LineComment = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
		pSMotion->nPlayTime = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256 + pbyTmp[nPos + 2] * 256 * 256 + pbyTmp[nPos + 3] * 256 * 256 * 256); nPos += 4;
		pSMotion->nRobotModelNum = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
		pSMotion->nMotorCnt = (int)(pbyTmp[nPos++]);
		// Size - MotionFrame, Comment, Caption, PlayTime
                  
		printf("==Done(Frame Size = %d)==\r\n", pSMotion->nFrameSize);


		// #region 실제 모션
		pSMotion->pSTable = (SMotionTable_t *)malloc(sizeof(SMotionTable_t) * pSMotion->nFrameSize);//new SMotionTable_t[pSMotion->nFrameSize];
		for(int j = 0; j < pSMotion->nFrameSize; j++)
		{
		    pSMotion->pSTable[j].pnMot = (int *)malloc(sizeof(int) * pSMotion->nMotorCnt);
		    pSMotion->pSTable[j].pnLed = (int *)malloc(sizeof(int) * pSMotion->nMotorCnt);//new int[pSMotion->nMotorCnt];
		    pSMotion->pSTable[j].pbEn = (bool *)malloc(sizeof(bool) * pSMotion->nMotorCnt);//new bool[pSMotion->nMotorCnt];
		    pSMotion->pSTable[j].pbType = (bool *)malloc(sizeof(bool) * pSMotion->nMotorCnt);//new bool[pSMotion->nMotorCnt];
		}
		
		int nH = pSMotion->nFrameSize;
		int nData;
		short sData;

		
		nMemorySize = 35 + pSMotion->nMotorCnt * 2;//15 + 24;
		byte *pbyteData = (byte *)malloc(sizeof(byte) * nMemorySize);
		if (pbyTmp == NULL)
		{
			printf("[BinaryFileOpen]memory alloc error(Header)\n");
			fclose(pfileAction);
			return false;					
		}
		
		for (int j = 0; j < nH; j++)
		{
		    nPos = 0;
		    fread(pbyteData, sizeof(byte), nMemorySize, pfileAction);

			//En
		    // #region Enable
		    int nEn = pbyteData[nPos++];
		    pSMotion->pSTable[j].bEn = ((nEn & 0x01) != 0) ? true : false;
		    // #endregion Enable
		    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		    // #region Motor
		    int nMotorCntMax = pSMotion->nMotorCnt;//(int)Math.Max(pSMotion->nMotorCnt, m_CHeader.nMotorCnt);
		    // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
		    for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
		    {
			 if (nAxis >= m_SMotion.nMotorCnt) nPos += 2;
		        else if (nAxis >= pSMotion->nMotorCnt) pSMotion->pSTable[j].pnMot[nAxis] = 0;//0.0f;// 실 모터수량과 맞지 않다면 그 부분을 0 으로 채울 것
		        else
		        {
		        	memcpy(&nData, &pbyteData[nPos], sizeof(byte) * 2); nPos += 2;
		            //nData = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
		            sData = (short)(nData & 0x0fff);
		            if ((sData & 0x800) != 0) sData -= 0x1000;
		            
		            pSMotion->pSTable[j].pnLed[nAxis] = (int)((nData >> 12) & 0x07);
		            pSMotion->pSTable[j].pbType[nAxis] = (bool)(((nData & 0x8000) != 0) ? true : false);
		            pSMotion->pSTable[j].pbEn[nAxis] = (bool)((sData == 0x7ff) ? false : true);

		            if (sData == 0x7ff)
		                pSMotion->pSTable[j].pnMot[nAxis] = 0;//0.0f;
		            else
		                pSMotion->pSTable[j].pnMot[nAxis] = sData;//(int)CalcEvd2Angle(nAxis, (int)sData);

			      printf("Mot[%d] = %d\r\n", nAxis, sData);//pSMotion->pSTable[j].pnMot[nAxis] );
		        }
		    }
		    // #endregion Motor
		    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		    // #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
		    // Speed  
		    pSMotion->pSTable[j].nTime = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;

		    // Delay  
		    pSMotion->pSTable[j].nDelay = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;

		    // Group  
		    pSMotion->pSTable[j].nGroup = (int)(pbyteData[nPos++]);

		    // Command  
		    pSMotion->pSTable[j].nCmd = (int)(pbyteData[nPos++]);

		    // Data0  
		    pSMotion->pSTable[j].nData0 = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
		    // Data1  
		    pSMotion->pSTable[j].nData1 = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
		    //
		    pSMotion->pSTable[j].nData2 = 0; //nPos++;
		    pSMotion->pSTable[j].nData3 = 0; //nPos++;
		    pSMotion->pSTable[j].nData4 = 0; //nPos++;
		    pSMotion->pSTable[j].nData5 = 0; //nPos++;
		    // #endregion Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
		    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		    // #region 추가한 Frame 위치 및 자세
		    nPos += 24;
		    //nPos += 4;
		    //nPos += 4;
		    //nPos += 4;

		    //nPos += 4;
		    //nPos += 4;
		    //nPos += 4;
		    // #endregion 추가한 Frame 위치 및 자세

			printf("%d\r\n", nPos);
		}
		// #endregion 실제 모션

		// 이 뒤의 comment & caption 필요 없음.
		free(pbyteData);
		free(pbyTmp);
		pbyteData = NULL;
		pbyTmp = NULL;
	}
	fclose(pfileAction);
	return true;	
/*
                    //SMotion.nCount = 0;
                    SMotion.strTableName = String.Empty;
                    SMotion.strFileName = strFileName;

                    SMotion.strVersion = String.Empty;
                    SMotion.strComment = String.Empty;
                    SMotion.nStartPosition = 0;
                    SMotion.nFrameSize = 0;
                    SMotion.nCommentSize = 0;
                    SMotion.nCnt_LineComment = 0;
                    SMotion.nPlayTime = 0;
                    SMotion.nRobotModelNum = 0;
                    SMotion.nMotorCnt = 0;
                    SMotion.STable = null;


                    FileInfo f = null;
                    FileStream fs = null;
*/
#if 0
                   
                            // nRobotModelNum 를 읽고 해당 파일을 읽어들인다.
                            #region Header 검증
#if false
                            if (SMotion.nMotorCnt != m_CHeader.nMotorCnt)
                            {
                                //if (bFile == true)
                                //{
                                //    fs.Close();
                                //    f = null;
                                //}
                                this.Cursor = System.Windows.Forms.Cursors.Default;
                                //MessageBox.Show("디자이너 파일의 모터 수량과 맞지 않습니다.(요구모터수량=" + Ojw.CConvert.IntToStr(m_CHeader.nMotorCnt) + ", 모션파일에 정의된 모터수량=" + Ojw.CConvert.IntToStr(nMotorCnt) + ")\n");// 해당 모델에 맞는 모션을 로드하십시오.");
                                //DialogResult dlgRet = MessageBox.Show("무시하고 계속 열겠습니까?", "파일열기", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
                                MessageBox.Show("Motor quantity error.(Motors in 3D Modeling =" + Ojw.CConvert.IntToStr(m_CHeader.nMotorCnt) + ", Motors in File =" + Ojw.CConvert.IntToStr(nMotorCnt) + ")\n");// 해당 모델에 맞는 모션을 로드하십시오.");
                                DialogResult dlgRet = MessageBox.Show("Do you want to continue?", "File Open", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
                                if (dlgRet == DialogResult.OK)
                                {
                                    //MessageBox.Show("Yes");
                                    //return;
                                }
                                else return false;
                            }
#endif
                            #endregion Header 검증

                            #region 실제 모션
                            SMotion.STable = new SMotionTable_t[SMotion.nFrameSize];
                            for(j = 0; j < SMotion.nFrameSize; j++)
                            {
                                SMotion.STable[j].anMot = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].anLed = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].abEn = new bool[SMotion.nMotorCnt];
                                SMotion.STable[j].abType = new bool[SMotion.nMotorCnt];
                            }

                            int nH = SMotion.nFrameSize;
                            int nData;
                            short sData;
                            //float fValue;
                            for (j = 0; j < nH; j++)
                            {
                                //En
                                #region Enable
                                int nEn = byteData[nPos++];
                                SMotion.STable[j].bEn = ((nEn & 0x01) != 0) ? true : false;
                                #endregion Enable
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                #region Motor
                                int nMotorCntMax = (int)Math.Max(SMotion.nMotorCnt, m_CHeader.nMotorCnt);
                                // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
                                for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
                                {
                                    if (nAxis >= m_CHeader.nMotorCnt) nPos += 2;
                                    else if (nAxis >= SMotion.nMotorCnt) SMotion.STable[j].anMot[nAxis] = 0;//0.0f;// 실 모터수량과 맞지 않다면 그 부분을 0 으로 채울 것
                                    else
                                    {
                                        nData = (int)(BitConverter.ToInt16(byteData, nPos)); nPos += 2;
                                        sData = (short)(nData & 0x0fff);
                                        if ((sData & 0x800) != 0) sData -= 0x1000;
                                        
                                        SMotion.STable[j].anLed[nAxis] = (int)((nData >> 12) & 0x07);
                                        SMotion.STable[j].abType[nAxis] = (bool)(((nData & 0x8000) != 0) ? true : false);
                                        SMotion.STable[j].abEn[nAxis] = (bool)((sData == 0x7ff) ? false : true);

                                        if (sData == 0x7ff)
                                            SMotion.STable[j].anMot[nAxis] = 0;//0.0f;
                                        else
                                            SMotion.STable[j].anMot[nAxis] = (int)sData;//CalcEvd2Angle(nAxis, (int)sData);
                                    }
                                }
                                #endregion Motor
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                // Speed  
                                SMotion.STable[j].nTime = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;

                                // Delay  
                                SMotion.STable[j].nDelay = (int)BitConverter.ToInt16(byteData, nPos); nPos += 2;

                                // Group  
                                SMotion.STable[j].nGroup = (int)(byteData[nPos++]);

                                // Command  
                                SMotion.STable[j].nCmd = (int)(byteData[nPos++]);

                                // Data0  
                                SMotion.STable[j].nData0 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                // Data1  
                                SMotion.STable[j].nData1 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                //
                                SMotion.STable[j].nData2 = 0;
                                SMotion.STable[j].nData3 = 0;
                                SMotion.STable[j].nData4 = 0;
                                SMotion.STable[j].nData5 = 0;
                                #endregion Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region 추가한 Frame 위치 및 자세
                                nPos += 24;
                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;

                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;
                                #endregion 추가한 Frame 위치 및 자세
                            }
                            #endregion 실제 모션
#if true
                            string strData_ME = "";
                            string strData_FE = "";

                            // 'M' 'E'
                            strData_ME += (char)(byteData[nPos++]);
                            strData_ME += (char)(byteData[nPos++]);

                            #region Comment Data
                            // Comment
                            byte[] pstrComment = new byte[SMotion.nCommentSize];
                            for (j = 0; j < SMotion.nCommentSize; j++)
                                pstrComment[j] = (byte)(byteData[nPos++]);
                            m_strMotionFile_Comment = System.Text.Encoding.Default.GetString(pstrComment);
                            SMotion.strComment = m_strMotionFile_Comment;
                            pstrComment = null;
                            #endregion Comment Data

                            #region Caption
                            int nLineNum = 0;
                            string strLineComment;
                            byte[] byLine = new byte[46];
                            for (j = 0; j < SMotion.nCnt_LineComment; j++)
                            {
                                nLineNum = (short)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                for (int k = 0; k < 46; k++)
                                    byLine[k] = (byte)(byteData[nPos++]);
                                strLineComment = System.Text.Encoding.Default.GetString(byLine);
                                strLineComment = strLineComment.Trim((char)0);
                                m_CGridMotionEditor.SetCaption(nLineNum, strLineComment);
                            }
                            byLine = null;
                            #endregion Caption

                            // 'T' 'E'
                            strData_FE += (char)(byteData[nPos++]);
                            strData_FE += (char)(byteData[nPos++]);
#endif

                            bFileOpened = true;
                            #endregion FileOpen V1.0
                        }
                        else if (strTmp.ToUpper() == _STR_EXT.ToUpper() + _STR_VER_V_11)
                        {
                            #region FileOpen V1.1
                            //int nPos = 6;   // 앞의 6개는 'DMT1.0' 에 할당

                            #region Header

                            #region 타이틀(21)
                            byte[] byteGetData = new byte[21];
                            for (i = 0; i < 21; i++) byteGetData[i] = 0;
                            for (i = 0; i < 21; i++)
                            {
                                if (byteData[i + nPos] == 0) break;
                                byteGetData[i] = byteData[i + nPos];
                            }
                            SMotion.strTableName = System.Text.Encoding.Default.GetString(byteGetData);
                            nPos += 21;
                            byteGetData = null;
                            #endregion 타이틀(21)

                            #region Start Position(1)
                            int nStartPosition = (int)(byteData[nPos++]);
                            SMotion.nStartPosition = (nStartPosition >= 0) ? nStartPosition : 0;
                            #endregion Start Position(1)

                            #region Size - MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1)
                            // Size
                            SMotion.nFrameSize = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nCommentSize = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nCnt_LineComment = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nPlayTime = (int)(byteData[nPos] + byteData[nPos + 1] * 256 + byteData[nPos + 2] * 256 * 256 + byteData[nPos + 3] * 256 * 256 * 256); nPos += 4;
                            SMotion.nRobotModelNum = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nMotorCnt = (int)(byteData[nPos++]);
                            #endregion Size - MotionFrame, Comment, Caption, PlayTime

                            #endregion Header

                            // nRobotModelNum 를 읽고 해당 파일을 읽어들인다.
                            #region Header 검증
#if false
                            if (SMotion.nMotorCnt != m_CHeader.nMotorCnt)
                            {
                                //if (bFile == true)
                                //{
                                //    fs.Close();
                                //    f = null;
                                //}
                                this.Cursor = System.Windows.Forms.Cursors.Default;
                                MessageBox.Show("디자이너 파일의 모터 수량과 맞지 않습니다.(요구모터수량=" + Ojw.CConvert.IntToStr(m_CHeader.nMotorCnt) + ", 모션파일에 정의된 모터수량=" + Ojw.CConvert.IntToStr(nMotorCnt) + ")\n");// 해당 모델에 맞는 모션을 로드하십시오.");
                                DialogResult dlgRet = MessageBox.Show("무시하고 계속 열겠습니까?", "파일열기", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
                                if (dlgRet == DialogResult.OK)
                                {
                                    //MessageBox.Show("Yes");
                                    //return;
                                }
                                else return false;
                            }
#endif
                            #endregion Header 검증

                            #region 실제 모션
                            SMotion.STable = new SMotionTable_t[SMotion.nFrameSize];
                            for (j = 0; j < SMotion.nFrameSize; j++)
                            {
                                SMotion.STable[j].anMot = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].anLed = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].abEn = new bool[SMotion.nMotorCnt];
                                SMotion.STable[j].abType = new bool[SMotion.nMotorCnt];
                            }

                            int nH = SMotion.nFrameSize;
                            int nData, nData2;
                            //short sData;
                            //float fValue;
                            for (j = 0; j < nH; j++)
                            {
                                //En
                                #region Enable
                                int nEn = byteData[nPos++];
                                SMotion.STable[j].bEn = ((nEn & 0x01) != 0) ? true : false;
                                #endregion Enable
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                #region Motor
                                int nMotorCntMax = (int)Math.Max(SMotion.nMotorCnt, m_CHeader.nMotorCnt);
                                // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
                                for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
                                {
                                    if (nAxis >= m_CHeader.nMotorCnt) nPos += 3;
                                    else if (nAxis >= SMotion.nMotorCnt) SMotion.STable[j].anMot[nAxis] = 0;// 실 모터수량과 맞지 않다면 그 부분을 0 으로 채울 것
                                    else
                                    {
                                        nData = byteData[nPos++];
                                        nData += byteData[nPos++] * 256;
                                        nData += byteData[nPos++] * 256 * 256;
                                        nData2 = nData & 0x3fff;
                                        if ((nData & 0x4000) != 0) nData2 *= -1; // 부호비트 검사

                                        // 엔코더 타입정의
                                        // 일단 넘어간다.

                                        // Stop Bit
                                        // 넘어간다.

                                        // Mode
#if false
                                        //Grid_SetFlag_Type(j, nAxis, (((nData & 0x20000) != 0) ? true : false));

                                        //Grid_SetFlag_Led(j, nAxis, ((nData >> 18) & 0x07));
                                        //Grid_SetFlag_En(j, nAxis, ((nData == 0x200000) ? false : true));

                                        if (m_CGridMotionEditor.GetEnable(j, nAxis) == false)
                                        {
                                            m_CGridMotionEditor.SetData(j, nAxis, 0);
                                        }
                                        else
#endif
                                        {
                                            SMotion.STable[j].anMot[nAxis] = nData2;// CalcEvd2Angle(nAxis, (int)nData2);
                                        }



                                        /* - Save
                                        fValue = GridMotionEditor_GetMotor(i, j);
                                        sData = (short)(OjwMotor.CalcAngle2Evd(j, fValue) & 0x03ff);
                                        //sData |= 0x0400; // 속도모드인때 정(0-0x0000), 역(1-0x0400)
                                        //sData |= LED;  // 00 - 0ff, 0x0800 - Red(01), 0x1000 - Blue(10), 0x1800 - Green(11)
                                        //sData |= 제어타입 // 0 - 위치, 0x2000 - 속도
                                        sData |= 0x4000; //Enable // 개별 Enable (0 - Disable, 0x4000 - Enable)
                                         */


                                        //fValue = GridMotionEditor_GetMotor(i, j);
                                        //nData = (int)(((Grid_GetFlag_En(i, j) == true) ? CalcAngle2Evd(j, fValue) : 0x07ff) & 0x0fff);

                                        //nData |= (int)(((j >= 6) && (j <= 8)) ? 0x8000 : 0x0000);
                                        //nData |= (int)((Grid_GetFlag_Type(i, j) == true) ? 0x20000 : 0x0000); // 제어타입 // 0 - 위치, 0x20000 - 속도

                                        //nData |= (int)((Grid_GetFlag_Led(i, j) & 0x07) << 18);
                                        //nData |= (int)((Grid_GetFlag_Type(i, j) == true) ? 0x8000 : 0x0000);
                                        //nData |= (int)((Grid_GetFlag_En(i, j) == false) ? 0x200000 : 0x00000);

                                        ////byteData = BitConverter.GetBytes((Int32)nData);
                                        ////fs.Write(byteData, 0, 3);
                                    }
                                }
                                #endregion Motor
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                // Speed  
                                SMotion.STable[j].nTime = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;

                                // Delay  
                                SMotion.STable[j].nDelay = BitConverter.ToInt16(byteData, nPos); nPos += 2;

                                // Group  
                                SMotion.STable[j].nGroup = (int)(byteData[nPos++]);

                                // Command  
                                SMotion.STable[j].nCmd = (int)(byteData[nPos++]);

                                // Data0  
                                SMotion.STable[j].nData0 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                // Data1  
                                SMotion.STable[j].nData1 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                SMotion.STable[j].nData2 = 0;
                                SMotion.STable[j].nData3 = 0;
                                SMotion.STable[j].nData4 = 0;
                                SMotion.STable[j].nData5 = 0;
                                #endregion Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region 추가한 Frame 위치 및 자세
                                nPos += 24;
                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;

                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;
                                #endregion 추가한 Frame 위치 및 자세
                            }
                            #endregion 실제 모션
#if true
                            string strData_ME = "";
                            string strData_FE = "";

                            // 'M' 'E'
                            strData_ME += (char)(byteData[nPos++]);
                            strData_ME += (char)(byteData[nPos++]);

                            #region Comment Data
                            // Comment
                            byte[] pstrComment = new byte[SMotion.nCommentSize];
                            for (j = 0; j < SMotion.nCommentSize; j++)
                                pstrComment[j] = (byte)(byteData[nPos++]);
                            m_strMotionFile_Comment = System.Text.Encoding.Default.GetString(pstrComment);
                            SMotion.strComment = m_strMotionFile_Comment;
                            pstrComment = null;
                            #endregion Comment Data

                            #region Caption
                            int nLineNum = 0;
                            string strLineComment;
                            byte[] byLine = new byte[46];
                            for (j = 0; j < SMotion.nCnt_LineComment; j++)
                            {
                                nLineNum = (short)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                for (int k = 0; k < 46; k++)
                                    byLine[k] = (byte)(byteData[nPos++]);
                                strLineComment = System.Text.Encoding.Default.GetString(byLine);
                                strLineComment = strLineComment.Trim((char)0);
                                m_CGridMotionEditor.SetCaption(nLineNum, strLineComment);
                            }
                            byLine = null;
                            #endregion Caption

                            // 'T' 'E'
                            strData_FE += (char)(byteData[nPos++]);
                            strData_FE += (char)(byteData[nPos++]);

                            //                     if (bMessage == true)
                            //                     {
                            //                         if (strData_ME != "ME") OjwMessage("Motion Table Error\r\n");
                            //                         else OjwMessage("Table Loaded");
                            //                         if (strData_FE != "TE") OjwMessage("File Error\r\n");
                            //                         else OjwMessage("Table Loaded");
                            //                     }
#endif
                            bFileOpened = true;
                            #endregion FileOpen V1.1
                        }
                        else if (strTmp.ToUpper() == _STR_EXT.ToUpper() + _STR_VER_V_12)
                        {
                            #region FileOpen V1.2
                            //int nPos = 6;   // 앞의 6개는 'DMT1.2' 에 할당

                            #region Header

                            #region 타이틀(21)
                            byte[] byteGetData = new byte[21];
                            for (i = 0; i < 21; i++) byteGetData[i] = 0;
                            for (i = 0; i < 21; i++)
                            {
                                if (byteData[i + nPos] == 0) break;
                                byteGetData[i] = byteData[i + nPos];
                            }
                            SMotion.strTableName = System.Text.Encoding.Default.GetString(byteGetData);
                            nPos += 21;
                            byteGetData = null;
                            #endregion 타이틀(21)

                            #region Start Position(1)
                            int nStartPosition = (int)(byteData[nPos++]);
                            SMotion.nStartPosition = (nStartPosition >= 0) ? nStartPosition : 0;
                            #endregion Start Position(1)

                            #region Size - MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1), Motor Index(MC), Mirror Index(MC)
                            // Size
                            SMotion.nFrameSize = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nCommentSize = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nCnt_LineComment = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nPlayTime = (int)(byteData[nPos] + byteData[nPos + 1] * 256 + byteData[nPos + 2] * 256 * 256 + byteData[nPos + 3] * 256 * 256 * 256); nPos += 4;
                            SMotion.nRobotModelNum = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                            SMotion.nMotorCnt = (int)(byteData[nPos++]);

                            // 모터의 인덱스
                            byte[] pbyteMotorIndex = new byte[SMotion.nMotorCnt];
                            for (int nIndex = 0; nIndex < SMotion.nMotorCnt; nIndex++) pbyteMotorIndex[nIndex] = byteData[nPos++];

                            // 모터의 Mirror 인덱스
                            byte[] pbyteMirrorIndex = new byte[SMotion.nMotorCnt];
                            for (int nIndex = 0; nIndex < SMotion.nMotorCnt; nIndex++) pbyteMirrorIndex[nIndex] = byteData[nPos++];

                            #endregion Size - MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1), Motor Index(MC), Mirror Index(MC)

                            #endregion Header

                            // nRobotModelNum 를 읽고 해당 파일을 읽어들인다.
                            #region Header 검증
#if false
                            if (SMotion.nMotorCnt != m_CHeader.nMotorCnt)
                            {
                                this.Cursor = System.Windows.Forms.Cursors.Default;
                                MessageBox.Show("디자이너 파일의 모터 수량과 맞지 않습니다.(요구모터수량=" + Ojw.CConvert.IntToStr(m_CHeader.nMotorCnt) + ", 모션파일에 정의된 모터수량=" + Ojw.CConvert.IntToStr(nMotorCnt) + ")\n");// 해당 모델에 맞는 모션을 로드하십시오.");
                                DialogResult dlgRet = MessageBox.Show("무시하고 계속 열겠습니까?", "파일열기", MessageBoxButtons.OKCancel, MessageBoxIcon.Warning);
                                if (dlgRet == DialogResult.OK)
                                {
                                    //MessageBox.Show("Yes");
                                    //return;
                                }
                                else return false;
                            }
#endif
                            #endregion Header 검증

                            for (i = SMotion.nFrameSize; i < m_CGridMotionEditor.GetLineCount() - SMotion.nFrameSize; i++) m_CGridMotionEditor.Clear(i);

                            #region 실제 모션
                            SMotion.STable = new SMotionTable_t[SMotion.nFrameSize];
                            for (j = 0; j < SMotion.nFrameSize; j++)
                            {
                                SMotion.STable[j].anMot = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].anLed = new int[SMotion.nMotorCnt];
                                SMotion.STable[j].abEn = new bool[SMotion.nMotorCnt];
                                SMotion.STable[j].abType = new bool[SMotion.nMotorCnt];
                            }

                            int nH = SMotion.nFrameSize;
                            int nData;
                            short sData;
                            //float fValue;
                            for (j = 0; j < nH; j++)
                            {
                                //En
                                #region Enable
                                int nEn = byteData[nPos++];
                                SMotion.STable[j].bEn = ((nEn & 0x01) != 0) ? true : false;
                                #endregion Enable
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                #region Motor
                                int nMotorCntMax = (int)Math.Max(SMotion.nMotorCnt, m_CHeader.nMotorCnt);
                                // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
                                for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
                                {
                                    if (nAxis >= m_CHeader.nMotorCnt) nPos += 3;
                                    else if (nAxis >= SMotion.nMotorCnt) SMotion.STable[j].anMot[nAxis] = 0;// 실 모터수량과 맞지 않다면 그 부분을 0 으로 채울 것
                                    else
                                    {
                                        nData = (int)(BitConverter.ToInt16(byteData, nPos)); nPos += 2;
                                        sData = (short)(nData & 0x3fff);
                                        if ((nData & 0x4000) != 0) sData -= 0x1000;
                                        // 엔코더 타입((0x8000) != 0)
                                        
                                        ///////////////////////////
                                        // Reserve(2), Noaction(1), LED(3-Red Blue Green), Mode(1), Stop Bit(1)
                                        int byteTmp = byteData[nPos++];
                                        ///////////////////////////

                                        //Grid_SetFlag_Led(j, nAxis, ((nData >> 12) & 0x07));
                                        //Grid_SetFlag_Type(j, nAxis, (((nData & 0x8000) != 0) ? true : false));
                                        //Grid_SetFlag_En(j, nAxis, ((sData == 0x7ff) ? false : true));

                                        if (sData == 0x7ff)
                                        {
                                            SMotion.STable[j].anMot[nAxis] = 0;
                                        }
                                        else
                                        {
                                            SMotion.STable[j].anMot[nAxis] = (int)sData;// CalcEvd2Angle(nAxis, (int)sData);
                                        }



                                        /* - Save
                                        fValue = GridMotionEditor_GetMotor(i, j);
                                        sData = (short)(OjwMotor.CalcAngle2Evd(j, fValue) & 0x03ff);
                                        //sData |= 0x0400; // 속도모드인때 정(0-0x0000), 역(1-0x0400)
                                        //sData |= LED;  // 00 - 0ff, 0x0800 - Red(01), 0x1000 - Blue(10), 0x1800 - Green(11)
                                        //sData |= 제어타입 // 0 - 위치, 0x2000 - 속도
                                        sData |= 0x4000; //Enable // 개별 Enable (0 - Disable, 0x4000 - Enable)
                                         */
                                    }
                                }
                                #endregion Motor
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                // Speed  
                                SMotion.STable[j].nTime = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;

                                // Delay  
                                SMotion.STable[j].nDelay = BitConverter.ToInt16(byteData, nPos); nPos += 2;

                                // Group  
                                SMotion.STable[j].nGroup = (int)(byteData[nPos++]);

                                // Command  
                                SMotion.STable[j].nCmd = (int)(byteData[nPos++]);

                                // Data0  
                                SMotion.STable[j].nData0 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                // Data1  
                                SMotion.STable[j].nData1 = (int)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                SMotion.STable[j].nData2 = 0;
                                SMotion.STable[j].nData3 = 0;
                                SMotion.STable[j].nData4 = 0;
                                SMotion.STable[j].nData5 = 0;
                                #endregion Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
                                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                #region 추가한 Frame 위치 및 자세
                                nPos += 24;
                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;

                                //nPos += 4;
                                //nPos += 4;
                                //nPos += 4;
                                #endregion 추가한 Frame 위치 및 자세
                            }
                            #endregion 실제 모션
#if true
                            string strData_ME = "";
                            string strData_FE = "";

                            // 'M' 'E'
                            strData_ME += (char)(byteData[nPos++]);
                            strData_ME += (char)(byteData[nPos++]);

                            #region Comment Data
                            // Comment
                            byte[] pstrComment = new byte[SMotion.nCommentSize];
                            for (j = 0; j < SMotion.nCommentSize; j++)
                                pstrComment[j] = (byte)(byteData[nPos++]);
                            m_strMotionFile_Comment = System.Text.Encoding.Default.GetString(pstrComment);
                            SMotion.strComment = m_strMotionFile_Comment;
                            pstrComment = null;
                            #endregion Comment Data

                            #region Caption
                            int nLineNum = 0;
                            string strLineComment;
                            byte[] byLine = new byte[46];
                            for (j = 0; j < SMotion.nCnt_LineComment; j++)
                            {
                                nLineNum = (short)(byteData[nPos] + byteData[nPos + 1] * 256); nPos += 2;
                                for (int k = 0; k < 46; k++)
                                    byLine[k] = (byte)(byteData[nPos++]);
                                strLineComment = System.Text.Encoding.Default.GetString(byLine);
                                strLineComment = strLineComment.Trim((char)0);
                                m_CGridMotionEditor.SetCaption(nLineNum, strLineComment);
                            }
                            byLine = null;
                            #endregion Caption

                            // 'T' 'E'
                            strData_FE += (char)(byteData[nPos++]);
                            strData_FE += (char)(byteData[nPos++]);
#endif

                            //                     if (bMessage == true)
                            //                     {
                            //                         if (strData_ME != "ME") OjwMessage("Motion Table Error\r\n");
                            //                         else OjwMessage("Table Loaded");
                            //                         if (strData_FE != "TE") OjwMessage("File Error\r\n");
                            //                         else OjwMessage("Table Loaded");
                            //                     }

                            pbyteMotorIndex = null;
                            pbyteMirrorIndex = null;
                            bFileOpened = true;
                            #endregion FileOpen V1.0
                        }
                        ////////////////////////////////////////////////////////////////////////////

                        if (bFileOpened == true)
                        {
                            #region Comment Data
                            // Comment
                            byte[] pstrComment = new byte[SMotion.nCommentSize];
                            for (j = 0; j < SMotion.nCommentSize; j++)
                                pstrComment[j] = (byte)(byteData[nPos++]);
                            m_strMotionFile_Comment = System.Text.Encoding.Default.GetString(pstrComment);
                            pstrComment = null;
                            #endregion Comment Data

                            return true;
                        }
                        return false;
                    }
                    catch
                    {
                        this.Cursor = System.Windows.Forms.Cursors.Default;
                        //if (bFile == true)
                        //{
                        //    fs.Close();
                        //    f = null;
                        //}
                        return false;
                    }
#endif
}

