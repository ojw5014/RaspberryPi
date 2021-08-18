/**
 * HerkuleX2
 * A library for Dongbu HerkuleX Servo
 *
 * Copyright 2014 Dongbu Robot
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

example )   sudo g++ -o testRun Test.cpp HerkuleX2.cpp HerkuleX2.h -lpthread -lwiringPi

 */
#ifndef __HERKULEX2
#define __HERKULEX2


// Jinwook On
//#include <time.h>	
#include <stdio.h>
#include <string.h>

#include <sys/time.h>
#include <unistd.h>
#include  <pthread.h>
typedef unsigned char byte;

extern int m_nTty;				//file description
//extern int m_anAxis_By_ID[_SIZE_MOTOR_MAX];

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
////////////////////
// add : Jinwook On 20160621
// get : Control Algorithm & Architecture from Open Jig Ware 

// Address
#define _ADDRESS_TORQUE_CONTROL  			52
#define _ADDRESS_LED_CONTROL  				53
#define _ADDRESS_VOLTAGE  					54
#define _ADDRESS_TEMPERATURE  				55
#define _ADDRESS_PRESENT_CONTROL_MODE  	56
#define _ADDRESS_TICK  						57
#define _ADDRESS_CALIBRATED_POSITION  	58

#define _SIZE_STRING						50

// Model
#define _MODEL_DRS_0101 1
#define _MODEL_DRS_0102 2
#define _MODEL_DRS_0201 3
#define _MODEL_DRS_0202 4
#define _MODEL_DRS_0401 5
#define _MODEL_DRS_0402 6
#define _MODEL_DRS_0601 7
#define _MODEL_DRS_0602 8
#define _MODEL_DRS_0603 9

#define _SIZE_MEMORY 256
#define _SIZE_MOTOR_MAX 254

#define _ID_BROADCASTING 254

//#define _FLAG_ENABLE 0x100


#define _HEADER1 				0
#define _HEADER2 				1
#define _SIZE 					2
#define _ID 						3
#define _CMD 					4
#define _CHECKSUM1 				5
#define _CHECKSUM2 				6
#define _SIZE_PACKET_HEADER 	7

#define _FLAG_STOP 				0x01
#define _FLAG_MODE_SPEED  		0x02
#define _FLAG_LED_GREEN  		0x04
#define _FLAG_LED_BLUE  		0x08
#define _FLAG_LED_RED  			0x10
#define _FLAG_NO_ACTION 		0x20

typedef struct 
{
	bool bEnable;
	int nID;
	int nAddress_First;
	int nAddress_Length;
}SRead_t;
typedef struct 
{
    int nID;

    int nDir;

    float fLimitUp;    // limit Max value - 0: No use
    float fLimitDn;    // limit Min value - 0: No use
    // Center position(Evd : Engineering value of degree)
    float fCenterPos;

    float fOffsetAngle_Display; // º¸¿©Áö´Â È­¸é»óÀÇ °¢µµ Offset

    // gear ratio
    float fMechMove;
    float fDegree;
}SParam_Axis_t;
typedef struct 
{
    bool bEn;

    int nDir;
    //Center
    float fCenterPos;

    float fMechMove;
    float fDegree;

    float fLimitUp;    // Limit - 0: Ignore
    float fLimitDn;    // Limit - 0: Ignore

    int nID;
    //float fPos;
    int nPos;
    float fTime;
    float fSpeed;

    int nFlag; // 76[543210] NoAction(5), Red(4), Blue(3), Green(2), Mode(    
}SMot_t;

typedef struct {
	bool 	bEn;
	int 		*pnMot;
	float 	*pfXyz; // reserve
	int 		*pnLed;
	bool 	*pbEn;
	bool 	*pbType;
	int 		nTime;
	int 		nDelay;
	int 		nGroup;
	int 		nCmd;
	int 		nData0;
	int 		nData1;
	int 		nData2;
	int 		nData3;
	int 		nData4;
	int 		nData5;

	//char		strCaption[256];
} SMotionTable_t;

typedef struct {
	char 	strVersion[6];
	int 		nFrameSize;
	int 		nCnt_LineComment;
	int 		nPlayTime;
	int 		nCommentSize;
	int 		nRobotModelNum;
	int 		nMotorCnt;
	int 		nStartPosition;
	char 	strFileName[21];
	char 	strTableName[21];
	char 	strComment[256];
	SMotionTable_t *pSTable;
} SMotion_t;

typedef struct 
{
	float pan;
	float tilt;
	float swing;
}SAngle3D_t;

typedef struct 
{
	float x;
	float y;
	float z;
}SVector3D_t;

typedef struct  			// Motor information
{
	int nMotorID;                    		// Motor ID
	int nMotorDir;                   		// Direction of Axis (0 - forward, 1 - inverse)
	float fLimit_Up;                 		// Max Angle(+)
	float fLimit_Down;               	// Min Angle(-)
	int nCenter_Evd;                 	// Pulse(Engineering value for 0 degree(Center Position))

	int nMechMove;                   	// Maximum Position ( Maximum Pulse value(Evd) )
	float fMechAngle;                	// It is a Degree when it moves in Maximum Position

	float fInitAngle;                		// Init position which you want it
	float fInitAngle2;               		// Init position which you want it(2'st)

	// Interference Axis(No use)
	int nInterference_Axis;          	// reserve(No use) - ÀÌ°Ô (-)°ªÀÌ¸é °£¼· Ãà ¾øÀ½.
	float fW;                        		// reserve(No use) - Side ¿¡ ºÙÀº ÃàÀÇ Å©±â(³ÐÀÌ)
	float fInterference_W;           	// reserve(No use) - °£¼·ÃàÀÌ ¾ÕµÚ·Î ºÙ¾ú´Ù°í °¡Á¤ÇÏ°í ÇØ´ç °£¼·ÃàÀÇ Å©±â(³ÐÀÌ)

	float fPos_Right;                		// reserve(No use) - ÃàÀÇ ¿À¸¥ÂÊ À§Ä¡
	float fPos_Left;                 		// reserve(No use) - ÃàÀÇ ¿ÞÂÊ À§Ä¡

	float fInterference_Pos_Front;// reserve(No use) - ÇØ´ç °£¼·ÃàÀÇ ¾ÕÂÊ À§Ä¡
	float fInterference_Pos_Rear;	// reserve(No use) - ÇØ´ç °£¼·ÃàÀÇ µÚÂÊ À§Ä¡

	// NickName
	char strNickName[_SIZE_STRING];              	// Nickname(32char)

	int nGroupNumber;                	// Group Number

	int nAxis_Mirror;                	// 0 ~ 253 : Motor ID of Mirroring one
	                                    		// -1      : there is no mirror motor.
	                                    		// -2 : there is no mirror motor(but it can has flip-direction(for using center), flip it from '0')

	int nMotorControlType;           	// Motor Control type => 0: Position, 1: Speed type
}SMotorInfo_t;

#if 0
class CDhParam
{
	 double dA;
	 double dD;
	 double dTheta;
	 double dAlpha;
	 int nAxisNum; // Motor Number(Similar with Virtual ID) : It means there is no motor when it use minus value(-)
	 int nAxisDir; // 0 - Forward, 1 - Inverse

	 CDhParam() // Constructor
	{
		dA = 0;
		dD = 0;
		dTheta = 0;
		nAxisNum = -1;
		nAxisDir = 0;
		strCaption = "";
	}

	 string strCaption;
	 void InitData()
	{
	    SetData(0, 0, 0, 0, -1, 0, "");
	}
	 void SetData(CDhParam OjwDhParam)
	{
	    SetData(OjwDhParam.dA, OjwDhParam.dD, OjwDhParam.dTheta, OjwDhParam.dAlpha, OjwDhParam.nAxisNum, OjwDhParam.nAxisDir, OjwDhParam.strCaption);
	}
	 void SetData(double dDh_a, double dDh_d, double dDh_theta, double dDh_alpha, int nDh_AxisNum, int nDh_AxisDir, string strDh_Caption)
	{
		dA = dDh_a;
		dD = dDh_d;
		dTheta = dDh_theta;
		dAlpha = dDh_alpha;
		nAxisNum = nDh_AxisNum;
		nAxisDir = nDh_AxisDir;
		strCaption = strDh_Caption;
	}
}
class CDhParamAll
{
	CDhParam* pSDhParam;
	int GetCount() { return (pSDhParam == NUKK) ? 0 : pSDhParam->Length; }
	int m_nAxis_X = 0;
	int m_nAxis_Y = 1;
	int m_nAxis_Z = 2;
	int[] m_pnDir = new int[3];
	CDhParamAll() // Constructor
	{
		m_pnDir[0] = 0;
		m_pnDir[1] = 0;
		m_pnDir[2] = 0;
	}
	void SetAxis_XYZ(int nX, int nX_Dir, int nY, int nY_Dir, int nZ, int nZ_Dir) // Define Motor Axis Number(Default 0, 1, 2)
	{
		if (((nX * nX + nY * nY + nZ * nZ) == 5) && (((nX + 1) * (nY + 1) * (nZ + 1)) == 6))
		{
			if (((nX_Dir >= 0) && (nX_Dir <= 1)) && ((nY_Dir >= 0) && (nY_Dir <= 1)) && ((nZ_Dir >= 0) && (nZ_Dir <= 1)))
			{
				m_nAxis_X = nX; m_nAxis_Y = nY; m_nAxis_Z = nZ;
				m_pnDir[m_nAxis_X] = nX_Dir;
				m_pnDir[m_nAxis_Y] = nY_Dir;
				m_pnDir[m_nAxis_Z] = nZ_Dir;
			}
		}
	}
	void GetAxis_XYZ(out int nX, out int nX_Dir, out int nY, out int nY_Dir, out int nZ, out int nZ_Dir)
	{
		nX = m_nAxis_X; nY = m_nAxis_Y; nZ = m_nAxis_Z;
		nX_Dir = m_pnDir[m_nAxis_X];
		nY_Dir = m_pnDir[m_nAxis_Y];
		nZ_Dir = m_pnDir[m_nAxis_Z];
	}
	CDhParam GetData(int nIndex)
	{
		if ((nIndex >= pSDhParam.Length) || (nIndex < 0)) return null;
		else return pSDhParam[nIndex];
	}
	bool SetData(int nIndex, CDhParam OjwDhParam)
	{
		if ((nIndex >= pSDhParam.Length) || (nIndex < 0)) return false;

		pSDhParam[nIndex].SetData(OjwDhParam);
		return true;
	}
	bool AddData(CDhParam OjwDhParam)
	{
		int nCnt = (pSDhParam == null) ? 1 : pSDhParam.Length + 1;
		Array.Resize(ref pSDhParam, nCnt);
		pSDhParam[nCnt - 1] = new CDhParam();
		pSDhParam[nCnt - 1].SetData(OjwDhParam);

		return true;
	}
	void DeleteAll()
	{
		if (pSDhParam != null)
		{
			for (int i = 0; i < pSDhParam.Length; i++)
			    pSDhParam[i] = null;
			Array.Resize(ref pSDhParam, 0);
		}
	}
}
class COjwDesignerHeader
{
	int nVersion;
	char strVersion[6];

	int nDefaultFunctionNumber = -1;

	int nModelNum = 0;                               // The name of the actual model with the at least 1(Kor: 1 ÀÌ»óÀÇ °ªÀ» °¡Áö´Â ½ÇÁ¦ÀûÀÎ ¸ðµ¨ÀÇ ÀÌ¸§)
	char strModelNum[256];               // 
	char strModelName[256];

	SAngle3D_t SInitAngle;// = new SAngle3D_t();        // The default angle facing the screen(Kor: È­¸éÀ» ¹Ù¶óº¸´Â ±âº» °¢µµ)
	SVector3D_t SInitPos;// = new SVector3D_t();        // The default position of the object present in the screen(Kor: È­¸é³»¿¡ Á¸ÀçÇÏ´Â ¿ÀºêÁ§Æ®ÀÇ ±âº» À§Ä¡)

	float fInitScale = 0.35f;                        // Size ratio in the initial screen(Kor: ÃÊ±â È­¸éÀÇ Å©±â ºñÀ²)

	bool bDisplay_Light = true;                      // To use the light(Kor: ºûÀ» »ç¿ëÇÒ °ÍÀÎÁö...)
	bool bDisplay_Invisible = false;                 // Transparent material, regardless of whether the ball(Kor: ÀçÁú°ú »ó°ü¾øÀÌ Åõ¸íÇÏ°Ô º¼ °ÍÀÎÁö...)
	bool bDisplay_Axis = false;                      // Look what the reference axis(Kor: ±âÁØÃàÀ» º¸ÀÏ °ÇÁö)

	long cBackColor;// = Color.FromArgb(-5658199);     // backgroud color(Kor: ¹è°æ »ö)

	SMotorInfo_t aSMotorInfo[256];          // This limit is necessary because it reflects axes up to 256 axes(Kor: ¸®¹ÌÆ®°¡ ÇÊ¿äÇÑ ÃàÀº ÃÖ´ë 256Ãà ÀÌ¹Ç·Î ÀÌ¸¦ ¹Ý¿µ)

	char astrGroupName[256];// = new string[512];          // Group name(Kor: ÁöÁ¤ÇÑ ±×·ìÀÇ ÀÌ¸§)
	CDhParamAll pDhParamAll[512];// = new CDhParamAll[512]; // (0~255 Group)DH Param
	int* pnSecret = new int[512];                   // 0: Normal, 1: Secret Letter
	int* pnType = new int[512];                     // 0: Normal, 1: Wheel Control Type
	bool* pbPython = new bool[512];                   // false: Normal, true: Python code
	string* pstrKinematics = new string[512];
	SEncryption_t* pSEncryptKinematics_encryption = new SEncryption_t[512];
	string* pstrInverseKinematics = new string[512];
	SEncryption_t* pSEncryptInverseKinematics_encryption = new SEncryption_t[512];

	SOjwCode_t* pSOjwCode = new SOjwCode_t[512];

	string strDrawModel;                                 // String that contains the actual data model(Kor: ½ÇÁ¦ ¸ðµ¨ µ¥ÀÌÅ¸°¡ µé¾îÀÖ´Â ½ºÆ®¸µ)
	// The number of motors in internal (However, be sure to order 0,1,2, ... must be created in order)
	// Kor: ³»ºÎ¿¡ µé¾îÀÖ´Â ¸ðÅÍÀÇ °¹¼ö ( ´Ü, ¹Ýµå½Ã ¼ø¼­´ë·Î 0,1,2,... ¼øÀ¸·Î ÀÛ¼ºÇØ¾ß ÇÑ´Ù. )
	int nMotorCnt;

	string strComment;                               // comment(Kor: ºÎ°¡¼³¸í)

	int nWheelCounter_2 = 0;                         // The number of 2-wheel wheels(Kor: 2·û µð¹ÙÀÌ½ºÀÇ °³¼ö)
	int nWheelCounter_3 = 0;                         // The number of 3-wheel wheels(Kor: 3·û µð¹ÙÀÌ½ºÀÇ °³¼ö)
	int nWheelCounter_4 = 0;                         // The number of 4-wheel wheels(Kor: 4·û µð¹ÙÀÌ½ºÀÇ °³¼ö)
/*
	COjwDesignerHeader() // »ý¼ºÀÚ
	{
		SInitAngle.pan = -10.0f;
		SInitAngle.tilt = 10.0f;
		SInitAngle.swing = 0.0f;

		SInitPos.x = 0.0f;
		SInitPos.y = 0.0f;
		SInitPos.z = 0.0f;

		pSMotorInfo.Initialize();
		for (int i = 0; i < 256; i++)
		{
			pSMotorInfo[i].nInterference_Axis = -1;

			// Alloc memory(Kor: ¸Þ¸ð¸® È®º¸)
			pDhParamAll[i] = new CDhParamAll();
			pDhParamAll[i].DeleteAll();

			pSMotorInfo[i].strNickName = "";

			pstrGroupName[i] = "";
			pstrKinematics[i] = "";
			//pbyteKinematics_encryption
			pstrInverseKinematics[i] = "";
			//pbyteInverseKinematics_encryption

			pSMotorInfo[i].nMotorID = i;
			pSMotorInfo[i].fMechAngle = 330.0f;
			pSMotorInfo[i].nMechMove = 1024;
			pSMotorInfo[i].nCenter_Evd = 512;


			pSEncryptKinematics_encryption[i].byteEncryption = new byte[0];
			pSEncryptInverseKinematics_encryption[i].byteEncryption = new byte[0];
		}
		for (int i = 0; i < 256; i++)
		{
			pstrGroupName[256 + i] = "";
			pstrKinematics[256 + i] = "";
			pstrInverseKinematics[256 + i] = "";
			pSEncryptKinematics_encryption[256 + i].byteEncryption = new byte[0];
			pSEncryptInverseKinematics_encryption[256 + i].byteEncryption = new byte[0];
		}
		strDrawModel = "";
		nMotorCnt = 0;
		nDefaultFunctionNumber = -1; // No Choice
	}*/
}

#endif
class CMotor 
{
public:
	CMotor();									// Initialize
	~CMotor();		 							// Destroy

	static void Open_Socket();//(const char *pcIp);//, int nPort);
	static void Close_Socket();
	void Open(const char *pcDevice, int nBaudrate);//, int nModel);		//serial port open
	void Close();								//serial port close

	void SetParam(int nAxis, int nRealID, int nDir, float fLimitUp, float fLimitDn, float fCenterPos, float fOffsetAngle_Display, float fMechMove, float fDegree);
	void SetParam(int nAxis, int nModel);
	void SetParam_RealID(int nAxis, int nRealID);		
	void SetParam_Dir(int nAxis, int nDir);		
	void SetParam_LimitUp(int nAxis, float fLimitUp);		
	void SetParam_LimitDown(int nAxis, float fLimitDn);		
	void SetParam_CenterEvdValue(int nAxis, float fCenterPos);		
	void SetParam_Display(int nAxis, float fOffsetAngle_Display);		
	void SetParam_MechMove(int nAxis, float fMechMove);		
	void SetParam_Degree(int nAxis, float fDegree);	

	bool SetParam_with_File(const char *strHeaderFile);

	static bool IsOpen_Socket();
	static bool IsOpen() { return ((m_nTty != 0) ? true : false); }
	bool IsStop() { return m_bStop; }
	bool IsEms() { return m_bEms; }

	void Stop(int nAxis);
	void Stop();
	void Ems();
	
	bool GetErrorCode(int nAxis); // Status 1
	bool IsError(int nAxis);
	bool IsWarning(int nAxis);
	////////////////////////////////////
	// Motor Control - Reset
	void Reboot();
	void Reboot(int nAxis);
	void Reset();
	void Reset(int nAxis);
	//void Reset_ErrorFlag();
	//void Reset_ErrorFlag(int nAxis);

	bool m_bIgnoredLimit;
	void SetLimitEn(bool bOn) { m_bIgnoredLimit = !bOn; }
	bool GetLimitEn() { return !m_bIgnoredLimit; }
	int Clip(int nLimitValue_Up, int nLimitValue_Dn, int nData);
	float Clip(float fLimitValue_Up, float fLimitValue_Dn, float fData);

	int CalcLimit_Evd(int nAxis, int nValue);	
	float CalcLimit_Angle(int nAxis, float fValue);
	
	int CalcTime_ms(int nTime);
	int CalcAngle2Evd(int nAxis, float fValue);
	float CalcEvd2Angle(int nAxis, int nValue);

	////////////////////////////////////
	// Motor Control - Torq On / Off
	void SetTorque(int nAxis, bool bDrvOn, bool bSrvOn); 	//torque on / Off
	void SetTorque(bool bDrvOn, bool bSrvOn);		//torque on / Off

	// baudrate
	void SetBaudrate(int nAxis, int nBaud); 	
	void SetMotorID(int nAxis, int nNewID);
	
	/////////////////////////////////////
	// Data Command(No motion) - just setting datas   		=> use with Send_Motor
	// ---- Position Control ----
	void Set(int nAxis, int nEvd);
	int 	Get(int nAxis);

	void Set_Angle(int nAxis, float fAngle);
	float Get_Angle(int nAxis);

	// ---- Speed Control ----
	void Set_Turn(int nAxis, int nEvd);
	int 	Get_Turn(int nAxis);

	int Get_Pos_Evd(int nAxis);
	float Get_Pos_Angle(int nAxis);
	/////////////////////////////////////
	// Led Control   									=> use with Send_Motor
	//void Set_Flag(int nAxis, int nFlag);
	void Clear_Flag();
	void Clear_Flag(int nAxis);
	void Set_Flag(int nAxis, bool bStop, bool bMode_Speed, bool bLed_Green, bool bLed_Blue, bool bLed_Red, bool bNoAction);
	void Set_Flag_Stop(int nAxis, bool bStop);
	void Set_Flag_Mode(int nAxis, bool bMode_Speed);
	void Set_Flag_Led(int nAxis, bool bGreen, bool bBlue, bool bRed);
	void Set_Flag_Led_Green(int nAxis, bool bGreen);
	void Set_Flag_Led_Blue(int nAxis, bool bBlue);
	void Set_Flag_Led_Red(int nAxis, bool bRed);
	void Set_Flag_NoAction(int nAxis, bool bNoAction);

	// 1111 1101
	int 	Get_Flag(int nAxis) { return m_aSMot[nAxis].nFlag; }
	int	Get_Flag_Mode(int nAxis);

	bool Get_Flag_Led_Green(int nAxis);
	bool Get_Flag_Led_Blue(int nAxis);
	bool Get_Flag_Led_Red(int nAxis);

	void TimerSet();			// Timer »ý¼º
	void TimerDestroy();		// Timer Destroy ( »ý¼ºÀ» ÇßÀ¸¸é ¹Ýµå½Ã Destroy¸¦ ÇÏµµ·Ï ÇÑ´Ù. )
	unsigned long Timer();				// Timer »ý¼º ÈÄ ÇöÀç±îÁöÀÇ ½Ã°£ °ªÀ» return

	#define _TIME_MUL	1000
	class CTimer
	{
		public:
			CTimer() { m_bTimer = false; m_ulTimer; }
			~CTimer() {}
			void Set() { m_bTimer = true; gettimeofday(&m_tvTemp, NULL ); m_ulTimer = (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL); }
			void Destroy() { m_bTimer = false; m_ulTimer; }
			unsigned long Get() { if (m_bTimer == true) { gettimeofday(&m_tvTemp, NULL );	 return (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL) - m_ulTimer; } else return 0; }
		private:
			bool m_bTimer;
			unsigned long m_ulTimer;
			struct timeval m_tvTemp;
	};
	//////////////////////////////////////
	// Motor Control - Move Motor(Action)
	void Send_Led();
	void Send_Motor(int nMillisecond);

	//////////////////////////////////////
	// Wait
	void Wait_Ready();
	bool Wait_Motor(int nMilliseconds);
	bool Wait_Motor();
	bool Wait_Position(int nAxis, float fAngle, int nErrorTime);
	bool Wait_Delay(int nMilliseconds);


	CTimer m_CTmr_Motion;
	long	m_lWaitActionTimer;	// ±âÁØ½Ã°£ Å¸ÀÌ¸Ó
	char WaitAction_SetTimer();	// Çö ½ÃÁ¡À» Áß½ÉÀ¸·Î Å¸ÀÌ¸Ó¸¦ ÃÊ±âÈ­
	char WaitAction_ByTimer(long t);	// WaitAction_SetTimer ÇÑ ½Ã°£ºÎÅÍ ÁöÁ¤½Ã°£ÀÌ ³Ñ¾ú´ÂÁö¸¦ Ã¼Å©. ³Ñ±â ±îÁö Wait

//#define _ADDRESS_TORQUE_CONTROL 			52
//#define _ADDRESS_LED_CONTROL 				53
//#define _ADDRESS_VOLTAGE 					54
//#define _ADDRESS_TEMPERATURE 				55
//#define _ADDRESS_PRESENT_CONTROL_MODE 	56
//#define _ADDRESS_TICK 						57
//#define _ADDRESS_CALIBRATED_POSITION		58

	void Read_Ram(int nAxis, int nAddress, int nLength);
	//void Read_Rom();
	void Read_Motor(int nAxis);
	void Read_Motor();
//	void Read_Motor(int nAxis);

	// Push Motor ID for checking(if you set a Motor ID with this function, you can get a feedback data with delay function)
	void Read_Motor_Push(int nAxis);
	// You can check your Motor ID for feedback, which you set or not.
	int Read_Motor_Index(int nAxis);
	bool Read_Motor_IsReceived();
	void Sync_Seq();

	// use this when you don't want to get some motor datas.
	void Read_Motor_Clear();

	// detail option
	void Read_Motor_Change_Address(int nAxis, int nAddress, int nLength);
	
	void Read_Motor_ShowMessage(bool bTrue);// { m_bShowMessage = bTrue; }
	
	//////////////////////////////////////
	// Setting
	//void Set_Ram(int nId, 
	//void Set_Rom(int nId, 
	
	//bool m_bProgEnd;	

	//void FileOpen(const char *strFileName, COjwDesignerHeader *pCHeader);
	void Motion_Play(const char *strFileName);
	void PlayFrame(SMotionTable_t STable);
	static void Write(byte *buffer, int nLength) { SendPacket(buffer, nLength); }
private:
	
	SMotion_t m_SMotion;
	SMotorInfo_t m_aSMotorInfo[256];
	//unsigned char getChksum1(class DataPacket * buf);	//Check sum1 ì„¤ì • 
	//unsigned char getChksum2(unsigned char chksum1);	//Check sum2 ì„¤ì •	

	//void sendPacket(class DataPacket * buf);			//Packet ë³´ë‚´ê¸°
	//int receivePacket();
	void Init();
	//static bool m_bShowMessage;
	//int m_nSeq_Motor;
	//int m_nSeq_Motor_Back;
	unsigned long m_ulDelay;
	
	CTimer m_CTmr;

//#define _TIMER_ARRAY
#ifdef _TIMER_ARRAY
	CTimer m_aCTmr_TimeOut[_SIZE_MOTOR_MAX];
#else
	CTimer m_CTmr_Timeout;
#endif
	pthread_t m_thReceive;
	static void* Thread_Receive(void* arg);
	static void* Thread_Socket(void* arg);
	
	static void SendPacket_Socket(byte *buffer, int nLength);
	
	static void SendPacket(byte *buffer, int nLength);
	void Make_And_Send_Packet(int nID, int nCmd, int nDataByteSize, byte* pbytePacket);
	void MakeCheckSum(int nAllPacketLength, byte* buffer);

	int m_nModel;

	int m_nTimeout;
	
	SRead_t m_aSRead[_SIZE_MOTOR_MAX];
	int m_nReadCnt;
	int m_nMotorCnt;
	int m_nMotorCnt_Back;
	SParam_Axis_t m_aSParam_Axis[256];
	SMot_t m_aSMot[_SIZE_MOTOR_MAX];
	SMot_t m_aSMot_Prev[_SIZE_MOTOR_MAX];
	
	char m_acEn[_SIZE_MOTOR_MAX];
	//char m_acEn_For_Feedback[_SIZE_MOTOR_MAX];

	//bool m_bOpen;
	bool m_bStop;
	bool m_bEms;
	bool m_bStart;
	
	void Push_Id(int nAxis);
	int Pop_Id();
	bool IsCmd(int nAxis);
	int GetID_By_Axis(int nAxis) { return (nAxis == 0xfe) ? 0xfe : m_aSMot[nAxis].nID; }

	bool BinaryFileOpen(const char *strFileName, SMotion_t *pSMotion);
};

#endif
