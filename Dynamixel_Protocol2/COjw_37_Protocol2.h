/**
 * @author      Jinwook On ojw5014@hanmail.net
 * @modified    2020.09.--
 * @version     01.00.00 Released

example )   sudo g++ -o testRun test.cpp ojw_dxl.cpp ojw_dxl.h -lpthread -lwiringpi

 */
#ifndef __OJW_PROTOCOL2
#define __OJW_PROTOCOL2

#include <stdio.h>
#include <string>

#include <sys/time.h>
#include <unistd.h>
#include  <pthread.h>

#include <sys/timeb.h>

//#define _CMD_LIST

#ifdef _CMD_LIST

#include <list>
using namespace std;

#else

#endif

typedef unsigned char byte;

extern int m_nTty;				//file description

// #define _MAX_FRAME 50
// class CFrame_t{
// public:
//     bool bEn = false;
//     int nCnt_Motor = 0;
//     int anID[_MAX_FRAME];
//     int anVal[_MAX_FRAME];
//     float fTime = 1000.0f;
//     float fDelay = 0.0f;
//     CFrame_t(int id, int nRawData) 
//     {
//     }
//     ~CFrame_t() 
//     {

//     }
//     void SetEnable(bool bEnable) { bEn = bEnable; }
//     void Set(int nID, int nEvd) { 
//         if (id == 0); 
//         anVal[nID] = nData; 
//     }
// };

class CCommand_t{
public:
    int nID = -1;
    float fVal = -2;
    CCommand_t()
    {

    }
    CCommand_t(int id, float val) 
    { 
        nID = id; 
        fVal = val; 
    }
    ~CCommand_t() 
    {

    }
};

class CProtocol2
{
public:
	CProtocol2();									// Initialize
	~CProtocol2();		 							// Destroy
    
    float m_afMot[256];
	float m_afMot_Pose[256];

 	// void Init();
    bool    IsOpen() { return ((m_nTty != 0) ? true : false); }
    bool    Open(const char  *pcDevice, int nBaudrate);
    void    Close();

    // void SetParam(int nID, bool bDirReverse = false);
    void    SetParam(int nID, bool bDirReverse = false, float fMulti = 1.0f, bool bSetDynamixelPro = false);
    
    void    Command_Clear();
    void    Command_Set(int nID, float fValue);
    void    Command_Set_Rpm(int nID, float fRpm);

    float   CalcEvd2Angle(int nID, int nValue);
    int     CalcAngle2Evd(int nID, float fValue);
    int     CalcPosition_Time(int nAxis, int nTime, int nDelay, float fAngle);
    float   CalcRaw2Rpm(int nID, int nValue);
    int     CalcRpm2Raw(int nID, float fRpm);
    float   CalcTime2Rpm(float fDeltaAngle, float fTime);

    //////////////////////////////////////////////////////
    // Protocol - basic(updateCRC, MakeStuff, SendPacket)
    //////////////////////////////////////////////////////
    void WaitSend();
    int updateCRC(byte *data_blk_ptr, int data_blk_size);
    int MakeStuff(byte *pBuff, int nLength);
    void SendPacket(byte *buffer, int nLength);
    //void Send(int nMotorRealID, int nCommand, int nAddress, const byte *pbyDatas, ...);
    void Send(int nMotorRealID, int nCommand, int nAddress, const byte *pbyDatas, int nDataLength);
    ////////////////////////////////////////////////////// => Protocol
    
    //////////////////////////////////////////////////////
    // Sync Write
    //////////////////////////////////////////////////////
    void Sync_Clear();
    void Sync_Push_Byte(int nID, int nData);
    void Sync_Push_Word(int nID, int nData);
    void Sync_Push_Dword(int nID, int nData);
    void Sync_Push_Angle(int nID, float fAngle);
    void Sync_Push(int nID, byte *pbyDatas, int nDataLength);
    void Sync_Flush(int nAddress);
    ////////////////////////////////////////////////////// => Sync Write
    
    //////////////////////////////////////////////////////
    // Read
    //////////////////////////////////////////////////////
    void ShowPacketReturn(int nPacket_0_Disable_1_Enable);
    bool WaitReceive();
    void ReceivedPacket(byte *buffer, int nBufferSize);
    void ShowError(int nErrorNumber);
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
    ////////////////////////////////////////////////////// => Read

    //////////////////////////////////////////////////////
    // Read Command
    //////////////////////////////////////////////////////
    void SyncRead_With_Address(int nAddress, int nSize, int *anIDs);
    void SyncRead(int nMotor_Cnt, int *anIDs);
    void Request(int nMotor, int nCommand, int nData_Size, byte *pbyDatas);

    void Request_Push(int nMotor);
    void Request_Clear();
    void Request_Flush(int nAddress, int nSize);
    void Request_with_RealID(int nMotorRealID, int nCommand, int nData_Size, byte *pbyDatas);
    ////////////////////////////////////////////////////// => Read Command

    //////////////////////////////////////////////////////
    // Command
    //////////////////////////////////////////////////////
    #if 0
    void FThread_Run();
    void PlayNext() { m_bNext = true; }
    void Play(string strFileName);
    #endif
    void SetTorq();
    
    bool IsEms();
    void Reset();

    void Wheel(float fRpm);
    // 마지막 모션이 아니라면 bContinue = false
    // 
    #ifdef _CMD_LIST
    void Move(int nTime_ms, int nDelay, bool bContinue, list<CCommand_t> aCCommands);
    void Move_NoWait(int nTime_ms, int nDelay, list<CCommand_t> aCCommands);
    // CCommand_t *ListCmdToArray(list<CCommand_t> lst);
    #else
    void Move(int nTime_ms, int nDelay, bool bContinue, int nCommandSize, CCommand_t *aCCommands);
    #endif
    void Move(int nTime_ms, int nDelay, bool bContinue);
    void Move(int nTime_ms, int nDelay);

    void Move_NoWait(int nTime_ms, int nDelay);
    void Move_NoWait(int nTime_ms, int nDelay, int nCommandSize, CCommand_t *aCCommands);

    void SetSpeed();
    void SetPosition_Speed();
    void SetPosition();
 
    bool Play(const char *strMotionFile);
    bool PlayFrameString(char *buff);
    bool PlayFrameString(char *buff, bool bNoWait);


    //
    // void SetTest(CCommand_t aCCmds, ...);
    ////////////////////////////////////////////////////// => Command

private:
};

class CParam_t
{
public:
    bool m_bModel_High = false;
    int m_nSet_Torq_Address = 64;
    int m_nSet_Torq_Size = 1;
    int m_nSet_Led_Address = 65;
    int m_nSet_Led_Size = 1;
    int m_nSet_Position_Speed_Address = 112;
    int m_nSet_Position_Speed_Size = 4;
    int m_nSet_Position_Address = 116;
    int m_nSet_Position_Size = 4;
    int m_nSet_Speed_Address = 104;
    int m_nSet_Speed_Size = 4;

    float m_fMechMove = 4096.0f;    // PH54-100 => -501,923 ~ 501,923, H54-200 => -501,923 ~ 501,923
    float m_fCenter = 2048.0f;
    float m_fMechAngle = 360;
    float m_fJointRpm = 0.229f; // ph54-100 = 0.01, H54-200 = 0.01
    bool m_bDirReverse = false;
    float m_fMulti = 1.0f;

    int m_nGet_Position_Address = 132;
    int m_nGet_Position_Size = 4;

    void SetParam_Address_Torq(int nVal = 64)                { m_nSet_Torq_Address = nVal; }
    void SetParam_Address_Size_Torq(int nVal = 1)            { m_nSet_Torq_Size = nVal; }
    void SetParam_Address_Led(int nVal = 66)                 { m_nSet_Led_Address = nVal; }
    void SetParam_Address_Size_Led(int nVal = 1)             { m_nSet_Led_Size = nVal; }
    void SetParam_Address_PositionSpeed(int nVal = 112)      { m_nSet_Position_Speed_Address = nVal; }
    void SetParam_Address_Size_PositionSpeed(int nVal = 4)   { m_nSet_Position_Speed_Size = nVal; }
    void SetParam_Address_Position(int nVal = 116)           { m_nSet_Position_Address = nVal; }
    void SetParam_Address_Size_Position(int nVal = 4)        { m_nSet_Position_Size = nVal; }
    void SetParam_Address_GetPosition(int nVal = 132)        { m_nGet_Position_Address = nVal; }
    void SetParam_Address_Size_GetPosition(int nVal = 4)     { m_nGet_Position_Size = nVal; }
    void SetParam_Address_Speed(int nVal = 104)              { m_nSet_Position_Speed_Address = nVal; }
    void SetParam_Address_Size_Speed(int nVal = 4)           { m_nSet_Position_Speed_Size = nVal; }

    void SetParam_Dir(bool bReverse = false) { m_bDirReverse = bReverse; }
    void SetParam_Multi(float fMulti = 1.0f) { m_fMulti = fMulti; if (fMulti == 0) m_fMulti = 1.0f; }

    void SetParam(bool bSetDynamixelPro = false)
    {
        //printf("SetParam()");
        if (bSetDynamixelPro == true) // PH, H54(Pro) ... 
        {
            m_bModel_High = true;
            m_nSet_Torq_Address = 512;
            m_nSet_Torq_Size = 1;
            m_nSet_Led_Address = 513;
            m_nSet_Led_Size = 1;
            m_nSet_Speed_Address = 552;
            m_nSet_Speed_Size = 4;
            m_nSet_Position_Speed_Address = 560;
            m_nSet_Position_Speed_Size = 4;
            m_nSet_Position_Address = 564;
            m_nSet_Position_Size = 4;

            m_fMechMove = 1003846.0f;    // PH54-100 => -501,923 ~ 501,923, H54-200 => -501,923 ~ 501,923
            m_fCenter = 0.0f;
            m_fMechAngle = 360;
            m_fJointRpm = 0.01f; // ph54-100 = 0.01, H54-200 = 0.01
            m_bDirReverse = false;
            m_fMulti = 1.0f;
            m_nGet_Position_Address = 580;
            m_nGet_Position_Size = 4;
        }
        else
        {
            m_bModel_High = false;
            m_nSet_Torq_Address = 64;
            m_nSet_Torq_Size = 1;
            m_nSet_Led_Address = 65;
            m_nSet_Led_Size = 1;
            m_nSet_Speed_Address = 104;
            m_nSet_Speed_Size = 4;
            m_nSet_Position_Speed_Address = 112;
            m_nSet_Position_Speed_Size = 4;
            m_nSet_Position_Address = 116;
            m_nSet_Position_Size = 4;

            m_fMechMove = 4096.0f;    // PH54-100 => -501,923 ~ 501,923, H54-200 => -501,923 ~ 501,923
            m_fCenter = 2048.0f;
            m_fMechAngle = 360;
            m_fJointRpm = 0.229f; // ph54-100 = 0.01, H54-200 = 0.01
            m_bDirReverse = false;
            m_fMulti = 1.0f;
            m_nGet_Position_Address = 132;
            m_nGet_Position_Size = 4;
        }
    }
};
#define Round(dValue)				(((double)dValue > 0)?floor((double)dValue + 0.5):ceil((double)dValue - 0.5))
#define Roundf(fValue)				(((float)fValue > 0)?floor((float)fValue + 0.5f):ceil((float)fValue - 0.5f))
#define _TIME_MUL	1000
class CTimer_t{
public:
	struct timeval m_tvTemp;
	unsigned long tmr;
	bool IsTimer = true;
	void start() { gettimeofday(&m_tvTemp, NULL ); tmr = (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL); IsTimer = true; }
	void stop() { IsTimer = false; }
	unsigned long get()
	{
		if (IsTimer)
		{
			gettimeofday(&m_tvTemp, NULL );
			return (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL) - tmr;
		}
		return 0;
	}
	unsigned long now()
	{
		gettimeofday(&m_tvTemp, NULL );	
		return (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL);
	}
  int get_weekday() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return tmrData->tm_wday; }
  int get_year() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return (tmrData->tm_year + 1900); }
  int get_month() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return (tmrData->tm_mon + 1); }
  int get_day() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return tmrData->tm_mday; }
  int get_hour() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return tmrData->tm_hour; }
  int get_minute() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return tmrData->tm_min; }
  int get_second() { time_t tmrTime = time( NULL ); struct tm* tmrData = localtime( &tmrTime ); return tmrData->tm_sec; }
  int get_millisecond() { struct timeb time; ftime(&time);localtime( &time.time ); return time.millitm; }
};
#define BytesToShort(buf, nStartIndex) ((buf[nStartIndex + 1] << 8) | buf[nStartIndex + 0])
#define BytesToInt(buf, nStartIndex) ((buf[nStartIndex + 3] << 24) | (buf[nStartIndex + 2] << 16) | (buf[nStartIndex + 1] << 8) | buf[nStartIndex + 0])

#endif // __OJW_PROTOCOL2
