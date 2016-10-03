// compile : 
//     sudo g++ -o test Test.cpp HerkuleX2.cpp HerkuleX2.h -lpthread -lwiringPi
// Run :
//     sudo ./test

// Y 축만 움직일 경우(11 turn, 30000 ms)
// sudo ./test 11 30000 2
// X 축만 움직일 경우(11 turn, 30000 ms)
// sudo ./test 11 30000 1
// 전체축이 움직일 경우(11 turn, 30000 ms)
// sudo ./test 11 30000 0
// sudo ./test 11 30000

// ID 변경
// sudo ./test --id 0 8				-> motor 0 -> motor 8
// sudo ./test --baud 0 115200     	-> motor 0 ,  115200 bps

//#define _DRS_0603


#include "stdio.h"
#include <stdlib.h>  // atoi
#include <string.h> // strncasecmp
#include <signal.h>
#include <math.h>
#include <pthread.h>
	
//#include <iostream>
#include "HerkuleX2.h"
//#include <unistd.h>

#include <wiringPi.h>

#define _SW_ORG_X	17
#define _SW_ORG_Y	18
#define _SW_EMS		22
#define _SW_RUN		23


#define _Y 0
#define _X 1
//using namespace std;

//#define _MOTOR_219 219
//#define _MOTOR_0 0
//void sig_int(int signo);
CMotor motor;
bool m_bAgingEnd = false;
bool m_bStart = false;
bool m_bEms = false;
bool m_bOrgMoving = false;
int m_nOrg = 0;
int m_nAging = 0;

bool m_bCheckSensor = false;

pthread_t m_thRun;
static void* Thread_Receive(void* arg);

void sig_int(int signo)
{
	sigset_t sigset, oldset;
	sigfillset(&sigset);

	if(sigprocmask(SIG_BLOCK, &sigset, &oldset) < 0 ) 
		printf("sigprocmask %d error\n",signo);
	if (motor.IsOpen() == true)
	{
		motor.Stop();
		if (m_nAging == 0) motor.Close();
	}
	
	m_bAgingEnd = true;
	m_bCheckSensor = false;
	
	return;
}

void* Thread_Input(void* arg)
{
	int nSw_Org_X = 0;
	int nSw_Org_Y = 0;
	int nSw_Ems = 0;
	int nSw_Run = 0;
	int nEms = 0;
	while(motor.IsOpen() == true)
	{
#if 1
		if (m_bCheckSensor == false)
		{
			if ((digitalRead(_SW_EMS) == 0) || ((m_bOrgMoving == false) && ((digitalRead(_SW_ORG_X) == 0) || (digitalRead(_SW_ORG_Y) == 0))))// && (m_bEms == false))
			{
				m_bStart = false;
				
				//if (digitalRead(_SW_EMS) == 0)printf("Emergency Switch\n");
				//else printf("Org Limit\n");
				
				m_bEms = true;
				motor.Stop();
				usleep(100);
			}


			///// Switch /////
			if (digitalRead(_SW_EMS) == 0) 
			{
				if (nSw_Ems == 0)
				{
					nSw_Ems = 1;
					printf("Emergency!!!\n");
				}
			}
			else
			{
				if (nSw_Ems == 1) 
				{
					nSw_Ems = 0;
					//printf("No Emergency\n");
				}
			}
		
			if (digitalRead(_SW_RUN) == 1) 
			{
				if (nSw_Run == 0)
				{
					nSw_Run = 1;
					

					if (m_bEms == false)
					{
						if (m_bStart == true)
						{
							printf("Stop\n");
							motor.Stop();
							m_bStart = false;		
						}
						else
						{
							printf("Start\n");
							motor.Reset();
							m_bStart = true;
						}
					}
					else
					{
						printf("shutdown\n");
						system("sudo shutdown -h now");
					}
				}
			}
			else
			{
				if (nSw_Run == 1) 
				{
					nSw_Run = 0;

					printf("Reset & Start\n");
					motor.Reset();
					m_bStart = true;
				}
			}

			if (digitalRead(_SW_ORG_X) == 0) 
			{
				if (nSw_Org_X== 0)
				{
					nSw_Org_X = 1;
					printf("Org(X)!!!\n");
				}
			}
			else
			{
				if (nSw_Org_X == 1) 
				{
					nSw_Org_X = 0;
					//printf("No Org(X)\n");
				}
			}
			if (digitalRead(_SW_ORG_Y) == 0) 
			{
				if (nSw_Org_Y== 0)
				{
					nSw_Org_Y = 1;
					printf("Org(Y)!!!\n");
				}
			}
			else
			{
				if (nSw_Org_Y == 1) 
				{
					nSw_Org_Y = 0;
					//printf("No Org(Y)\n");
				}
			}




#if 0
			if (m_bEms == false)
			{			
				if (digitalRead(_SW_RUN) == 1)
				{
					if (m_bStart == true)
					{
						printf("Stop\n");
						m_bStart = false;		
						motor.Stop();
					}
					else
					{
						printf("Start\n");
						m_bStart = true;
						motor.Reset();
					}
					usleep(100);
					while (digitalRead(_SW_RUN) == 1) usleep(100);
				}
			}
			else
			{			
				printf("Reset & Start\n");
				m_bStart = true;
				motor.Reset();
			}
#endif
		}
#else
		if (digitalRead(_SW_EMS) == 0) printf("Ems\n");
		if (digitalRead(_SW_RUN) == 1) printf("Run\n");
		usleep(1000);
#endif
		usleep(10);
	}
}

void Org()
{
	m_nOrg = 0;
	m_bOrgMoving = true;
	int nOrg = 0;
	float fValue_X = 0.0f;
	float fValue_Y = 0.0f;
	float fDef = 1.0f;
	motor.Reboot();
	motor.Wait_Delay(1000);
	motor.SetTorque(true, true);
	while((m_bAgingEnd == false) && (m_nOrg != 0x03))
	{
		if ((nOrg & 0x01) == 0) 
		{		
			fValue_X += fDef;
			motor.Set_Angle(_X, 360.0f * 1 + fValue_X);
		}
		if ((nOrg & 0x02) == 0) 
		{
			fValue_Y += fDef;
			motor.Set_Angle(_Y, 360.0f * 1 + fValue_Y);
		}
		motor.Send_Motor(4);
		motor.Wait_Delay(4);
		
		if (digitalRead(_SW_ORG_X) == 0) 
		{
			printf("Found X\n");
			nOrg |= 0x01;
		}
		if (digitalRead(_SW_ORG_Y) == 0) 
		{
			printf("Found Y\n");
			nOrg |= 0x02;
		}
		
		if (nOrg == 0x03)
		{
			m_nOrg = nOrg;

			motor.Set_Angle(_X, -90.0f + motor.Get_Angle(_X));
			motor.Set_Angle(_Y, -90.0f + motor.Get_Angle(_Y));
			motor.Send_Motor(1000);
			motor.Wait_Delay(1000);
		
			motor.Reboot();
			motor.Wait_Delay(1000);
			motor.SetTorque(true, true);

			printf("move 0\n");
			motor.Set_Angle(_X, 0.0f);
			motor.Set_Angle(_Y, 0.0f);
			motor.Send_Motor(1000);
			motor.Wait_Delay(1000);
			break;
		}
		//usleep(0);
	}
	m_bOrgMoving = false;
}
int main(int argc, char* argv[]) {

	if (wiringPiSetupGpio() == -1) return -1;

	pinMode(_SW_ORG_X, INPUT);
	pinMode(_SW_ORG_Y, INPUT);
	pinMode(_SW_EMS, INPUT);
	pinMode(_SW_RUN, INPUT);

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	if (motor.IsOpen() == false)
	{
		printf("Cannot open serial port\n");
		return -1;
	}
#ifdef _DRS_0603	
	motor.SetParam(0, _MODEL_DRS_0603);
	motor.SetParam(1, _MODEL_DRS_0603);
#endif

	float fTurn = 0.0f;
	int nCmd = 0;
	if (argc > 1)  
	{
		if (strcasecmp(argv[1], "--reset") == 0) 
		{
			nCmd++; 
			motor.Reset();
			printf("Reset\n");
		}
		if (strcasecmp(argv[1], "--org") == 0)
		{
			nCmd++; 
			printf("Org\n");
			
			Org();
		}
		if (strcasecmp(argv[1], "--reboot") == 0)
		{
			nCmd++; 
			motor.Reboot();
			printf("Reboot\n");
		}		
		if (strcasecmp(argv[1], "--baud") == 0)
		{
			nCmd++; 

			int nID = atoi(argv[2]);
			int nBaud = atoi(argv[3]);
			
			printf("Change Baud rate : ID[%d] %d\n", nID, nBaud);

			motor.SetBaudrate(nID, nBaud);
			
			motor.Reboot();
		}
		if (strcasecmp(argv[1], "--id") == 0)
		{
			nCmd++; 

			int nID = atoi(argv[2]);
			int nNewID = atoi(argv[3]);

			printf("Change Baud rate : ID[%d -> %d]\n", nID, nNewID);
			motor.SetMotorID(nID, nNewID);
			motor.Reboot();
		}
		if (strcasecmp(argv[1], "--checking") == 0)
		{
			nCmd++; 
			bool abID[254];
			printf("\nChecking...\n");
			//memset(abID, 0, sizeof(bool) * 254);
			for (int i = 0; i <= 253; i++)
			{
				abID[i] = false;
				for (int j = 0; j < 3; j++)
				{
					CMotor::CTimer CTmr;
					CTmr.Set();
					motor.Read_Motor(i);
					bool bBreak = false;
					while(CTmr.Get() < 20) 
					{				
						if (motor.Read_Motor_IsReceived() == true) 
						{
							bBreak = true;
							abID[i] = true;
							break;
						}		
						usleep(0);
					}
					if ((motor.Read_Motor_IsReceived() == true) || (bBreak == true))
					{
						abID[i] = true;
						break;
					}
				}

				if ((i % 10) == 0) 
				{
					printf("%d\%\n", i * 100 / 253);
				}
				//printf("%d/253 : ", i);
				if (abID[i] == true) 
				{
					//printf("ID = %d", i);
					printf("ID = %d\n", i);
				}
				//printf("\n");
				usleep(0);
			}
			printf("100\%\n\n==============\n");
			for (int i = 0; i <= 253; i++)
			{
				if (abID[i] == true) 
				{
					//printf("ID = %d", i);
					printf("ID = %d\n", i);
				}
			}
			printf("==============\nChecking Done\n");
		}
		if (strcasecmp(argv[1], "--sensor") == 0)
		{
			nCmd++;			
			printf("Sensor\n");
			m_bCheckSensor = true;
			int nSw_Org_X = 0;
			int nSw_Org_Y = 0;
			int nSw_Ems = 0;
			int nSw_Run = 0;
			while(m_bCheckSensor == true)
			{
				if (digitalRead(_SW_EMS) == 0) 
				{
					if (nSw_Ems == 0)
					{
						nSw_Ems = 1;
						printf("Emergency!!!\n");
					}
				}
				else
				{
					if (nSw_Ems == 1) 
					{
						nSw_Ems = 0;
						printf("No Emergency\n");
					}
				}
				if (digitalRead(_SW_RUN) == 1) 
				{
					if (nSw_Run == 0)
					{
						nSw_Run = 1;
						printf("Run!!!\n");
					}
				}
				else
				{
					if (nSw_Run == 1) 
					{
						nSw_Run = 0;
						printf("No Run\n");
					}
				}
		
				if (digitalRead(_SW_ORG_X) == 0) 
				{
					if (nSw_Org_X== 0)
					{
						nSw_Org_X = 1;
						printf("Org(X)!!!\n");
					}
				}
				else
				{
					if (nSw_Org_X == 1) 
					{
						nSw_Org_X = 0;
						printf("No Org(X)\n");
					}
				}
				if (digitalRead(_SW_ORG_Y) == 0) 
				{
					if (nSw_Org_Y== 0)
					{
						nSw_Org_Y = 1;
						printf("Org(Y)!!!\n");
					}
				}
				else
				{
					if (nSw_Org_Y == 1) 
					{
						nSw_Org_Y = 0;
						printf("No Org(Y)\n");
					}
				}
				usleep(1000);
			}
		}
		if (nCmd > 0) 
		{
			motor.Close();
			return 0;
		}
		else
		{
			int nCmp = 0;
			if (strcasecmp(argv[1], "-?") == 0) nCmp++;
			if (strcasecmp(argv[1], "/?") == 0) nCmp++;
			if (strcasecmp(argv[1], "--?") == 0) nCmp++;
			if (strcasecmp(argv[1], "--help") == 0) nCmp++;
			if (strcasecmp(argv[1], "/help") == 0) nCmp++;
			if (nCmp > 0)
			{
	// X 축만 움직일 경우(11 turn, 30000 ms)
	// sudo ./test 11 30000 1
	// 전체축이 움직일 경우(11 turn, 30000 ms)
	// sudo ./test 11 30000 0
	// sudo ./test 11 30000		
				printf("help 								: ./test --?\n");
				printf("Checking 							: ./test --checking\n");
				printf("Change ID							: ./test --id [id] [newid]\n");
				printf("Change Baudrate						\t: ./test --baud [id] [baudrate]\n");
				printf("[I/O, Sensor Checking]				\t\t: ./test --sensor\n");
				printf("Origin								: ./test --org\n");
				printf("Aging 								: ./test --aging\n");
				printf("Reset 								: ./test --reset\n");
				printf("Reboot								: ./test --reboot\n");
				printf("move only X Axis(11 turn, 30000 ms) \t\t\t\t: sudo ./test 11 30000 1\n");		
				printf("move only Y Axis(11 turn, 30000 ms) \t\t\t\t: sudo ./test 11 30000 2\n");		
				printf("move All Axis(11 turn, 30000 ms) 	\t\t\t: sudo ./test 11 30000 0  ( or   sudo ./test 11 30000 )\n");		
				return 0;
			}
			else
			{
			 	fTurn = atof(argv[1]);	
				printf("Turn %f\n", fTurn);
			}
		}
	}

	// Thread
	int nRet = pthread_create(&m_thRun, NULL, Thread_Input, NULL);
	if (nRet != 0)
	{
		printf("Thread(Input) Init Error\r\n");
		return -1;
	}
	printf("Init Thread(Input)\r\n");
	
//// Sig Action - Cntl-C ////////////////////////////////////////////////////////////////////////////////
	struct sigaction intsig;																			//
																									//
	intsig.sa_handler = sig_int;																		//
	sigemptyset(&intsig.sa_mask);																		//
	intsig.sa_flags=0;																					//
																									//
	if ( sigaction(SIGINT, &intsig, 0) == -1 ) {															//
		printf("Error sigaction(SIGINT)\n");			//				
		return -1;																				//
	}																								//
//////////////////////////////////////////////////////////////////////////////////////////////////////


	

	// you need to set the param if you have DRS_0401, DRS_0402, DRS_0601, DRS_0602, DRS_0603 
	int nSpd = 3000;
	int nWait = 20000;
	
	// Error Clear
	motor.Reset();
	//motor.Wait_Delay(1000);
	// Torq On
	motor.SetTorque(true, true);

	//motor.Motion_Play("test5dof.dmt");

	//printf("Start\n");

	// Port Close
	//motor.Close();

	//motor.SetParam_MechMove(0, 2048.0);

	//motor.Read_Motor_Push(

	//motor.SetParam_Dir(1, 1);
#if 0
	motor.Set_Angle(0, 0.0f);
	motor.Set_Angle(1, 0.0f);
	motor.Send_Motor(nSpd);
	//motor.Wait_Delay(1000);//(0, 0.0f, 10000);
	motor.Wait_Position(1, 0.0f, nWait);
	motor.Wait_Position(0, 0.0f, nWait);
#endif
	//float fTurn = 0.0f;
	//int nCmd = 0;
	if (argc > 1)  
	{
		
		if (strcasecmp(argv[1], "--aging") == 0)
		{
			nCmd++; 
			printf("Aging\n");
			m_nAging = 1;


			if (m_nOrg != 0x03)
			{
				printf("Origin\n");
				Org();
			}

			int nY = 0;
			int nX = 1;

			float fX0 = -12.0f;
			float fY0 = -12.0f;
			#if 1
			printf("Ready\n");
			motor.Set_Angle(nX, fX0 * 360.0f);
			motor.Set_Angle(nY, fY0 * 360.0f);
			motor.Send_Motor(10000);
			motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
			motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
			motor.Wait_Delay(1000);
			#endif
			float fPercent = 1.0f;
			int nTime = 20000;
			m_bStart = true; // 일단 시작
			while(m_bAgingEnd == false)
			{
				if ((m_bStart == true) && (m_bEms == false))
				{
					printf("First\n");
					motor.Set_Angle(nX, 0.0f * 360.0f);
					motor.Set_Angle(nY, 0.0f * 360.0f);
					motor.Send_Motor(nTime);
					motor.Wait_Delay(nTime * fPercent);
					if (m_bAgingEnd == true) break;
					
					printf("Last\n");
					motor.Set_Angle(nX, fX0 * 360.0f);
					motor.Set_Angle(nY, fY0 * 360.0f);
					motor.Send_Motor(nTime);
					motor.Wait_Delay(nTime * fPercent);
					if (m_bAgingEnd == true) break;
				}
			}
			if (motor.IsStop() == true) 
			{
				//motor.Wait_Delay(3000);
				motor.Reset();
			}

			if (m_nOrg != 0)
			{
				printf("Done -> goto ORG\n");
				motor.Set_Angle(nX, 1 * 360.0f);
				motor.Set_Angle(nY, 1 * 360.0f);
				motor.Send_Motor(20000);
				motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
				motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
			}
		}		
		if (strcasecmp(argv[1], "--run") == 0)
		{
			nCmd++; 
			printf("Aging\n");
			m_nAging = 1;


			if (m_nOrg != 0x03)
			{
				printf("Origin\n");
				Org();
			}

			int nY = 0;
			int nX = 1;

			float fX0 = -5.0f;
			float fY0 = -10.0f;
			float fW = 3.0f;
			float fH = 3.0f;
			#if 1
			printf("Ready\n");
			motor.Set_Angle(nX, fX0 * 360.0f);
			motor.Set_Angle(nY, fY0 * 360.0f);
			motor.Send_Motor(20000);
			motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
			motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
			motor.Wait_Delay(1000);
			#endif
			float afX[2];
			float afY[2];
			afX[0] = 0.0f;
			afY[0] = 0.0f;
			afX[1] = 1.0f;
			afY[1] = 1.0f;
			float fX, fY;

			float fX1, fY1;
			int nW = 4;
			int nH = 4;
			float fPercent = 0.7f;
			int nTime = 3000;
			//m_bStart = true;
			while(m_bAgingEnd == false)
			{
				if ((m_bStart == true) && (m_bEms == false))
				{
					for (int j = 0; j < nH; j++)
						for (int i = 0; i < nW; i++)
						{
							int nPulse = pow(-1, j % 2);
							int nMax = (nW - 1) * (j % 2);
							fX1 = fX0 - j * fH;
							fY1 = fY0 + (nMax + i * nPulse) * fW;
							printf("1-[%.2f, %.2f]\n", fX1, fY1);
							
							#if 1
							// 1		
							fX = afX[1];
							motor.Set_Angle(nX, (fX1 + fX) * 360.0f);
							motor.Send_Motor(nTime * 2);
							motor.Wait_Delay(nTime * 2 * fPercent);//Position(nX, motor.Get_Angle(nX), 30000);
							//motor.Wait_Delay(1000);
							if (m_bAgingEnd == true) break;
							printf("2\n");
							// 2		
							fY = afY[1];				
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							printf("3\n");
							// 3		
							fX = afX[0];
							motor.Set_Angle(nX, (fX1 + fX) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
							if (m_bAgingEnd == true) break;
							printf("4\n");
							// 4		
							fY = afY[0];				
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							
							printf("S1\n");
							// 대각
							fX = afX[1];
							fY = afY[1];
							motor.Set_Angle(nX, (fX1 + fX) * 360.0f);
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
							//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							
							printf("Up\n");
							// Up
							fY = afY[0];
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							
							printf("S2\n");
							// 대각
							fX = afX[0];
							fY = afY[1];
							motor.Set_Angle(nX, (fX1 + fX) * 360.0f);
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
							//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							
							printf("Last\n");
							// First
							fY = afY[0];
							motor.Set_Angle(nY, (fY1 + fY) * 360.0f);
							motor.Send_Motor(nTime);
							motor.Wait_Delay(nTime * fPercent);//motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
							if (m_bAgingEnd == true) break;
							#endif
						}
				}

				// test
				//break;
			}
			if (motor.IsStop() == true) 
			{
				//motor.Wait_Delay(3000);
				motor.Reset();
			}

			if (m_nOrg != 0)
			{
				printf("Done -> goto ORG\n");
				motor.Set_Angle(nX, 1 * 360.0f);
				motor.Set_Angle(nY, 1 * 360.0f);
				motor.Send_Motor(20000);
				motor.Wait_Position(nX, motor.Get_Angle(nX), 30000);
				motor.Wait_Position(nY, motor.Get_Angle(nY), 30000);
			}
		}
		if (nCmd > 0) 
		{
			motor.Close();
			return 0;
		}
	} 

	if (nCmd == 0)
	{
		nSpd = nSpd * 3;	
		if (argc > 2)  
		{
		 	nSpd = atoi(argv[2]);	
			if (nWait > nSpd) nWait = nSpd;
			printf("Spd %d\n", nSpd);
		} 

		int nAxis = 0; // all
		if (argc > 3)  
		{
		 	nAxis = atoi(argv[3]);			
			if (nAxis > 2) nAxis = 0;
			if (nAxis < 0) nAxis = 0;
			printf("Axis=%s\n", ((nAxis == 0) ? "ALL":((nAxis == 1) ? "X":"Y")));
		} 
		
		float fDest = 360.0f * fTurn;//7.0f;
		if ((nAxis == 0) || (nAxis == 2)) motor.Set_Angle(0, fDest);
		if ((nAxis == 0) || (nAxis == 1))motor.Set_Angle(1, fDest);
		motor.Send_Motor(nSpd);
		//motor.Wait_Delay(3000);
		motor.Wait_Position(0, fDest, nWait);
		motor.Wait_Position(1, fDest, nWait);
	}

	motor.Close();

#if 0
	motor.Reset();
	fDest = 0.0f;
	motor.Set_Angle(1, fDest);
	motor.Send_Motor(nSpd);
	//motor.Wait_Delay(3000);
	motor.Wait_Position(1, fDest, nWait);
	int nError0 = motor.GetErrorCode(0);
	int nError1 = motor.GetErrorCode(1);
	if ((nError0 != 0) || (nError1 != 0)) printf("Error Code[%02x, %02x]\n", nError0, nError1);

#endif	
#if 0
	//motor.Set_Turn(0, 500);
	for (int j = 0; j < 3; j++)
	{
		motor.Set_Angle(0, 0.0f);	
		motor.Set_Angle(1, 0.0f);
		motor.Set_Angle(2, 0.0f);
		motor.Set_Angle(3, 0.0f);
		motor.Set_Angle(4, -70.0f);
		
		motor.Send_Motor(1000);
		
		//motor.Read_Motor()
		motor.Wait_Delay(1000);
		for (int i = 0; i < 5; i++)
		{
			motor.Set_Angle(i, -45.0f);
			motor.Send_Motor(1000);
			motor.Wait_Motor();
			motor.Wait_Delay(1000);
		}
	}
#endif
	printf("Done");
#if 0

	// Control 1 Motor
	CMotor motor;
	float fValue = 360.0f * 10.0f;//90.0f;
	int nTime = 1000 * 20;
	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	
	// Show Feedback Message(if you want - for checking - )
	//motor.Read_Motor_ShowMessage(true);
	
	motor.SetParam(_MOTOR_219, _MODEL_DRS_0603);

	// Change Motor Direction ( forward -> inverse ) : only [Angle] control...
	motor.SetParam_Dir(_MOTOR_219, 0);


while (1)
{
	
	// Error Flag Reset (if you have any errors...)
	printf("Reset\r\n");
	motor.Reset();
	motor.Wait_Delay(100); //

	// Torq On (****************)
	printf("Torq On\r\n");
	motor.SetTorque(true, true);

	motor.Read_Motor(_MOTOR_219);
	motor.Wait_Delay(100); //
//
	printf("Current Positionf\r\n");
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	motor.Set_Angle(_MOTOR_219, 0.0f);
	motor.Set_Flag_Led_Blue(_MOTOR_219, true);
	// Move
	motor.Send_Motor(1000);
	motor.Wait_Motor();
	
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	// Delay
	motor.Wait_Delay(3000); //

	printf("%.2f\r\n", fValue);
	motor.Set_Angle(_MOTOR_219, fValue);
	motor.Set_Flag_Led_Blue(_MOTOR_219, false);
	// Move
	motor.Send_Motor(nTime);
	motor.Wait_Motor();
	
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	// Delay
	motor.Wait_Delay(3000); //

	printf("0.0f\r\n");
	motor.Set_Angle(_MOTOR_219, 0.0f);
	motor.Set_Flag_Led_Green(_MOTOR_219, true);
	// Move
	motor.Send_Motor(nTime);
	motor.Wait_Motor();
	
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	// Delay
	motor.Wait_Delay(3000); //

	printf("%.2f\r\n", -fValue);
	motor.Set_Angle(_MOTOR_219, -fValue);
	motor.Set_Flag_Led_Green(_MOTOR_219, false);
	// Move
	motor.Send_Motor(nTime);
	motor.Wait_Motor();
	
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	// Delay
	motor.Wait_Delay(3000); //
	
	printf("0.0f\r\n");
	motor.Set_Angle(_MOTOR_219, 0.0f);
	// Move
	motor.Send_Motor(nTime);
	motor.Wait_Motor();
	
	printf("->%.2f(0x%02x)\r\n", motor.Get_Pos_Angle(_MOTOR_219), motor.Get_Pos_Evd(_MOTOR_219));
	// Delay
	motor.Wait_Delay(3000); //



	printf("Turn (+)\r\n");
	motor.Set_Turn(_MOTOR_219, 100);
	// Move
	motor.Send_Motor(1000);
	
	motor.Wait_Delay(10000); //


	

	printf("Turn (+) : 500 \r\n");
	motor.Set_Turn(_MOTOR_219, 500);
	// Move
	motor.Send_Motor(1000);
	
	motor.Wait_Delay(10000); //


	


	printf("Turn (-)\r\n");
	motor.Set_Turn(_MOTOR_219, -100);
	// Move
	motor.Send_Motor(1000);
	
	motor.Wait_Delay(10000); //



	printf("Turn (-) : 500 \r\n");
	motor.Set_Turn(_MOTOR_219, -500);
	// Move
	motor.Send_Motor(1000);
	
	motor.Wait_Delay(10000); //


	printf("Stop\r\n");

	motor.Stop();
	
	motor.Reset();
}
#endif
#if 0

#if 1
	// Control 1 Motor
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	
	// Show Feedback Message(if you want - for checking - )
	//motor.Read_Motor_ShowMessage(true);
	
	motor.SetParam(_MOTOR_0, _MODEL_DRS_0601);
	motor.SetParam(_MOTOR_1, _MODEL_DRS_0401);
	motor.SetParam(_MOTOR_2, _MODEL_DRS_0602);


	// Change Motor Direction ( forward -> inverse ) : only [Angle] control...
	motor.SetParam_Dir(_MOTOR_1, 1);
	
	// Error Flag Reset (if you have any errors...)
	motor.Reset();

	// Torq On (****************)
	motor.SetTorque(true, true);
//
	motor.Set_Angle(_MOTOR_0, 0.0f);
    motor.Set_Flag_Led_Blue(_MOTOR_0, true);
	motor.Set_Angle(_MOTOR_1, 0.0f);
	motor.Set_Angle(_MOTOR_2, 0.0f);
	// Move
	motor.Send_Motor(1000);
	// Delay
	motor.Wait_Motor();

	printf("Stoped\r\n");
	motor.Stop();
	motor.Wait_Delay(100);

	// you must use Reset function when you stoped before
	motor.Reset();

	// Time Delay
	motor.Wait_Delay(3000);


	
//
	motor.Set_Angle(_MOTOR_0, 90.0f);
	motor.Set_Angle(_MOTOR_1, 90.0f);
	motor.Set_Angle(_MOTOR_2, 90.0f);
	// Move
	motor.Send_Motor(1000);
	// Delay
	motor.Wait_Motor();
	
	printf("Motor Position => 0 [ %.2f ], 1 [ %.2f ], 2[ %.2f ]\r\n", motor.Get_Angle(_MOTOR_0), motor.Get_Angle(_MOTOR_1), motor.Get_Angle(_MOTOR_2));
//
	motor.Set_Angle(_MOTOR_0, 0.0f);
	motor.Set_Angle(_MOTOR_1, 0.0f);
	motor.Set_Angle(_MOTOR_2, 0.0f);
	// Move
	motor.Send_Motor(1000);
	// Delay
	motor.Wait_Motor();



	// Time Delay
	motor.Wait_Delay(500);

	
	printf("Motor Position => 0 [ %.2f ], 1 [ %.2f ], 2[ %.2f ]\r\n", motor.Get_Angle(_MOTOR_0), motor.Get_Angle(_MOTOR_1), motor.Get_Angle(_MOTOR_2));

	motor.Set_Turn(_MOTOR_0, 500);
	motor.Send_Motor(500);
	motor.Wait_Delay(3000);

	motor.Set_Turn(_MOTOR_1, 500);
	motor.Send_Motor(500);
	motor.Wait_Delay(3000);

	motor.Set_Turn(_MOTOR_2, 500);
	motor.Send_Motor(500);
	motor.Wait_Delay(3000);

	// stop
	motor.Set_Turn(_MOTOR_0, 0);
	motor.Set_Turn(_MOTOR_1, 0);
	motor.Set_Turn(_MOTOR_2, 0);
	motor.Send_Motor(500);

	
	printf("Done\r\n");
	#endif
	#endif
	return 0;
}
