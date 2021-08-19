#include <stdio.h>     // fopen, feof, fread, fclose 함수가 선언된 헤더 파일
#include <string.h>    // strlen, memset 함수가 선언된 헤더 파일
#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    
    CProtocol2 m_CProtocol;
    if (m_CProtocol.IsOpen() == false)
    {
        printf("try Open()\r\n");
        if (m_CProtocol.Open("/dev/ttyAMA0", 1000000) == true) // Raspberrypi hat
        // if (m_CProtocol.Open("/dev/ttyACM0", 1000000) == true) // OpenCM9.04
        // if (m_CProtocol.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            printf("Connected Ok\r\n");

            // 2번과 4번 모터는 방향을 반대로(Direction -> Backward)
            // m_CProtocol.SetParam(1, false, 1.0f, false);
            m_CProtocol.SetParam(2, true, 1.0f, false);
            // m_CProtocol.SetParam(3, false, 1.0f, false);
            m_CProtocol.SetParam(4, true, 1.0f, false);

            int anIDs[256];
            int i;

            int nMotorMax = 4;
            
            anIDs[0] = 1;
            anIDs[1] = 2;
            anIDs[2] = 3; 
            anIDs[3] = 4; 
            
#if 1 // Torq On
            for (i =0; i < nMotorMax; i++)
            {
                anIDs[i] = i + 1; // 1 ~ 4
                m_CProtocol.Command_Set(anIDs[i], 1);
            }
            m_CProtocol.SetTorq();
#endif

#if 1 // Read Position - 동작 전에 최초의 위치를 확인한다. 이걸 안하면 Move 명령시 첫 동작을 급격하게 움직일 수 있다.
            // Read Position
	        m_CProtocol.SyncRead(nMotorMax, anIDs);
            printf("%f", m_CProtocol.m_afMot[anIDs[0]]);
#endif

#if 1 // 모션 테스트
        for (int nTest = 0; nTest < 5; nTest++)
        {
            printf("SetMotion 1\r\n");
            m_CProtocol.Command_Set(1, -50);
            m_CProtocol.Command_Set(2, -50);
            m_CProtocol.Command_Set(3, -50);
            m_CProtocol.Command_Set(4, -50);
#if 1
            m_CProtocol.Move(3000, 0);
#else
            m_CProtocol.SetPosition();
            sleep(3);
#endif
            // Read Position
	        m_CProtocol.SyncRead(nMotorMax, anIDs);
            printf("%f", m_CProtocol.m_afMot[anIDs[0]]);

	        printf("SetMotion 2\r\n");
            m_CProtocol.Command_Set(1, 50);
            m_CProtocol.Command_Set(2, 50);
            m_CProtocol.Command_Set(3, 50);
            m_CProtocol.Command_Set(4, 50);
            #if 1
            m_CProtocol.Move(3000, 0);
#else
            m_CProtocol.SetPosition();
            sleep(3);
#endif
            // Read Position
	        m_CProtocol.SyncRead(nMotorMax, anIDs);
            printf("%f", m_CProtocol.m_afMot[anIDs[0]]);

	        printf("SetMotion 3\r\n");
            m_CProtocol.Command_Set(1, 0);
            m_CProtocol.Command_Set(2, 0);
            m_CProtocol.Command_Set(3, 0);
            m_CProtocol.Command_Set(4, 0);
            #if 1
            m_CProtocol.Move(3000, 0);
#else
            m_CProtocol.SetPosition();
            sleep(3);
#endif
            // Read Position
	        m_CProtocol.SyncRead(nMotorMax, anIDs);
            printf("%f", m_CProtocol.m_afMot[anIDs[0]]);
        }
            
#endif
        }
        else
        {
            printf("Connected Fail\r\n");
        }
    }
    printf("Test Ok\r\n");
    if (m_CProtocol.IsOpen() == true)
    {
        m_CProtocol.Close();
        printf("Close Port\r\n");
    }
    return 0;
}