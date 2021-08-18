#include "COjw_37_Protocol2.h"

#include <stdio.h>     // fopen, feof, fread, fclose 함수가 선언된 헤더 파일
#include <string.h>    // strlen, memset 함수가 선언된 헤더 파일

int main(int argc, char* argv[]) {
    
    CProtocol2 m_CProtocol;
    if (m_CProtocol.IsOpen() == false)
    {
        printf("try Open()\r\n");
        // if (m_CProtocol.Open("/dev/ttyAMA0", 1000000) == true) // Raspberrypi hat
        if (m_CProtocol.Open("/dev/ttyACM0", 1000000) == true) // OpenCM9.04
        // if (m_CProtocol.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            printf("Connected Ok\r\n");

#if 0 // Setting Dynamixel Pro
            m_CProtocol.SetParam(2, true);
            m_CProtocol.SetParam(3, true);
#endif
            int anIDs[256];
            int i;
            for (i =0; i < 8; i++)
            {
                anIDs[i] = i + 1; // 1 ~ 8
                m_CProtocol.Command_Set(anIDs[i], 1);
            }
            m_CProtocol.SetTorq();

#if 0
            // Read Position
            int nCnt_Motors = 8;

            for (int i = 0; i < nCnt_Motors; i++) anIDs[i] = i+1;
            m_CProtocol.SyncRead(nCnt_Motors, anIDs);
            printf("%f", m_CProtocol.m_afMot[anIDs[0]]);
#endif

#if 1 // 모션 테스트
            
            anIDs[0] = 1;
            anIDs[1] = 2;
            anIDs[2] = 3; 
            anIDs[3] = 4; 
            printf("SetMotion 2\r\n");
            m_CProtocol.Command_Set(1, -60);
            m_CProtocol.Command_Set(2, -60);
            m_CProtocol.Command_Set(3, -60);
            m_CProtocol.Command_Set(4, -60);
            m_CProtocol.Move(3000, 0);
            //m_CProtocol.SetPosition();
            printf("SetMotion 3\r\n");
            m_CProtocol.Command_Set(1, 60);
            m_CProtocol.Command_Set(2, 60);
            m_CProtocol.Command_Set(3, 60);
            m_CProtocol.Command_Set(4, 60);
            m_CProtocol.Move(3000, 0);
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













