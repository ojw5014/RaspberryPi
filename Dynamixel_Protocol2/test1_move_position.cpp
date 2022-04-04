#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            // 모터의 방향을 반대로(Direction -> Backward)
            bool bDir_Inverse = false;
            bool bDynamixel_Pro = false;
            
            // m_CCom.SetParam(1, false, 1.0f, false); // 기본설정이라 굳이 하지 않아도 된다.(default setting)
            m_CCom.SetParam(2, bDir_Inverse, 1.0f, bDynamixel_Pro); // 맨 뒤의 false 를 true 로 바꿔주면 Dynamixel Pro 제어

            int i, anIDs[256];
            int nMotorMax = 2;
            anIDs[0] = 1;
            anIDs[1] = 2;
            
            // Torq On
            for (i =0; i < nMotorMax; i++) m_CCom.Set(anIDs[i], 1);
            m_CCom.SetTorq();

            // 초기위치를 확인한다. 이걸 안하면 Move 명령시 첫 동작을 급격하게 움직일 수 있다.
            m_CCom.SyncRead(nMotorMax, anIDs); // id 1, 2

            float fAngle = 90.0f;
            int nMovingTime = 3000; // 3 seconds (3초)
            m_CCom.Set(1, fAngle);
            m_CCom.Set(2, fAngle);
            m_CCom.Move_NoWait(nMovingTime, 0);
            m_CCom.Wait();

            fAngle = 0.0f;
            m_CCom.Set(1, fAngle);
            m_CCom.Set(2, fAngle);
            m_CCom.Move_NoWait(nMovingTime, 0);
            m_CCom.Wait();


            #if 0
            m_CCom.PlayFrameString("s2,1000,0,1:0,2:0,3:0,4:0", true); m_CCom.Wait();
            m_CCom.PlayFrameString("s2,2000,0,1:90,2:90,3:30,4:30", true); m_CCom.Wait();
            m_CCom.PlayFrameString("s2,2000,0,1:-90,2:-90,3:-30,4:-30", true); m_CCom.Wait();
            m_CCom.PlayFrameString("s2,5000,0,1:90,2:90,3:30,4:30", true); m_CCom.Wait();
            m_CCom.PlayFrameString("s2,5000,0,1:-90,2:-90,3:-30,4:-30", true); m_CCom.Wait();
            m_CCom.PlayFrameString("s2,1000,0,1:0,2:0,3:0,4:0", true); m_CCom.Wait();
            #else
            m_CCom.PlayFrameString("s2,1000,0,1:0,2:0,3:0,4:0");
            m_CCom.PlayFrameString("s2,2000,0,1:90,2:90,3:30,4:30");
            m_CCom.PlayFrameString("s2,2000,0,1:-90,2:-90,3:-30,4:-30");
            m_CCom.PlayFrameString("s2,5000,0,1:90,2:90,3:30,4:30");
            m_CCom.PlayFrameString("s2,5000,0,1:-90,2:-90,3:-30,4:-30");
            m_CCom.PlayFrameString("s2,1000,0,1:0,2:0,3:0,4:0");
            #endif
            m_CCom.Play("motion.txt");
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}