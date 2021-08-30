#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            // test motors -> 1, 2
            int nTorqOn = 1;
            m_CCom.Set(1, nTorqOn);
            m_CCom.Set(2, nTorqOn);
            m_CCom.SetTorq();

            float fRpm, fAngle;
            for (int i = 0; i < 2; i++)
            {
                // set speed ... 모터의 속도를 정한다
                fRpm = 10.0f; // 1분에 10 바퀴를 도는 정도의 속도
                m_CCom.Set_Rpm(1, fRpm);
                m_CCom.Set_Rpm(2, fRpm);
                m_CCom.SetPosition_Speed();

                // moving ... 실제로 움직이기
                fAngle = 90.0f;
                m_CCom.Set(1, fAngle);
                m_CCom.Set(2, fAngle);
                m_CCom.SetPosition();

                m_CCom.Wait(3000); // 3초를 기다려 본다.

                //////////////////////////////////////////////
                
                // set speed ... 모터의 속도를 정한다
                fRpm = 100.0f; // 1분에 10 바퀴를 도는 정도의 속도
                m_CCom.Set_Rpm(1, fRpm);
                m_CCom.Set_Rpm(2, fRpm);
                m_CCom.SetPosition_Speed();

                // moving ... 실제로 움직이기
                fAngle = 0.0f;
                m_CCom.Set(1, fAngle);
                m_CCom.Set(2, fAngle);
                m_CCom.SetPosition();

                m_CCom.Wait(3000); // 3초를 기다려 본다.
            }
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}