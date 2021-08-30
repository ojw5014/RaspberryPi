#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            bool bBackward = true; // 방향 반대
            bool bDynamixelPro = false; // true 일 경우 다이나믹셀 프로로 동작
            m_CCom.SetParam(2, bBackward, 1.0f, bDynamixelPro); // 2번 모터의 방향을 바꾼다

            // test motors -> 1, 2
            int nTorqOn = 1;
            m_CCom.Set(1, nTorqOn);
            m_CCom.Set(2, nTorqOn);
            m_CCom.SetTorq();

            float fAngle;
            int nMovingTime = 3000; // 3초동안 움직인다.(ms)
            int nDelay = 0; // 동작 후 멈춰있는 시간(ms)
            for (int i = 0; i < 2; i++)
            {
                fAngle = 90.0f;
                m_CCom.Set(1, fAngle);
                m_CCom.Set(2, fAngle);
                m_CCom.Move_NoWait(nMovingTime, nDelay); // (모터의 움직임을 한번 지령한다.)
                m_CCom.Wait();
                //////////////////////////////////////////////
                fAngle = 0.0f;
                m_CCom.Set(1, fAngle);
                m_CCom.Set(2, fAngle);
                m_CCom.Move_NoWait(nMovingTime, nDelay); // (모터의 움직임을 한번 지령한다.)
                m_CCom.Wait();
            }
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}