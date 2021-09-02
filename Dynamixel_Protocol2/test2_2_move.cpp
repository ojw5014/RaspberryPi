#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            // test motors -> 1, 2
            m_CCom.SetParam(2, false, 1.0f, true);
            
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
                m_CCom.Move(nMovingTime, nDelay); // (3초 동안 모터의 움직임을 계속 지령한다.)

                //////////////////////////////////////////////
                fAngle = 0.0f;
                m_CCom.Set(1, fAngle);
                m_CCom.Set(2, fAngle);
                m_CCom.Move(nMovingTime, nDelay); // (3초 동안 모터의 움직임을 계속 지령한다.)
            }
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}