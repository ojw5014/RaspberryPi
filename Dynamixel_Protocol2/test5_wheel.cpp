#include "COjw_37_Protocol2.h"

// 사전에 Operation(Address 11) -> 1 (speed) 로 되어 있어야 동작 가능합니다.

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

            float fRpm = 10.0f; // 1분에 10 바퀴를 도는 정도의 속도, 회전 시작
            m_CCom.Set_Rpm(1, fRpm);
            m_CCom.Set_Rpm(2, fRpm);
            m_CCom.SetSpeed();

            m_CCom.Wait(5000); // 5초동안 기다려 본다.(5초동안 회전하다 멈추는 동작을 위해)
            
            // 멈춘다.
            m_CCom.Set_Rpm(1, 0);
            m_CCom.Set_Rpm(2, 0);
            m_CCom.SetSpeed();
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}