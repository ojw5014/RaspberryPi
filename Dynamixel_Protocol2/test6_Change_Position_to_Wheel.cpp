#include "COjw_37_Protocol2.h"

// 사전에 Operation(Address 11) -> 1 (speed) 로 되어 있어야 동작 가능합니다.

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            // test motors -> 1, 2
            int nTorqOn = 0;
            m_CCom.Set(1, nTorqOn);
            m_CCom.SetTorq();

            m_CCom.Write_Byte(1, 11, 1); // 11: operation -> 1(Wheel) 3(Position) 4(Multiturn)

            nTorqOn = 1;
            m_CCom.Set(1, nTorqOn);
            m_CCom.SetTorq();

            m_CCom.Set_Rpm(1, 10);
            m_CCom.SetSpeed();

            m_CCom.Wait(5000);
            
            m_CCom.Set_Rpm(1, 0);
            m_CCom.SetSpeed();

            nTorqOn = 0;
            m_CCom.Set(1, nTorqOn);
            m_CCom.SetTorq();

            m_CCom.Write_Byte(1, 11, 3); // 11: operation -> 1(Wheel) 3(Position) 4(Multiturn)

        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}