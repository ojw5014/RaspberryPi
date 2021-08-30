#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            int nOn = 1; // 0 : Disable, 1 : Torq On
            m_CCom.Set(1, nOn);
            m_CCom.Set(2, nOn);
            m_CCom.SetTorq();
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}