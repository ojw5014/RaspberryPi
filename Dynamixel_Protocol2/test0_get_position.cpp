#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            // test motors -> 1, 2
            int i, anIDs[256];
            int nMotorMax = 2;
            anIDs[0] = 1; // 첫 번째로   검사할 다이나믹셀의 ID -> 1
            anIDs[1] = 2; // 그 다음으로 검사할 다이나믹셀의 ID -> 2
            
            // 1, 2 번 다이나믹셀의 위치를 확인한다.
            m_CCom.SyncRead(nMotorMax, anIDs); // id 1, 2

            printf("ID %d : %.2f, ID %d : %.2f\r\n", anIDs[0], m_CCom.Get(anIDs[0]), anIDs[1], m_CCom.Get(anIDs[1]));
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}