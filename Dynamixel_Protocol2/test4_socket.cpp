#include "COjw_37_Protocol2.h"

int main(int argc, char* argv[]) {
    CProtocol2 m_CCom;
    if (m_CCom.IsOpen() == false)
    {
        if (m_CCom.Open("/dev/ttyUSB0", 1000000) == true) // USB
        {
            m_CCom.SetParam(2, false, 1.0f, true);
            
            // test motors -> 1, 2
            int nTorqOn = 1;
            m_CCom.Set(1, nTorqOn);
            m_CCom.Set(2, nTorqOn);
            m_CCom.SetTorq();

            int anIDs[256];
            int nMotorMax = 2;
            anIDs[0] = 1; // 첫 번째로   검사할 다이나믹셀의 ID -> 1
            anIDs[1] = 2; // 그 다음으로 검사할 다이나믹셀의 ID -> 2
            
            // 1, 2 번 다이나믹셀의 위치를 확인한다. 이걸 하지 않으면 첫 위치를 모른채로 움직이게 되어 첫 동작이 급격히 움직일 수 있다.
            m_CCom.SyncRead(nMotorMax, anIDs); // id 1, 2 




            m_CCom.Open_Socket(8081); // 소켓 명령을 받을 준비를 한다.(포트 8081 - 외부에서 접속시 이 포트를 이용해 접속을 한다.)
            while(1)
            {
                sleep(10);
            }
            
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}