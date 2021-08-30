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

            int anIDs[256];
            int nMotorMax = 2;
            anIDs[0] = 1; // 첫 번째로   검사할 다이나믹셀의 ID -> 1
            anIDs[1] = 2; // 그 다음으로 검사할 다이나믹셀의 ID -> 2
            
            // 1, 2 번 다이나믹셀의 위치를 확인한다. 이걸 하지 않으면 첫 위치를 모른채로 움직이게 되어 첫 동작이 급격히 움직일 수 있다.
            m_CCom.SyncRead(nMotorMax, anIDs); // id 1, 2 

            // S1(Raw) S2(Angle), MovingTime, Delay, ID:Angle, ...
            bool bNoWait = true;
            m_CCom.PlayFrameString("s2,2000,0,1:90,2:90", bNoWait); // 최초 명령 한번만 수행한다.
            m_CCom.Wait(); // 이걸 해 주어야 이 라인에서 동작시간만큼 대기한다.
            m_CCom.PlayFrameString("s2,1000,0,1:0,2:0", bNoWait); // 최초 명령 한번만 수행한다.
            m_CCom.Wait(); // 이걸 해 주어야 이 라인에서 동작시간만큼 대기한다.
            
        }
    }
    if (m_CCom.IsOpen() == true)
    {
        m_CCom.Close();
    }
    return 0;
}