// compile : 
//     sudo g++ -o test9 test_9_turn.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test9

// �ӵ������ �����̴� example �Դϴ�. �⺻ example ���� Set_Angle -> Set_Turn ���� �����Ͻø� �˴ϴ�.

#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);

////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

////// Move first //////
	motor.Set_Turn(0, 500);
	motor.Send_Motor(10); // ���� �ӵ����� �ǹ̾���
	motor.Wait_Delay(1000); // 1�ʰ� ���� ��

	motor.Stop(0);

////// Move back //////
	motor.Set_Turn(0, -500);
	motor.Send_Motor(10);
	motor.Wait_Delay(1000);

	motor.Stop(0);

	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
