// compile : 
//     sudo g++ -o test8 test_8_motion_play.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test8

// ���ǻ��� : ������ �� ������ ������ DR-Sim �� 3D �𵨸��� ����� �����ؾ� �մϴ�.
// motor.SetParam_Dir �Լ� example �� ���� �ٶ��ϴ�.

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

	// ÷�ε� 0~4 �� ���͸� �����ϴ� ����� ����� ��� ������ �÷��� �Ѵ�. (DR-Sim ���� ���� ����)
	motor.Motion_Play("test5dof.dmt");


	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
