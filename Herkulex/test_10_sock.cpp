// compile : 
//     sudo g++ -o test10 test_10_sock.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test10

// ���ǻ��� : ������ �� ������ ������ DR-Sim �� 3D �𵨸��� ����� �����ؾ� �մϴ�.
// motor.SetParam_Dir �Լ� example �� ���� �ٶ��ϴ�.

#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	motor.Open_Socket();
////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

	// ÷�ε� 0~4 �� ���͸� �����ϴ� ����� ����� ��� ������ �÷��� �Ѵ�. (DR-Sim ���� ���� ����)
	//motor.Motion_Play("test5dof.dmt");
	while(1)
	{
		usleep(1000);
	}

	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();
	motor.Close_Socket();
	return 0;
}

