// compile : 
//     sudo g++ -o test5 test_5_change_mech.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test5
// ���� 1/2 �� �����Ͽ� 90 �� �����̴� ���� 45�� �θ� �����̴� ���� ���δ�.
// ������ ���µ� ��ȯ�� ���°� ������. �ܺ� ������̳� ���ӱ⸦ �� ��� ��� ������ �Լ�


#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);

	motor.SetParam_MechMove(0, 1024 / 2); // Default ���ÿ����� 1024 �̳� �̰� �������� �ٿ� ���� �� �о� ���̴� ���� ���ϴ� ���� Ȯ���Ѵ�.

////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

////// Move first //////
	motor.Set_Angle(0, 90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);
	printf("first Position = %.2f\n", motor.Get_Pos_Angle(0));

////// Move back //////
	motor.Set_Angle(0, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);
	printf("return Position = %.2f\n", motor.Get_Pos_Angle(0));

	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
