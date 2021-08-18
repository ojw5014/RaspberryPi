// compile : 
//     sudo g++ -o test3 test_3_move_override.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test3

// 90���� ���� -90�� ��ġ�� ���ϴ� �� 45���� ��ġ���� ������ ���� �� �� 0 ��ġ�� �̵��ϴ� ����Դϴ�. �� ��� Stop �� ������ �ϳ��� ���͸� �ý��� ������ �ƴ� �ϳ��� ���͸� �����ϹǷ� Reset �� �� �ʿ�� �����ϴ�.
//

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
	motor.Set_Angle(0, 90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);
	printf("first Position = %.2f\n", motor.Get_Pos_Angle(0));

	motor.Set_Angle(0, -90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Position(0, 45.0f, 1000);
	
	motor.Stop(0);
	printf("second Position = %.2f\n", motor.Get_Pos_Angle(0));
	
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
