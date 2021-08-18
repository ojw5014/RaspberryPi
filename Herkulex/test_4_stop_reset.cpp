// compile : 
//     sudo g++ -o test4 test_4_stop_reset.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test4

// Stop() �Լ��� ����� ��� Reset() �� �ϱ� ������ ����� �ȵǴ� ���� ���̴� example.
// Ems() �Լ��� emergency switch �Լ��� �̰� �� �ص� ���� Stop() �� ���� ����� ���δ�. �ٸ� ������ ��ũ�� ���� Ǯ����.

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

////////////////////////////////////////////////////////////////

////// Move first //////
	motor.Set_Angle(0, 90.0f);
	motor.Send_Motor(1000);	
	motor.Wait_Position(0, 45.0f, 1000);

	motor.Stop();


////// Move back ////// - cannot move by Stop() function - you can move it after running Reset() function
	motor.Set_Angle(0, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


	printf("Done first\n");

////////////////////////////////////////////////////////////////


	motor.Reset(); // Stop() �Լ��� call �� ��� Reset �� �ؾ� ������ �����ϴ�.
////// Move first //////
	motor.Set_Angle(0, 90.0f);
	motor.Send_Motor(1000);	
	motor.Wait_Position(0, 45.0f, 1000);

	motor.Stop();
	motor.Reset();

////// Move back ////// - cannot move by Stop() function - you can move it after running Reset() function
	motor.Set_Angle(0, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


	printf("Done\n");
	
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
