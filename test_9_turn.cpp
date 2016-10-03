// compile : 
//     sudo g++ -o test9 test_9_turn.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test9

// 속도제어로 움직이는 example 입니다. 기본 example 에서 Set_Angle -> Set_Turn 으로 변경하시면 됩니다.

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
	motor.Send_Motor(10); // 여기 속도값은 의미없음
	motor.Wait_Delay(1000); // 1초간 멈춘 후

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
