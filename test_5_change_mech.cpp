// compile : 
//     sudo g++ -o test5 test_5_change_mech.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test5
// 기어비를 1/2 을 감속하여 90 도 움직이던 것이 45도 로만 움직이는 것을 보인다.
// 읽히는 형태도 변환된 형태가 읽힌다. 외부 기어축이나 감속기를 달 경우 사용 가능한 함수


#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);

	motor.SetParam_MechMove(0, 1024 / 2); // Default 셋팅에서는 1024 이나 이걸 절반으로 줄여 동작 및 읽어 들이는 것이 변하는 것을 확인한다.

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
