// compile : 
//     sudo g++ -o test7 test_7_DRS_0401.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test7
// model 명이 DRS_0401 인 경우 모터의 설정을 변경해 주는 함수
// DRS_0101, DRS_0201 의 경우 변경할 필요 없다. (Default)
// DRS_0401, DRS_0402, DRS_0601, DRS_0602, DRS_0603 의 경우 아래의 example 을 사용 가능.


#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

	int nMotor = 219;
////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	motor.SetParam(nMotor, _MODEL_DRS_0401);
////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

////// Move first //////
	motor.Set_Angle(nMotor, 90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


////// Move back //////
	motor.Set_Angle(nMotor, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
