// compile : 
//     sudo g++ -o test0 test_0_basic.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test0



#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);
	motor.SetParam(0, _MODEL_DRS_0603);
////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

////// Move first //////
	motor.Set_Angle(0, 90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


////// Move back //////
	motor.Set_Angle(0, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
