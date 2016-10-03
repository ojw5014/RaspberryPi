// compile : 
//     sudo g++ -o test0 test_6_change_dir.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test6



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

	motor.SetParam_Dir(0, 1); // 0 : default, 1 : inverse(backward)

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
