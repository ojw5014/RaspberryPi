// compile : 
//     sudo g++ -o test111 test_111.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test111



#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 57600);

	int nID = 101;
	motor.SetParam(nID, _MODEL_DRS_0601);
	motor.SetParam(0, _MODEL_DRS_0601);
	motor.SetParam(1, _MODEL_DRS_0601);

////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

////// Move first //////
	motor.Set_Angle(nID, 90.0f);
	motor.Set_Angle(0, 90.0f);
	motor.Set_Angle(1, 90.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


////// Move back //////
	motor.Set_Angle(nID, 0.0f);
	motor.Set_Angle(0, 0.0f);
	motor.Set_Angle(1, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Delay(1000);


	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
