// compile : 
//     sudo g++ -o test1 test_1_get_position.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test1



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
	//motor.Set_Angle(0, 90.0f);

for (int i = 0; i < 30; i++)
{
	motor.Set_Angle(0, 0.0f);
	motor.Set_Angle(1, 0.0f);
	motor.Set_Angle(2, 0.0f);
	motor.Set_Angle(3, 0.0f);
	motor.Set_Angle(4, 0.0f);
	motor.Send_Motor(1000);
	motor.Wait_Motor();//Delay(1000);
	printf("first Position = %.2f\n", motor.Get_Pos_Angle(0));

////// Move back //////
	
	motor.Set_Angle(0, 45.0f);
	motor.Set_Angle(1, 30.0f);
	motor.Set_Angle(2, -45.0f);
	motor.Set_Angle(3, 45.0f);
	motor.Set_Angle(4, -45.0f);
	motor.Send_Motor(1000);
	motor.Wait_Motor();//Delay(1000);
	printf("return Position = %.2f\n", motor.Get_Pos_Angle(0));
}
	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
