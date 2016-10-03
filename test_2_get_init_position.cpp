// compile : 
//     sudo g++ -o test2 test_2_get_init_position.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test2



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
	
	// put your checking motor
	motor.Read_Motor_Push(0); // 0 motor check 

////// Get Ready Position //////		
	// check start position for 1000 milli-seconds
	CMotor::CTimer CTmr; // variable for timer
	CTmr.Set(); // start timer
	motor.Read_Motor();
	while ( CTmr.Get() < 1000)
	{
		// break it if you get the position data from your motor.. 
		if (motor.Read_Motor_IsReceived() == true) break;
		usleep(0);
	}
	printf("ready Position = %.2f\n", motor.Get_Pos_Angle(0));

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
