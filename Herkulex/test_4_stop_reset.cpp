// compile : 
//     sudo g++ -o test4 test_4_stop_reset.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test4

// Stop() 함수를 사용할 경우 Reset() 을 하기 전에는 사용이 안되는 것을 보이는 example.
// Ems() 함수는 emergency switch 함수로 이걸 콜 해도 위의 Stop() 과 같은 결과를 보인다. 다만 모터의 토크도 같이 풀린다.

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


	motor.Reset(); // Stop() 함수를 call 한 경우 Reset 을 해야 동작이 가능하다.
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
