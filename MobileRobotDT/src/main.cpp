/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\User                                             */
/*    Created:      Tue Dec 10 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightBack            motor         4               
// LeftBack             motor         2               
// RightFront           motor         3               
// LeftFront            motor         1               
// Gyro1                gyro          A               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Autonomous.h"
#include "Driver_Controlled.h"
using namespace vex;

int main(){
	// Initializing Robot Configuration. DO NOT REMOVE!
	vexcodeInit();

	//vex:: gyro Gyro1(Brain.ThreeWirePort.B);    
  
	Gyro1.calibrate(); 
	task::sleep(4000);

 
	Drive_Front_encoder_speed(70, 40);
	task::sleep(400);

	rotateRight(180);
	task::sleep(400);
 
	rotateLeft(180);
	task::sleep(400);

	Drive_Back_encoder_speed(70, 40);
	task::sleep(400);
  
	while (1)
	Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));

	Tank_DT();
	rotateLeft(180);
	rotateRight(90);

}
