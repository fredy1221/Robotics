#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include "cmath"
#include "Autonomous.h"
#include "Driver_Controlled.h"

using namespace vex;

void Tank_DT()
{
   while(true) 
  {
    if(Controller1.Axis3.value()>25 or Controller1.Axis2.value()>25 )
    {
    LeftFront.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);
    task::sleep(20); 
    }

    else
    {
      stop_drivetrain();
    }
  }
}

void Tank_DT_v2()
{
   while(true) 
  {
    if(Controller1.Axis3.value()>25 or Controller1.Axis2.value()>25 )
    {
    LeftFront.spin(vex::directionType::fwd, (Controller1.Axis3.value()+Controller1.Axis4.value())/2, vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::fwd,(Controller1.Axis3.value()+Controller1.Axis4.value())/2, vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, (Controller1.Axis3.value()-Controller1.Axis4.value())/2, vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, (Controller1.Axis3.value()- Controller1.Axis4.value())/2, vex::velocityUnits::pct);
    task::sleep(20); 
    }
    else
    {
      stop_drivetrain();
    }
  }
}




