#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include "cmath"
#include "Autonomous.h"

using namespace vex;

float EncoderRotation(float distance)
{
    return 360*distance/(3.14*2*5.12);
}

void stop_drivetrain()
{
  LeftBack.stop();
  LeftFront.stop();
  RightBack.stop();
  RightFront.stop();
}

void Drive_Front_encoder_speed(float distance, float pow)
{
  LeftFront.resetRotation();
  RightFront.resetRotation();   
  float higherpower;
  float firstpow=3;
  float Rotation=EncoderRotation(distance); //total rotations in degree
  float sat_dist=EncoderRotation(distance/7.0); //rotations in degree untill we reach max speed 
  float diff=5;   
  float power=pow;
  bool beg=(LeftFront.rotation(rotationUnits::deg)<sat_dist);
  
  while(std::abs(LeftFront.rotation(rotationUnits::deg))<Rotation-18) //while 18 degrees before the final position
  {
        beg=(LeftFront.rotation(rotationUnits::deg)<sat_dist); //beg = 1 if we are still in the acceleration phase
        higherpower=beg?((LeftFront.rotation(rotationUnits::deg)*power/sat_dist)+firstpow):power; //eza ba3dne abel l sat_dist zid l ser3a suivant equation de droite w bass sir add sat_distance bet sir ser3et l moteur = power
        if(LeftFront.rotation(rotationUnits::deg)>RightFront.rotation(rotationUnits::deg)) //eza left msabba2 right
        {
                   LeftBack.spin(directionType::fwd,higherpower-diff, velocityUnits::pct); //batte2 left
                   LeftFront.spin(directionType::fwd,higherpower-diff,velocityUnits::pct); 
                   RightBack.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   RightFront.spin(directionType::fwd,higherpower,velocityUnits::pct);
        }
         if(LeftFront.rotation(rotationUnits::deg)<RightFront.rotation(rotationUnits::deg)) //eza right msabba2 left
        {
                   LeftBack.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   LeftFront.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   RightBack.spin(directionType::fwd,higherpower-diff,velocityUnits::pct); //batte2 right
                   RightFront.spin(directionType::fwd,higherpower-diff,velocityUnits::pct);
        }
         if(LeftFront.rotation(rotationUnits::deg)==RightFront.rotation(rotationUnits::deg)) //eza add ba3ed ma tghayer chi
        {
                   LeftBack.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   LeftFront.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   RightBack.spin(directionType::fwd,higherpower,velocityUnits::pct);
                   RightFront.spin(directionType::fwd,higherpower,velocityUnits::pct);
        }
  }
  stop_drivetrain();
 
 }


 void Drive_Back_encoder_speed(float distance, float pow)
{   LeftFront.resetRotation();
    RightFront.resetRotation();   
    float higherpower;
    float firstpow=5;
    float Rotation=EncoderRotation(distance);
    float sat_dist=EncoderRotation(distance/7.0);
    float diff=3;   
    float power=pow;
    bool beg=(std::abs(LeftFront.rotation(rotationUnits::deg))<sat_dist);
    while(std::abs(LeftFront.rotation(rotationUnits::deg))<Rotation-18)
    
    {
        beg=(std::abs(LeftFront.rotation(rotationUnits::deg))<sat_dist);//check if in linear mode
        higherpower=beg?((std::abs(LeftFront.rotation(rotationUnits::deg))*power/sat_dist)+firstpow):power;//if linear the interpolate 
        //Brain.Screen.printAt(15,15,true,"Encoder:%lf",std::abs(LeftFront.rotation(rotationUnits::deg)));
        //Brain.Screen.printAt(15,36,true,"Degree:%lf:",Gyro1.value(rotationUnits::deg));
        if(LeftFront.rotation(rotationUnits::deg)<RightFront.rotation(rotationUnits::deg))
        {
                   LeftBack.spin(directionType::rev,higherpower-diff, velocityUnits::pct);
                   LeftFront.spin(directionType::rev,higherpower-diff,velocityUnits::pct);
                   RightBack.spin(directionType::rev,higherpower,velocityUnits::pct);
                   RightFront.spin(directionType::rev,higherpower,velocityUnits::pct);
        }
         if(LeftFront.rotation(rotationUnits::deg)>RightFront.rotation(rotationUnits::deg))
        {
                   LeftBack.spin(directionType::rev,higherpower,velocityUnits::pct);
                   LeftFront.spin(directionType::rev,higherpower,velocityUnits::pct);
                   RightBack.spin(directionType::rev,higherpower-diff,velocityUnits::pct);
                   RightFront.spin(directionType::rev,higherpower-diff,velocityUnits::pct);
        }
         if(LeftFront.rotation(rotationUnits::deg)==RightFront.rotation(rotationUnits::deg))
        {
                   LeftBack.spin(directionType::rev,higherpower,velocityUnits::pct);
                   LeftFront.spin(directionType::rev,higherpower,velocityUnits::pct);
                   RightBack.spin(directionType::rev,higherpower,velocityUnits::pct);
                   RightFront.spin(directionType::rev,higherpower,velocityUnits::pct);
        }
        
        
    }
  stop_drivetrain();

}



void rotateRight(float angle)
{
  float a=0.77;
  float b=-1;

  float anglee=angle*a+b;
   const int high=40;
   //const int low=10;
   int lowlow=5;
   float gyroca=Gyro1.value(rotationUnits::deg);
    Brain.Screen.printAt(250,123,"%lf",gyroca);

   while ( Gyro1.value(rotationUnits::deg) < gyroca+anglee*0.8 ) // more than 30 degrees from final position, rotate at high speed
   {     
       Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::fwd,high,velocityUnits::pct);
       LeftFront.spin(directionType::fwd,high,velocityUnits::pct);
       RightBack.spin(directionType::rev,high,velocityUnits::pct);
       RightFront.spin(directionType::rev,high,velocityUnits::pct);
       Brain.Screen.clearLine();
   }
   
    while (Gyro1.value(rotationUnits::deg)<gyroca+anglee-0.5 || Gyro1.value(rotationUnits::deg)>gyroca+anglee+0.5) //in a range of 0.5 degrees of final position
       {   
        
        if(Gyro1.value(rotationUnits::deg)<gyroca+anglee-0.5) //still didn't reach the final position, rotate at low speed
        {
        Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::fwd,lowlow,velocityUnits::pct);
       LeftFront.spin(directionType::fwd,lowlow,velocityUnits::pct);
       RightBack.spin(directionType::rev,lowlow,velocityUnits::pct);
       RightFront.spin(directionType::rev,lowlow,velocityUnits::pct);
        }
        else if ( Gyro1.value(rotationUnits::deg)>gyroca+anglee+0.5) //overshooted, rotate to the left at low speed

        {
        Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::rev,lowlow,velocityUnits::pct);
       LeftFront.spin(directionType::rev,lowlow,velocityUnits::pct);
       RightBack.spin(directionType::fwd,lowlow,velocityUnits::pct);
       RightFront.spin(directionType::fwd,lowlow,velocityUnits::pct);
        }
        
        else //reached final position
        {
            LeftBack.stop();
            LeftFront.stop();
            RightBack.stop();
            RightFront.stop();
            Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));
        }
     Brain.Screen.clearLine();
    }

 LeftBack.stop();
 LeftFront.stop();
 RightBack.stop();
 RightFront.stop();
 Brain.Screen.printAt(250,123,"%lf",Gyro1.value(rotationUnits::deg));
   
    
}    



void rotateLeft(float angles)
{ 
  float a=0.77;
  float b=-1;

  float angle=angles*a+b;
   const int high=40;
   //const int low=15;
   int lowlow=5;
   int gyroca=Gyro1.value(rotationUnits::deg);
   while ( Gyro1.value(rotationUnits::deg) >(gyroca-(angle*0.8)) )
   {     Brain.Screen.print(Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::rev,high,velocityUnits::pct);
       LeftFront.spin(directionType::rev,high,velocityUnits::pct);
       RightBack.spin(directionType::fwd,high,velocityUnits::pct);
       RightFront.spin(directionType::fwd,high,velocityUnits::pct);
       Brain.Screen.clearLine();
   }
       
    
    while (Gyro1.value(rotationUnits::deg)>gyroca-angle+0.3 || Gyro1.value(rotationUnits::deg)<gyroca-angle-0.3)
    {  
        if(Gyro1.value(rotationUnits::deg)>gyroca-angle)
        {
        Brain.Screen.print(Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::rev,lowlow,velocityUnits::pct);
       LeftFront.spin(directionType::rev,lowlow,velocityUnits::pct);
       RightBack.spin(directionType::fwd,lowlow,velocityUnits::pct);
       RightFront.spin(directionType::fwd,lowlow,velocityUnits::pct);
        }
        else if (Gyro1.value(rotationUnits::deg)<gyroca-angle)
        {
        Brain.Screen.print(Gyro1.value(rotationUnits::deg));
       LeftBack.spin(directionType::fwd,lowlow,velocityUnits::pct);
       LeftFront.spin(directionType::fwd,lowlow,velocityUnits::pct);
       RightBack.spin(directionType::rev,lowlow,velocityUnits::pct);
       RightFront.spin(directionType::rev,lowlow,velocityUnits::pct);
        }
        else {
         LeftBack.stop();
    LeftFront.stop();
    RightBack.stop();
    RightFront.stop();   
        }
     
     
    }
     LeftBack.stop();
    LeftFront.stop();
    RightBack.stop();
    RightFront.stop();
    Brain.Screen.print(Gyro1.value(rotationUnits::deg));
    
}
