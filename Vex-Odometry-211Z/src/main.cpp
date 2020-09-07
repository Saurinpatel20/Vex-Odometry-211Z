/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel (saurinpatel and noahgelbart)                  */
/*    Created:                                                 */
/*    Description:  Odometry For Precise Autonomous Motion                    */
/*    Credits:      5225A For Pilons POS Document and QUEENS for odom Alg.    */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
#define Pi 3.14159265358979323846
#define SL 5 //distance from tracking center to middle of left wheel
#define SR 5 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation

double DeltaL = 0;
double DeltaR = 0;
double DeltaB = 0;
double currentL = 0;
double currentR = 0;
double PreviousL = 0;
double PreviousR = 0;
double DeltaTheta = 0;
double X = 0;
double Y = 0;
double Theta = 0;
double DeltaXSide = 0;
double DeltaYSide = 0;
double SideChord = 0;
double OdomHeading = 0;

/*---------------------------------------------------------------------------*/
/*                            Odometry Functions                             */
/*---------------------------------------------------------------------------*/
void TrackPOS() {
  // 2 cases could be occuring in odometry
  // 1: Going in a straight line
  // 2: Going in an arc motion

  // If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed (Trig ratios to calc movement)
  // Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
  // Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
  currentR = (RightFront.position(degrees) + RightBack.position(degrees)) / 2;
  currentL = (LeftFront.position(degrees) + LeftBack.position(degrees)) / 2;

  DeltaL = ((currentL - PreviousL) * 12.9590697) / 360;
  DeltaR = ((currentR - PreviousR) * 12.9590697) / 360;
  //DeltaB = ((finishB - StartB) * 12.9590697) / 360;
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);
  if(DeltaTheta == 0) {
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);
  } else {
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }
    OdomHeading = Theta * 57.295779513;
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;

    //This is for printing to the brain for debugging (Terminal coming soon)
    Brain.Screen.printAt(100,20, "X: %f",X);
    Brain.Screen.printAt(100,40, "Y: %f",Y);
    Brain.Screen.printAt(100,60, "Theta: %f",Theta);
    Brain.Screen.printAt(100,80, "Angle: %f",OdomHeading);
    Brain.Screen.printAt(100,100, "Displacement1: %f",SideChord);
    Brain.Screen.printAt(100,120, "DeltaLeftInches: %f",DeltaL);
    Brain.Screen.printAt(100,140, "DeltaRightInches: %f",DeltaR);
    Brain.Screen.printAt(100,160, "DeltaX: %f",DeltaXSide);
    Brain.Screen.printAt(100,180, "DeltaY: %f",DeltaYSide);

  }
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
    vexcodeInit();
    Brain.resetTimer();
    RightFront.resetRotation();
    RightBack.resetRotation();
    LeftFront.resetRotation();
    LeftBack.resetRotation();
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {

}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    Brain.Screen.clearScreen();
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    TrackPOS();
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton();
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(5); //Slight delay so the Brain doesn't overprocess
    }
}