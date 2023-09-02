/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       CHS Robotics                                              */
/*    Created:      9/2/2023, 12:45:16 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// vscode.dev/

#include "vex.h"

using namespace vex;

competition Competition;


void pre_auton(void) {

}

// Settings (adjust these values)
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; // SensorValue - DesiredValue (delta X) : Positional -> speed -> acceleration
int prevError = 0; // Position 20 miliseconds ago
int derivative; // error - prevError : Speed (helps with friction, battery voltage)
int totalError = 0; // totalError = totalError + error

int turnError; // SensorValue - DesiredValue (delta X) : Positional -> speed -> acceleration
int turnPrevError = 0; // Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed (helps with friction, battery voltage)
int turnTotalError = 0; // totalError = totalError + error

bool resetDriveSensors = false;

// Variables modified for use
bool enableDrivePID = true;

int drivePID() {

  while(enableDrivePID) {

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotor.setPosition(0,degrees);
      RightMotor.setPosition(0,degrees);
      // https://api.vexcode.cloud/v5/html/
    }

    // Positions of both motors
    int leftMotorPosition = LeftMotor.position(degrees);
    int rightMotorPosition = RightMotor.position(degrees);

    /////////////////////////////////////////////////////////////////
    // Lateral movement PID
    /////////////////////////////////////////////////////////////////

    // Average of 2 motors
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    // Potential
    error = averagePosition - desiredValue;

    // Derivative
    derivative - error - prevError;

    // Integral : Velocity -> Position -> Absement
    totalError += error;

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;

    /////////////////////////////////////////////////////////////////
    // Turning movement PID
    /////////////////////////////////////////////////////////////////

    // Average of 2 motors
    int turnDifference = leftMotorPosition - rightMotorPosition;

    // Potential
    turnError = turnDifference - desiredTurnValue;

    // Derivative
    turnDerivative - turnError - turnPrevError;

    // Integral : Velocity -> Position -> Absement
    turnTotalError += turnError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    /////////////////////////////////////////////////////////////////

    LeftMotor.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotor.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
  vex::task weWinThese(drivePID);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(100);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) { 
  enableDrivePID = false;

  while (true) {
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    LeftMotor.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RightMotor.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
