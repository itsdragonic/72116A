/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       CHS Robotics                                              */
/*    Created:      9/2/2023, 12:45:16 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <string>

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller();

motor LFront = motor(vex::PORT7);
motor LMiddle = motor(vex::PORT6);
motor LRear = motor(vex::PORT5);
motor RFront = motor(vex::PORT4);
motor RMiddle = motor(vex::PORT2);
motor RRear = motor(vex::PORT1);

//motor_group leftSide(LFront, LMiddle, LRear);
//motor_group rightSide(RFront, RMiddle, RRear);

digital_out wings = digital_out(Brain.ThreeWirePort.A);

//inertial Inertial = inertial(PORT17);

// Pneumatics
//wings.set(true)

motor ArmMotor = motor(vex::PORT16);

void pre_auton(void) {
  ArmMotor.setStopping(hold);
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
      LFront.setPosition(0,degrees);
      LMiddle.setPosition(0,degrees);
      LRear.setPosition(0,degrees);

      RFront.setPosition(0,degrees);
      RMiddle.setPosition(0,degrees);
      RRear.setPosition(0,degrees);

      // https://api.vexcode.cloud/v5/html/
    }

    // Positions of both motors
    int leftMotorPosition = LFront.position(degrees);
    int rightMotorPosition = RFront.position(degrees);

    /////////////////////////////////////////////////////////////////
    // Lateral movement PID
    /////////////////////////////////////////////////////////////////

    // Average of 2 motors
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;

    // Potential
    error = averagePosition - desiredValue;

    // Derivative
    derivative = error - prevError;

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
    turnDerivative = turnError - turnPrevError;

    // Integral : Velocity -> Position -> Absement
    turnTotalError += turnError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;

    /////////////////////////////////////////////////////////////////

    LFront.spin(forward, -lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LMiddle.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LRear.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);

    RFront.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RMiddle.spin(forward, -lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RRear.spin(forward, -lateralMotorPower + turnMotorPower, voltageUnits::volt);

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
void stopAllMotors() {
  LFront.stop();
  LMiddle.stop();
  LRear.stop();
  
  RFront.stop();
  RMiddle.stop();
  RRear.stop();
}

void moveDrivetrain(std::string dir, int velocity, double time) {
  LFront.setVelocity(velocity, vex::percent);
  LMiddle.setVelocity(velocity, vex::percent);
  LRear.setVelocity(velocity, vex::percent);

  RFront.setVelocity(velocity, vex::percent);
  RMiddle.setVelocity(velocity, vex::percent);
  RRear.setVelocity(velocity, vex::percent);

  //double finalAmount = (static_cast<double>(time) / 90) * 0.5;
  // 1 second = 70-80 cm
  
  double finalAmount = static_cast<double>(time) * 1000;
  Brain.Screen.print(finalAmount);
  Brain.Screen.newLine();

  if (dir == "reverse") {
    LFront.spin(reverse);
    LMiddle.spin(forward);
    LRear.spin(forward);

    RFront.spin(forward);
    RMiddle.spin(reverse);
    RRear.spin(reverse);
  }
  else if (dir == "forward") {
    LFront.spin(forward);
    LMiddle.spin(reverse);
    LRear.spin(reverse);

    RFront.spin(reverse);
    RMiddle.spin(forward);
    RRear.spin(forward);
  }

  wait(finalAmount, msec);
  stopAllMotors();
}

// turning
void turnDrivetrain(std::string dir, int velocity, double amount) {
  LFront.setVelocity(velocity, vex::percent);
  LMiddle.setVelocity(velocity, vex::percent);
  LRear.setVelocity(velocity, vex::percent);

  RFront.setVelocity(velocity, vex::percent);
  RMiddle.setVelocity(velocity, vex::percent);
  RRear.setVelocity(velocity, vex::percent);

  double finalAmounts = (static_cast<double>(amount) / 90.0) * 0.4;
  Brain.Screen.print(finalAmounts);
  Brain.Screen.newLine();

  if (dir == "left") {
    LFront.spin(reverse);
    LMiddle.spin(forward);
    LRear.spin(forward);

    RFront.spin(reverse);
    RMiddle.spin(forward);
    RRear.spin(forward);
  }
  else if (dir == "right") {
    LFront.spin(forward);
    LMiddle.spin(reverse);
    LRear.spin(reverse);

    RFront.spin(forward);
    RMiddle.spin(reverse);
    RRear.spin(reverse);
  }

  wait(finalAmounts, sec);
  stopAllMotors();
}


void autonomous(void) {
  bool sameSide = false;
  wings.set(true);
  /*vex::task weWinThese(drivePID);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 600;

  vex::task::sleep(100);

  resetDriveSensors = true;
  desiredValue = 300;
  desiredTurnValue = 300;*/

  int def = 60;
  if (sameSide) {
    moveDrivetrain("forward", 80, 1);
    wait(100, msec);
    turnDrivetrain("left", def, 35);
    wait(200, msec);
    moveDrivetrain("reverse", def, 1.1);
    wait(150, msec);
    turnDrivetrain("left", def, 60);
    wait(200, msec);
    moveDrivetrain("forward", def, 1.5);
    wait(150, msec);
  }
  else {
    moveDrivetrain("forward", 80, 1);
    wait(100, msec);
    turnDrivetrain("right", def, 35);
    wait(200, msec);
    moveDrivetrain("reverse", def, 0.92);
    wait(150, msec);
    turnDrivetrain("right", def, 120);
    wait(200, msec);
    moveDrivetrain("forward", def, 1.65);
  }
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
  bool goForward = true;

  bool flipControls = false;
  bool wingsOn = false;
  wings.set(true);

  while (true) {
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);

    if (flipControls) {
      forwardVal = forwardVal * -1;
    }

    // subject to change
    double turnImportance = -0.2;

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    // Motors
    LFront.spin(reverse, forwardVolts - turnVolts, voltageUnits::volt);
    LMiddle.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    LRear.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    RFront.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
    RMiddle.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
    RRear.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);

    // Buttons
    if (Controller1.ButtonR2.pressing()) {
      flipControls = !flipControls;
      wait(500, msec);
    }

    // Arm Launcher
    ArmMotor.setVelocity(40, vex::percent);

    if (Controller1.ButtonL1.pressing()) {
      ArmMotor.spin(forward, 30.0, voltageUnits::volt);
    } else if (Controller1.ButtonL2.pressing()) {
      ArmMotor.spin(forward, -30.0, voltageUnits::volt);
    } else {
      ArmMotor.stop();
    }

    // Pneumatics
    if (Controller1.ButtonR1.pressing()) {
      if (wingsOn) {
        wings.set(true);
      } else {
        wings.set(false);
      }
      wingsOn = !wingsOn;
      wait(500, msec);
    }

    wait(20, msec);
  }
}

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
