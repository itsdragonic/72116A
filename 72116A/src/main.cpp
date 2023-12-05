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
motor LMiddle = motor(vex::PORT8);
motor LRear = motor(vex::PORT5);
motor RFront = motor(vex::PORT4);
motor RMiddle = motor(vex::PORT2);
motor RRear = motor(vex::PORT1);
digital_out wings = digital_out(Brain.ThreeWirePort.G);
digital_out jammer = digital_out(Brain.ThreeWirePort.H);
inertial Inertial = inertial(PORT11);

motor ArmMotor = motor(vex::PORT17);
motor BarLift = motor(vex::PORT16);

void pre_auton(void) {
  ArmMotor.setStopping(hold);
  BarLift.setStopping(hold);
}

void stopAllMotors() {
  LFront.stop();
  LMiddle.stop();
  LRear.stop();
  
  RFront.stop();
  RMiddle.stop();
  RRear.stop();
}

class PIDController {
public:
  double kP, kI, kD;
  double integral, previousError;

  PIDController(double p, double i, double d) : kP(p), kI(i), kD(d), integral(0), previousError(0) {}

  double calculate(double setpoint, double current) {
    double error = setpoint - current;
    integral += error;
    double derivative = error - previousError;

    double output = error * kP + derivative * kD + integral * kI;

    previousError = error;

    return output;
  }

  void reset() {
    integral = 0;
    previousError = 0;
  }
};

// Define PID controllers & tune
PIDController leftDrivePID(0.38, 0.001, 0.45);
PIDController rightDrivePID(0.38, 0.001, 0.45);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void move(std::string dir, int velocity, double time) {
  LFront.setVelocity(velocity, vex::percent);
  LMiddle.setVelocity(velocity, vex::percent);
  LRear.setVelocity(velocity, vex::percent);

  RFront.setVelocity(velocity, vex::percent);
  RMiddle.setVelocity(velocity, vex::percent);
  RRear.setVelocity(velocity, vex::percent);
  double slowConstant = 0.5;

  if (dir == "reverse") {
    LFront.spin(forward);
    LMiddle.spin(forward);
    LRear.spin(forward);

    RFront.spin(reverse);
    RMiddle.spin(reverse);
    RRear.spin(reverse);
  }
  else if (dir == "forward") {
    LFront.spin(reverse);
    LMiddle.spin(reverse);
    LRear.spin(reverse);

    RFront.spin(forward);
    RMiddle.spin(forward);
    RRear.spin(forward);
  }

  // Slow down a bit
  wait(time - slowConstant, sec);
  double div = 2;
  LFront.setVelocity(velocity/div, vex::percent);
  LMiddle.setVelocity(velocity/div, vex::percent);
  LRear.setVelocity(velocity/div, vex::percent);

  RFront.setVelocity(velocity/div, vex::percent);
  RMiddle.setVelocity(velocity/div, vex::percent);
  RRear.setVelocity(velocity/div, vex::percent);

  if (dir == "reverse") {
    LFront.spin(forward);
    LMiddle.spin(forward);
    LRear.spin(forward);

    RFront.spin(reverse);
    RMiddle.spin(reverse);
    RRear.spin(reverse);
  }
  else if (dir == "forward") {
    LFront.spin(reverse);
    LMiddle.spin(reverse);
    LRear.spin(reverse);

    RFront.spin(forward);
    RMiddle.spin(forward);
    RRear.spin(forward);
  }

  wait(slowConstant, sec);
  stopAllMotors();
  wait(60, msec);
}

void PIDturn(double degree) {
  double threshold = 0.2;
  double terminationThreshold = 1;
  double strikes = 0;

  vex::timer timer;
  timer.clear();  // Clear the timer at the beginning

  while (!(Inertial.rotation() > std::abs(degree) - threshold && Inertial.rotation() < std::abs(degree) + threshold)) {
    // Check timeout
    if (timer.time(sec) >= 2.0) {
      break;  // Exit the loop if timeout is reached
    }

    double leftTurnVolts = leftDrivePID.calculate(degree, Inertial.rotation());
    double rightTurnVolts = rightDrivePID.calculate(degree, Inertial.rotation());

    // Motors
    LFront.spin(forward, -leftTurnVolts, voltageUnits::volt);
    LMiddle.spin(forward, -leftTurnVolts, voltageUnits::volt);
    LRear.spin(forward, -leftTurnVolts, voltageUnits::volt);

    RFront.spin(reverse, rightTurnVolts, voltageUnits::volt);
    RMiddle.spin(reverse, rightTurnVolts, voltageUnits::volt);
    RRear.spin(reverse, rightTurnVolts, voltageUnits::volt);

    wait(70, msec);

    if (std::abs(leftTurnVolts) < terminationThreshold) {
      if (strikes >= 8) {
        strikes = 0;
        break;
      }
      strikes++;
    }
  }

  leftDrivePID.reset();
  rightDrivePID.reset();
  stopAllMotors();
  wait(25, msec);
}

void autonomous(void) {
  int whatCase = 1;
  int def = 60;
  wings.set(false);
  jammer.set(true);
  
  Inertial.calibrate();
  // Wait for calibration to finish
  while (Inertial.isCalibrating()) {
    vex::task::sleep(50);
  }

  switch (whatCase) {
    // Same side: safe case
    case 1:
      move("reverse", def, 1.2);
      PIDturn(-25.0);
      move("reverse", def, 0.9);
      move("forward", def, 0.7);
      PIDturn(-30.0);
      move("reverse", def, 1);
      PIDturn(-220.0);
      move("reverse", def, 0.7);
      break;

    // Opposite side: safe case
    case 2:
      move("reverse", def, 1.28);
      PIDturn(25.0);
      move("reverse", def, 0.64);
      move("forward", def, 0.55);
      PIDturn(30.0);
      move("reverse", def, 0.7);
      PIDturn(200.0);
      move("reverse", def, 0.7);
      BarLift.spin(forward, 5.0, voltageUnits::volt);
      wait(2, sec);

      BarLift.stop();
      break;

    // Autonomous Skills (Blue Bar)
    case 3:
      BarLift.spin(forward, 5.0, voltageUnits::volt);
      wait(1.3, sec);

      BarLift.stop();
      ArmMotor.spin(forward, -8.0, voltageUnits::volt);
      wait(10, sec);
      ArmMotor.stop();
      BarLift.setStopping(coast);
      ArmMotor.setStopping(coast);

      move("forward", def, 1.4);
      PIDturn(-35.0);
      move("forward", 100, 1.86);
      PIDturn(33.0);
      wings.set(true);
      move("forward", def, 0.8);
      break;

    // Autonomous Skills (Red Bar)
    case 4:
      BarLift.spin(forward, 8.0, voltageUnits::volt);
      wait(2.12, sec);

      BarLift.stop();
      ArmMotor.spin(forward, -9.0, voltageUnits::volt);
      wait(54, sec);
      ArmMotor.stop();
      BarLift.setStopping(coast);
      ArmMotor.setStopping(coast);

      move("forward", def, 1.4);
      PIDturn(-35.0);
      move("forward", 100, 1.86);
      PIDturn(33.0);
      wings.set(true);
      move("forward", def, 0.8);
      break;

    // AWP Blue
    case 5:
      wings.set(true);
      move("forward", def, 0.6);
      PIDturn(10.0);
      wings.set(false);
      move("forward", def, 0.8);
      PIDturn(35.0);
      move("forward", def, 0.8);
      move("reverse", def, 1.9);
      /*PIDturn(150.0);
      move("forward", def, 1.3);
      PIDturn(130.0);
      move("forward", def, 2);*/
      break;

    // Testing
    case 6:
      PIDturn(0.0);
      move("forward", def, 2);
      wait(1, sec);
      PIDturn(180.0);
      move("forward", def, 2);
      PIDturn(0.0);
      break;
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
  bool goForward = true;

  bool flipControls = false;
  bool wingsOn = false;
  bool jammerOn = false;
  wings.set(false);
  jammer.set(true);

  while (true) {
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = -Controller1.Axis3.position(percent);

    if (flipControls) {
      forwardVal = -forwardVal;
    }

    double turnImportance = -0.1;

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    // Motors
    LFront.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    LMiddle.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
    LRear.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    RFront.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
    RMiddle.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
    RRear.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);

    // Buttons
    if (Controller1.ButtonR2.pressing()) {
      flipControls = !flipControls;
      wait(500, msec);
    }

    // Arm Launcher
    ArmMotor.setVelocity(30, vex::percent);

    if (Controller1.ButtonL1.pressing()) {
      ArmMotor.spin(forward, 10.0, voltageUnits::volt);
    } else if (Controller1.ButtonL2.pressing()) {
      ArmMotor.spin(forward, -10.0, voltageUnits::volt);
    } else {
      ArmMotor.stop();
    }

    // Bar Lift
    BarLift.setVelocity(60, vex::percent);

    if (Controller1.ButtonX.pressing()) {
      BarLift.spin(forward, 30.0, voltageUnits::volt);
    } else if (Controller1.ButtonB.pressing()) {
      BarLift.spin(forward, -30.0, voltageUnits::volt);
    } else {
      BarLift.stop();
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

    // Jammer
    if (Controller1.ButtonA.pressing()) {
      if (jammerOn) {
        jammer.set(true);
      } else {
        jammer.set(false);
      }
      jammerOn = !jammerOn;
      wait(500, msec);
    }

    wait(10, msec);
    Controller1.Screen.clearScreen();
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
