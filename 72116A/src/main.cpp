/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: CHS Robotics */
/* Created: 9/2/2023, 12:45:16 PM */
/* Description: V5 project */
/* */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <cstring>

using namespace vex;

competition Competition;
brain Brain;
controller Controller1 = controller();

motor LFront = motor(vex::PORT3);
motor LMiddle = motor(vex::PORT1);
motor LRear = motor(vex::PORT2);
motor RFront = motor(vex::PORT7);
motor RMiddle = motor(vex::PORT10);
motor RRear = motor(vex::PORT8);
digital_out wings = digital_out(Brain.ThreeWirePort.B);
digital_out jammer = digital_out(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT20);

motor Flywheel = motor(vex::PORT14);
motor ArmMotor = motor(vex::PORT12);

const int maxSize = 300;
int whatCase = 1;

class Button
{
public:
    int originX;
    int originY;
    int width;
    int height;
    color buttonColor;

    const char *text;
    int textX;
    int textY;

    Button(int x, int y, int w, int h, color c, const char *t, int X, int Y) : originX(x), originY(y), width(w), height(h), buttonColor(c), text(t), textX(X), textY(Y) {}

    void draw()
    {
        Brain.Screen.drawRectangle(originX, originY, width, height, buttonColor);
        Brain.Screen.setCursor(textY, textX);

        Brain.Screen.print(text);
        Brain.Screen.newLine();
    }

    bool isPressed()
    {
        if (Brain.Screen.pressing())
        {
            int X = Brain.Screen.xPosition(); // X pos of press
            int Y = Brain.Screen.yPosition(); // Y pos of press
            return (X >= originX && X <= originX + width) && (Y >= originY && Y <= originY + height);
        }
        return false;
    }
};

void pressButton(Button button, color color1, color color2)
{
    button.buttonColor = color1;
    button.draw();
    wait(1000, msec);
    button.buttonColor = color2;
    button.draw();
}

void pre_auton(void)
{
    Flywheel.setStopping(coast);
    ArmMotor.setStopping(hold);
    ArmMotor.setPosition(0, deg);

    // Inertial
    /*Inertial.calibrate();
    // Wait for calibration to finish
    while (Inertial.isCalibrating())
    {
        vex::task::sleep(50);
    }*/

    // Create multiple instances of the Button class
    Button button1(10, 20, 100, 50, color::blue, "Blue Offense 2", 2, 2);
    Button button2(220, 20, 100, 50, color::blue, "Blue Defense 1", 21, 2);
    Button button3(10, 160, 100, 50, color::red, "Red Offense 2", 2, 10);
    Button button4(220, 160, 100, 50, color::red, "Red Defense 1", 22, 10);
    Button button5(360, 90, 100, 50, color::purple, "Auton Skills", 36, 6);

    // Draw buttons initially
    button1.draw();
    button2.draw();
    button3.draw();
    button4.draw();
    button5.draw();

    Brain.Screen.drawImageFromFile("overunder.png", 115, 60);

    while (true)
    {
        // Check if any button is pressed
        if (button1.isPressed())
        {

            pressButton(button1, green, blue);
            whatCase = 2;
            break;
        }
        else if (button2.isPressed())
        {

            pressButton(button2, green, blue);
            whatCase = 1;
            break;
        }
        else if (button3.isPressed())
        {

            pressButton(button3, green, red);
            whatCase = 2;
            break;
        }
        else if (button4.isPressed())
        {

            pressButton(button4, green, red);
            whatCase = 1;
            break;
        }
        else if (button5.isPressed())
        {

            pressButton(button5, green, purple);
            whatCase = 3;
            break;
        }
    }
}

void stopAllMotors()
{
    LFront.stop();
    LMiddle.stop();
    LRear.stop();

    RFront.stop();
    RMiddle.stop();
    RRear.stop();

    LFront.setPosition(0, deg);
    LMiddle.setPosition(0, deg);
    LRear.setPosition(0, deg);

    RFront.setPosition(0, deg);
    RMiddle.setPosition(0, deg);
    RRear.setPosition(0, deg);

    Inertial.resetRotation();
}

class PIDController
{
public:
    double kP, kI, kD;
    double integral, previousError;

    PIDController(double p, double i, double d) : kP(p), kI(i), kD(d), integral(0), previousError(0) {}

    double calculate(double setpoint, double current)
    {
        double error = setpoint - current;
        integral += error;
        double derivative = error - previousError;

        double output = error * kP + derivative * kD + integral * kI;

        previousError = error;

        return output;
    }

    void reset()
    {
        integral = 0;
        previousError = 0;
    }
};

// Define PID controllers & tune
PIDController leftDrivePID(0.38, 0.001, 0.45);
PIDController rightDrivePID(0.38, 0.001, 0.45);

/*---------------------------------------------------------------------------*/
/* */
/* Autonomous Task */
/* */
/* This task is used to control your robot during the autonomous phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/

void move(std::string dir, int velocity, double time)
{
    LFront.setVelocity(velocity, vex::percent);
    LMiddle.setVelocity(velocity, vex::percent);
    LRear.setVelocity(velocity, vex::percent);

    RFront.setVelocity(velocity, vex::percent);
    RMiddle.setVelocity(velocity, vex::percent);
    RRear.setVelocity(velocity, vex::percent);
    double slowConstant = 0.5;

    if (dir == "forward")
    {
        LFront.spin(forward);
        LMiddle.spin(reverse);
        LRear.spin(forward);

        RFront.spin(reverse);
        RMiddle.spin(forward);
        RRear.spin(reverse);
    }
    else if (dir == "reverse")
    {
        LFront.spin(reverse);
        LMiddle.spin(forward);
        LRear.spin(reverse);

        RFront.spin(forward);
        RMiddle.spin(reverse);
        RRear.spin(forward);
    }

    // Slow down a bit
    wait(time - slowConstant, sec);
    double div = 2;
    LFront.setVelocity(velocity / div, vex::percent);
    LMiddle.setVelocity(velocity / div, vex::percent);
    LRear.setVelocity(velocity / div, vex::percent);

    RFront.setVelocity(velocity / div, vex::percent);
    RMiddle.setVelocity(velocity / div, vex::percent);
    RRear.setVelocity(velocity / div, vex::percent);

    if (dir == "forward")
    {
        LFront.spin(forward);
        LMiddle.spin(reverse);
        LRear.spin(forward);

        RFront.spin(reverse);
        RMiddle.spin(forward);
        RRear.spin(reverse);
    }
    else if (dir == "reverse")
    {
        LFront.spin(reverse);
        LMiddle.spin(forward);
        LRear.spin(reverse);

        RFront.spin(forward);
        RMiddle.spin(reverse);
        RRear.spin(forward);
    }

    wait(slowConstant, sec);
    stopAllMotors();
    wait(60, msec);
}

// 3.14 * 4" / 360 = .0349 inches per degree
void PIDmove(double distance)
{
    double threshold = 0.2;
    double terminationThreshold = 1;
    double strikes = 0;

    vex::timer timer;
    timer.clear();

    // Average motor position
    double average = (LFront.position(deg) + LMiddle.position(deg) + LRear.position(deg) + RFront.position(deg) + RMiddle.position(deg) + RRear.position(deg)) / 6;
    double avgMotor = average * 0.0349;

    while (!(avgMotor > std::abs(distance) - threshold && avgMotor < std::abs(distance) + threshold))
    {
        // Check timeout
        if (timer.time(sec) >= 2.0)
        {
            break; // Exit the loop if timeout is reached
        }

        average = (LFront.position(deg) + LMiddle.position(deg) + LRear.position(deg) + RFront.position(deg) + RMiddle.position(deg) + RRear.position(deg)) / 6;
        avgMotor = average * 0.0349;

        double moveVolts = leftDrivePID.calculate(distance, avgMotor);
        // Controller1.Screen.print(avgMotor);
        // Controller1.Screen.newLine();

        // Motors
        LFront.spin(forward, moveVolts, voltageUnits::volt);
        LMiddle.spin(reverse, moveVolts, voltageUnits::volt);
        LRear.spin(forward, moveVolts, voltageUnits::volt);

        RFront.spin(reverse, moveVolts, voltageUnits::volt);
        RMiddle.spin(forward, moveVolts, voltageUnits::volt);
        RRear.spin(reverse, moveVolts, voltageUnits::volt);

        wait(70, msec);

        if (std::abs(moveVolts) < terminationThreshold)
        {
            if (strikes >= 200)
            {
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

void PIDturn(double degree)
{
    double threshold = 0.2;
    double terminationThreshold = 1;
    double strikes = 0;

    vex::timer timer;
    timer.clear(); // Clear the timer at the beginning

    while (!(Inertial.rotation() > std::abs(degree) - threshold && Inertial.rotation() < std::abs(degree) + threshold))
    {
        // Check timeout
        if (timer.time(sec) >= 2.0)
        {
            break; // Exit the loop if timeout is reached
        }

        double leftTurnVolts = leftDrivePID.calculate(degree, Inertial.rotation());
        double rightTurnVolts = rightDrivePID.calculate(degree, Inertial.rotation());
        Controller1.Screen.print(Inertial.rotation());
        Controller1.Screen.newLine();

        // Motors
        LFront.spin(forward, -leftTurnVolts, voltageUnits::volt);
        LMiddle.spin(reverse, -leftTurnVolts, voltageUnits::volt);
        LRear.spin(forward, -leftTurnVolts, voltageUnits::volt);

        RFront.spin(reverse, rightTurnVolts, voltageUnits::volt);
        RMiddle.spin(forward, rightTurnVolts, voltageUnits::volt);
        RRear.spin(reverse, rightTurnVolts, voltageUnits::volt);

        wait(70, msec);

        if (std::abs(leftTurnVolts) < terminationThreshold)
        {
            if (strikes >= 200)
            {
                strikes = 0;
                break;
            }
            strikes++;
        }
    }

    // Controller1.Screen.newLine();
    // Controller1.Screen.print(Inertial.rotation());
    leftDrivePID.reset();
    rightDrivePID.reset();
    stopAllMotors();
    wait(25, msec);
}

void autonomous(void)
{
    int def = 60;
    wings.set(false);
    jammer.set(true);

    /*// Inertial
    Inertial.calibrate();
    // Wait for calibration to finish
    while (Inertial.isCalibrating())
    {
        vex::task::sleep(50);
    }*/

    switch (whatCase)
    {
      //////////////////////
     //    MAIN CASES    //
    //////////////////////


    // Launching Side Win Point
    case 1:
        ArmMotor.spin(forward, 12, volt);
        wait(0.1, sec);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        wait(1.1, sec);
        ArmMotor.resetPosition();
        
        move("reverse", def, 0.7);
        wings.set(true);
        move("forward", def, 0.3);
        wings.set(false);
        PIDturn(10);
        move("reverse", def, 0.3);
        ArmMotor.setStopping(hold);

        // Go to goal
        PIDturn(-30);
        move("forward", def, 0.8);
        PIDturn(-25);
        ArmMotor.spinToPosition(200, deg);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        Flywheel.spin(forward, 8, volt);
        wait(0.5, sec);
        move("forward", def, 0.8);
        ArmMotor.setStopping(hold);
        ArmMotor.spinToPosition(500, deg);
        move("forward", def, 0.25);
        wait(1, sec);
        Flywheel.stop();
        break;

    // Goal Side Win Point
    case 2:
        ArmMotor.spin(forward, 12, volt);
        wait(0.1, sec);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        wait(1.1, sec);

        // First Triball
        move("forward", def, 1.5);
        PIDturn(42);
        ArmMotor.resetPosition();
        Flywheel.spin(forward, 5, volt);
        ArmMotor.spinToPosition(100, degrees);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        wait(0.7, sec);
        Flywheel.stop();

        // Second Triball
        PIDturn(-60);
        ArmMotor.spinToPosition(160, degrees);
        Flywheel.spin(reverse, 8, volt);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        move("forward", def, 1.0);
        Flywheel.stop();
        wait(0.8, sec);

        PIDturn(105);
        Flywheel.spin(forward, 4, volt);
        ArmMotor.spinToPosition(100, degrees);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();
        wait(1.2, sec);
        Flywheel.stop();

        // Final
        PIDturn(10);
        wings.set(true);
        move("forward", def, 1.2);
        move("reverse", def, 0.7);

        ArmMotor.stop();
        wings.set(false);
        break;

    // Autonomous Skills
    case 3:
        move("forward", def, 0.9);
        PIDturn(119);
        move("reverse", def, 0.2);

        Flywheel.spin(reverse, 11, volt);
        ArmMotor.setVelocity(100, percent);
        ArmMotor.setStopping(hold);
        //ArmMotor.setPosition(400, degrees);
        //ArmMotor.spinToPosition(550, degrees);
        //ArmMotor.spinToPosition(300, degrees);

        // Done
        wait(35, sec);

        PIDturn(55);
        Flywheel.stop();
        ArmMotor.spinToPosition(100, degrees);
        ArmMotor.setStopping(coast);
        ArmMotor.stop();

        move("forward", def, 1.3);
        PIDturn(-32);
        move("forward", def, 2.5);
        PIDturn(-125);
        move("forward", def, 1.7);
        PIDturn(40);
        wings.set(true);
        PIDturn(30);
        move("forward", def, 0.8);
        PIDturn(30);
        move("forward", def, 0.8);
        PIDturn(30);
        move("forward", def, 0.8);
        move("reverse", def, 1.9);

        break;
    

      //////////////////////
     //    SAFE CASES    //
    //////////////////////


    // Launching side: safe case
    case 4:
        move("forward", def, 1.8);
        PIDturn(-90.0);
        ArmMotor.spinToPosition(180, degrees);
        Flywheel.spin(forward, 12, volt);
        wait(0.1, sec);
        ArmMotor.setStopping(coast);
        move("forward", def, 0.7);

        ArmMotor.stop();
        break;

    // Goal side: safe case
    case 5:
        move("forward", def, 1.8);
        PIDturn(90.0);
        ArmMotor.spinToPosition(180, degrees);
        Flywheel.spin(forward, 12, volt);
        wait(0.1, sec);
        ArmMotor.setStopping(coast);
        move("forward", def, 0.7);

        ArmMotor.stop();
        break;

    // Same side: safe case
    case 6:
      move("reverse", def, 1.0);
      PIDturn(35.0);
      move("reverse", def, 0.9);
      move("forward", def, 0.6);
      PIDturn(40.0);
      move("reverse", def, 0.95);
      move("forward", def, 0.7);
      PIDturn(230.0);
      ArmMotor.spin(forward, 12.0, voltageUnits::volt);
      wait(2, sec);

      ArmMotor.stop();
      break;

    // Opposite side: safe case
    case 7:
      move("reverse", def, 1.0);
      PIDturn(-35.0);
      move("reverse", def, 0.95);
      move("forward", def, 0.6);
      PIDturn(-40.0);
      move("reverse", def, 1.0);
      move("forward", def, 0.7);
      PIDturn(-230.0);
      break;

      //////////////////////
     //    TEST CASES    //
    //////////////////////

    // Testing
    case 8:
        PIDturn(0.0);
        move("forward", def, 1);
        // PIDmove(12);
        wait(1, sec);
        PIDturn(90.0);
        move("forward", def, 1);
        PIDturn(90.0);
        move("forward", def, 1);
        break;

    // Re-Run Auton
    case 9:
        int turnRoute[maxSize] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-6,-26,-45,-49,-49,-51,-51,-51,-51,-40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-29,-55,-70,-100,-100,-100,-12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,5,13,16,16,15,15,14,0,0,0,0,0,0,0,0,0,0,-4,-7,-12,-14,-14,-14,-18,-20,-34,-37,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        int movementRoute[maxSize] = {0,0,0,0,0,0,0,0,0,4,9,19,27,30,30,30,0,0,-29,-40,-40,-40,-42,-46,-45,-52,-52,-55,-3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,55,56,57,57,57,57,57,57,57,57,57,57,57,57,57,14,0,0,0,-41,-48,-67,-77,-75,-78,-42,0,0,0,0,0,0,0,0,0,0,0,0,-17,-24,-25,-24,-18,0,40,48,57,57,57,57,57,57,57,51,47,40,22,7,7,5,7,14,26,37,37,33,20,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        int currentTime = 0;

        while (currentTime <= maxSize)
        {
            double turnVal = turnRoute[currentTime];
            double forwardVal = movementRoute[currentTime];
            currentTime++;

            double turnImportance = -0.1;

            double turnVolts = turnVal * 0.12;
            double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts) / 12.0) * turnImportance);

            // Motors
            LFront.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
            LMiddle.spin(reverse, forwardVolts - turnVolts, voltageUnits::volt);
            LRear.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

            RFront.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
            RMiddle.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
            RRear.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);

            wait(100, msec);
            Controller1.Screen.clearScreen();
        }
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/* User Control Task                                                         */
/*                                                                           */
/* This task is used to control your robot during the user control phase of  */
/* a VEX Competition.                                                        */
/*                                                                           */
/* You must modify the code to add your own robot specific commands here.    */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
    bool goForward = true;

    bool flipControls = false;
    bool rerun = true;
    bool wingsOn = false;
    bool jammerOn = false;
    wings.set(false);
    jammer.set(true);

    int turns[maxSize] = {};
    int movements[maxSize] = {};
    int currentIndex = 0;

    while (true)
    {
        double turnVal = Controller1.Axis1.position(percent);
        double forwardVal = Controller1.Axis3.position(percent);

        if (flipControls)
        {
            forwardVal = -forwardVal;
        }

        double turnImportance = -0.3;

        double turnVolts = turnVal * 0.12;
        double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts) / 12.0) * turnImportance);

        // Motors
        LFront.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
        LMiddle.spin(reverse, forwardVolts - turnVolts, voltageUnits::volt);
        LRear.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

        RFront.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
        RMiddle.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
        RRear.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);

        // Temperature (130, 140, 150, 160 °F)

        int motorTemps[6];
        motorTemps[0] = LFront.temperature(temperatureUnits::fahrenheit);
        motorTemps[1] = LMiddle.temperature(temperatureUnits::fahrenheit);
        motorTemps[2] = LRear.temperature(temperatureUnits::fahrenheit);
        motorTemps[3] = RFront.temperature(temperatureUnits::fahrenheit);
        motorTemps[4] = RMiddle.temperature(temperatureUnits::fahrenheit);
        motorTemps[5] = RRear.temperature(temperatureUnits::fahrenheit);

        int *maxElement = std::max_element(std::begin(motorTemps), std::end(motorTemps));

        if (*maxElement >= 130)
        {
            Controller1.Screen.print("WARNING: ");
        }
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print(*maxElement);
        Controller1.Screen.print("°F");
        //FController1.Screen.print(LFront.position(deg));
        Controller1.Screen.newLine();

        // Motors Installed
        motor motorList[] = {LFront, LMiddle, LRear, RFront, RMiddle, RRear};
        const char *motorNames[] = {"LFront", "LMiddle", "LRear", "RFront", "RMiddle", "RRear"};

        for (int i = 0; i < 6; ++i)
        {
            if (!motorList[i].installed())
            {
                Controller1.Screen.setCursor(1, 1);
                Controller1.Screen.print(motorNames[i]);
                Controller1.Screen.print(" Disconnected!");
                Controller1.Screen.newLine();
                break;
            }
        }

        // Re-run Auton
        /*if (rerun) {
        turns[currentIndex] = turnVal;
        movements[currentIndex] = forwardVal;
        currentIndex++;
        Controller1.Screen.print(currentIndex);
        }

        // Un-comment to activate
        if (currentIndex >= maxSize && rerun) {
        // Output arrays to console log
        std::cout << "Turns array: ";
        for (int i = 0; i < currentIndex; i++) {
        std::cout << turns[i] << ",";
        }
        std::cout << std::endl;

        std::cout << "Movements array: ";
        for (int i = 0; i < currentIndex; i++) {
        std::cout << movements[i] << ",";
        }
        std::cout << std::endl;
        rerun = false;
        }*/

        // Buttons
        /*if (Controller1.ButtonY.pressing()) {
        flipControls = !flipControls;
        wait(500, msec);
        }*/

        // Arm Motor Positions
        if (Controller1.ButtonUp.pressing())
        {
            ArmMotor.setVelocity(100, vex::percent);
            ArmMotor.spinToPosition(1400, deg);
        }
        if (ArmMotor.position(deg) < 1000 && Flywheel.current() <= 0.01) {
        ArmMotor.setStopping(coast);
        } else {
        ArmMotor.setStopping(hold);
        }

        // Bar Lift
        ArmMotor.setVelocity(60, vex::percent);

        if (Controller1.ButtonL1.pressing())
        {
            ArmMotor.spin(forward, 12.0, voltageUnits::volt);
        }
        else if (Controller1.ButtonL2.pressing())
        {
            ArmMotor.spin(forward, -12.0, voltageUnits::volt);
        }
        else
        {
            ArmMotor.stop();
        }

        // Flywheel
        Flywheel.spin(forward, 10, volt);

        if (Controller1.ButtonR1.pressing())
        {
            Flywheel.spin(forward, 12.0, voltageUnits::volt);
        }
        else if (Controller1.ButtonR2.pressing())
        {
            Flywheel.spin(forward, -12.0, voltageUnits::volt);
        }
        else
        {
            Flywheel.stop();
        }

        // Pneumatics
        if (Controller1.ButtonX.pressing())
        {
            if (wingsOn)
            {
                wings.set(true);
            }
            else
            {
                wings.set(false);
            }
            wingsOn = !wingsOn;
            wait(1000, msec);
        }

        // Jammer
        if (Controller1.ButtonA.pressing())
        {
            if (jammerOn)
            {
                jammer.set(true);
            }
            else
            {
                jammer.set(false);
            }
            jammerOn = !jammerOn;
            wait(1000, msec);
        }

        wait(10, msec);
        Controller1.Screen.clearScreen();
    }
}

int main()
{
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true)
    {
        wait(50, msec);
    }
}
