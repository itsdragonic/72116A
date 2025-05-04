#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup leftMotors({-11, 12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-14,16, 17}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Other Motors
pros::Motor arm(8);
pros::Motor conveyor(3);
pros::Motor intake(9);

// Pneumatics
pros::ADIDigitalOut latch('B');
pros::ADIDigitalOut elevation('A'); // LB
pros::ADIDigitalOut pto('C'); // PTO

// Sensors
pros::Imu imu(21);
pros::Optical optical(4);

// Important Variables
bool alliance = false; // true means blue, false means red
int autonSide = 2; // 1 is positive, -1 is negative, 0 is skills
int autonRoute = 6;

bool colorSorting = true;
bool activated = false;
bool detectBlockage = false;
bool allianceStake = true;
bool touchLadder = false;
bool primed = false;

// Tracking wheels

pros::Rotation horizontalEnc(15);
pros::Rotation verticalEnc1(-10);
pros::Rotation verticalEnc2(1);

// 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5);
lemlib::TrackingWheel vertical1(&verticalEnc1, lemlib::Omniwheel::NEW_2, -4.55);
lemlib::TrackingWheel vertical2(&verticalEnc2, lemlib::Omniwheel::NEW_2, 4.55);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.4, // track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              480, // drivetrain rpm
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                            50, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             5 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical1, // vertical tracking wheel
                            &vertical2, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 int maxSpeed = 120;
 int conveyorSpeed = maxSpeed;
 int speedUp = -700;
 int speedUp2 = 0;
 int spinConveyor = 0;
 
 int Clock = 100;
 int counter = 0;
 
 void conveyorChecking() {
     while (true) {
         conveyorSpeed = maxSpeed;
 
         // fix conveyor if stuck
         if (detectBlockage && !primed) {
             if (Clock > 0) {
                 Clock --;
             } else {
                 Clock = 500;
                 counter = 0;
             }
 
             if (abs(conveyor.get_target_velocity()) > 0 && abs(conveyor.get_actual_velocity()) < 10) {
                 counter ++;
             }
 
             //controller.print(0,0, ": %8.2f", counter);
 
             if (counter > 12) {
                 controller.rumble(".");
                 conveyor.move(-80);
                 pros::delay(500);
                 conveyor.move(0);
                 Clock = 0;
             }
         }
 
         // Optical Sensing
         if (speedUp > -100) {
             speedUp --;
         }
 
         if (speedUp < 0 && speedUp > -40) {
             conveyorSpeed = -100;
         }
         if (speedUp < -80) {
            activated = false;
        }
 
         if (speedUp2 > 0) {
             speedUp2 --;
             conveyorSpeed = maxSpeed;
         }
 
         optical.set_led_pwm(100);
         if (colorSorting && !activated && optical.get_saturation() > 0.2) {
             // color sorting
             if (alliance && ((optical.get_hue() > 345) || (optical.get_hue() < 15))) {
                 speedUp = 17;
                 activated = true;
             }
             if (!alliance && ((optical.get_hue() > 210) && (optical.get_hue() < 230))) {
                 speedUp = 17;
                 activated = true;
             }
             
             // speed up
             /*if (!alliance && ((optical.get_hue() > 345) || (optical.get_hue() < 15))) {
                 speedUp2 = 23;
             }
             if (alliance && ((optical.get_hue() > 210) && (optical.get_hue() < 230))) {
                 speedUp2 = 23;
             }*/
         }
 
         // Spin conveyor
         switch (spinConveyor) {
             case 1:
                 conveyor.move(conveyorSpeed);
                 intake.move(conveyorSpeed);
                 break;
             case -1:
                 conveyor.move(-conveyorSpeed);
                 intake.move(-conveyorSpeed);
                 break;
             case 0:
                 conveyor.move(0);
                 intake.move(0);
                 break;
         }
 
         pros::delay(10);
     }
 }

 const std::string autonBlurbs[8] = {
    "Blue, negative side",      // Auton 1
    "Red, negative side",       // Auton 2
    "Red, positive side",       // Auton 3
    "Blue, positive side",      // Auton 4
    "Auton Skills",             // Auton 5
    "Test 1",                   // Auton 6
    "Test 2",                   // Auton 7
    "Test 3"                    // Auton 8
};
 
void on_left_button() {
    alliance = !alliance;
    if (alliance) {
        pros::lcd::set_text(3, "Blue Team Selected");
    } else {
        pros::lcd::set_text(3, "Red Team Selected");
    }
}

void on_center_button() {
    autonRoute = (autonRoute % 8) + 1;

    std::string blurb = autonBlurbs[autonRoute - 1];

    pros::lcd::set_text(4, "Auton " + std::to_string(autonRoute) + " Selected (" + blurb + ")");
}

int configIndex = 0;
void on_right_button() {
    configIndex = (configIndex + 1) % 4;

    // Update booleans based on configIndex
    allianceStake = configIndex & 0b10; // Bit 1
    touchLadder = configIndex & 0b01;   // Bit 0

    pros::lcd::print(5, "Get Alliance Stake: %s", allianceStake ? "true" : "false");
    pros::lcd::print(6, "Touch Ladder: %s", touchLadder ? "true" : "false");
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::delay(3000);

    pros::Task conveyor_task(conveyorChecking);

    pros::lcd::register_btn0_cb(on_left_button); // alliance color
    pros::lcd::register_btn1_cb(on_center_button); // auton path
    pros::lcd::register_btn2_cb(on_right_button); // alliance stake & ladder
    pros::lcd::print(3, "Red Team Selected");
    pros::lcd::print(4, "Auton [Default] Selected");
    pros::lcd::print(5, "Get Alliance Stake: true");
    pros::lcd::print(6, "Touch Ladder: false");

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %.3f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %.3f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %.3f", chassis.getPose().theta); // heading

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    int def = 1500;
    int pickupTime = 2500;
    detectBlockage = true;

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    latch.set_value(false);
    elevation.set_value(false);

    // Determine corner
    if (autonSide == 0) { // auton skills
        autonRoute = 5;
    } else if (autonSide == 1) { // positive corner
        autonRoute = (alliance ? 4 : 3); // left number is blue side, right is red side
    } else if (autonSide == -1) { // negative corner
        autonRoute = (alliance ? 1 : 2); // left number is blue side, right is red side
    }

    switch (autonRoute) {
        // Blue, negative side
        case 1:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(50, 11.2, 118);
            chassis.moveToPose(69, 0, 120, 2000, {.maxSpeed = 47});
            pros::delay(900);
            arm.move(-120);
            pros::delay(700);
            arm.move(120);
            pros::delay(500);
            arm.move(0);
            chassis.moveToPose(20, 25, 116, 2500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntil(36);
            latch.set_value(true);
            //pros::delay(50);
            chassis.turnToHeading(0, def);
            spinConveyor = 1;

            // grabs
            chassis.moveToPose(24, 47, 4, def, {.earlyExitRange = 1});

            if (touchLadder) {
                chassis.moveToPose(24, 48, 4, def);
                pros::delay(3500);
                chassis.moveToPose(14, 0, 26, 2500, {.forwards = false});
                chassis.waitUntilDone();
                spinConveyor = 0;
            } else {
                chassis.turnToHeading(50, def);
                chassis.moveToPose(64.5, 67, 35, def);
                chassis.turnToHeading(40, def);
                chassis.waitUntilDone();
                
                spinConveyor = 0;
                leftMotors.move(40);
                rightMotors.move(40);
                pros::delay(500);
                //leftMotors = 5;
                //rightMotors = 5;
                spinConveyor = 1;
                pros::delay(500);

                leftMotors.move(-40);
                rightMotors.move(-40);
                pros::delay(200);
                leftMotors.move(40);
                rightMotors.move(40);
                pros::delay(400);
                leftMotors.move(5);
                rightMotors.move(5);

                pros::delay(1000);
                leftMotors.move(-40);
                rightMotors.move(-40);
                pros::delay(100);
                leftMotors.move(0);
                rightMotors.move(0);
            }
            
            break;

        // Red, negative side
        case 2:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(-50, 11.2, 242);
            chassis.moveToPose(-69, 0, 240, 2000, {.maxSpeed = 47});
            pros::delay(900);
            arm.move(-120);
            pros::delay(700);
            arm.move(120);
            pros::delay(500);
            arm.move(0);
            chassis.moveToPose(-20, 25, 244, 2500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntil(36);
            latch.set_value(true);
            //pros::delay(50);
            chassis.turnToHeading(0, def);
            spinConveyor = 1;

            // grabs
            chassis.moveToPose(-24, 47, 356, def, {.earlyExitRange = 1});

            if (touchLadder) {
                chassis.moveToPose(-24, 48, 356, def);
                pros::delay(3500);
                chassis.moveToPose(-14, 0, 334, 2500, {.forwards = false});
                chassis.waitUntilDone();
                spinConveyor = 0;
            } else {
                chassis.turnToHeading(310, def);
                chassis.moveToPose(-64.5, 67, 325, def);
                chassis.turnToHeading(320, def);
                chassis.waitUntilDone();
                
                spinConveyor = 0;
                leftMotors.move(40);
                rightMotors.move(40);
                pros::delay(500);
                //leftMotors = 5;
                //rightMotors = 5;
                spinConveyor = 1;
                pros::delay(500);

                leftMotors.move(-40);
                rightMotors.move(-40);
                pros::delay(200);
                leftMotors.move(40);
                rightMotors.move(40);
                pros::delay(400);
                leftMotors.move(5);
                rightMotors.move(5);

                pros::delay(1000);
                leftMotors.move(-40);
                rightMotors.move(-40);
                pros::delay(100);
                leftMotors.move(0);
                rightMotors.move(0);
            }
            
            break;
        
        // Red, positive side
        case 3:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(-50, -11.2, 298);
            chassis.moveToPose(-69, -2, 295, 2000, {.maxSpeed = 47});
            pros::delay(1000);
            arm.move(-120);
            pros::delay(600);
            arm.move(0);
            pros::delay(500);
            chassis.moveToPose(-20, -24, 296, 2500, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntil(36);
            latch.set_value(true);
            arm.move(120);
            pros::delay(1000);
            arm.move(0);
            chassis.turnToHeading(180, def);
            spinConveyor = 1;

            // grabs
            chassis.moveToPose(-24, -45, 184, def, {.earlyExitRange = 1});

            if (touchLadder) {
                pros::delay(1000);
                chassis.moveToPose(-24, -47, 184, def);
                pros::delay(2000);
                chassis.moveToPose(-16, 2, 206, 2500, {.forwards = false});
                chassis.waitUntilDone();
                spinConveyor = 0;
            } else {
                pros::delay(1000);
                chassis.moveToPose(-24, -47, 184, def);
                pros::delay(1500);
                chassis.turnToHeading(70, def);
                chassis.moveToPose(-50, -54, 80, def, {.forwards = false});
                latch.set_value(false);
                spinConveyor = 0;
                pros::delay(100);

                chassis.moveToPose(-8, -42, 90, def);
                chassis.turnToHeading(270, def);
            }
            
            break;

        // Blue, positive side
        case 4:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(50, -11.2, 62);
            chassis.moveToPose(69, -2, 65, 2000, {.maxSpeed = 47});
            pros::delay(1000);
            arm.move(-120);
            pros::delay(600);
            arm.move(0);
            pros::delay(500);
            chassis.moveToPose(20, -24, 64, 2500, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntil(36);
            latch.set_value(true);
            arm.move(120);
            pros::delay(1000);
            arm.move(0);
            chassis.turnToHeading(180, def);
            spinConveyor = 1;

            // grabs
            chassis.moveToPose(24, -45, 176, def, {.earlyExitRange = 1});
            pros::delay(3500);

            if (touchLadder) {
                chassis.moveToPose(24, -47, 176, def);
                chassis.moveToPose(16, 2, 154, 2500, {.forwards = false});
                chassis.waitUntilDone();
                spinConveyor = 0;
            } else {
                chassis.turnToHeading(290, def);
                chassis.moveToPose(48, -54, 280, def, {.forwards = false});
                latch.set_value(false);
                spinConveyor = 0;
                pros::delay(100);

                chassis.moveToPose(8, -44, 270, def);
                chassis.turnToHeading(90, def);
            }
            
            break;

        // Auton Skills
        case 5:
            def = 2000;
            chassis.setPose(-60.7, 0, 270);
            arm.move(-120);
            pros::delay(600);
            arm.move(120);
            pros::delay(400);
            arm.move(0);
            chassis.moveToPose(-50, -23, 0, 2500, {.forwards = false});
            chassis.waitUntilDone();
            latch.set_value(true);
            pros::delay(500);
            spinConveyor = 1;

            // Red Pos Corner
            chassis.moveToPose(-23.8, -23.65, 90, def);
            chassis.turnToHeading(130, 500);
            chassis.moveToPose(-10, -40, 140, def, {.earlyExitRange = 5});
            chassis.moveToPose(0, -64, 180, 2000);
            pros::delay(200);
            chassis.turnToHeading(273, def);

            chassis.moveToPose(-24, -49, 270, def);
            chassis.turnToHeading(270, 500);
            chassis.moveToPose(-41, -51, 270, 2000, {.maxSpeed = 70, .earlyExitRange = 5});
            chassis.moveToPose(-60, -51, 270, 2000, {.maxSpeed = 45});
            
            chassis.moveToPose(-61, -35, 0, def);
            chassis.moveToPose(-68, -73, 0, 2000, {.forwards = false});
            chassis.turnToHeading(45, def);
            latch.set_value(false);
            pros::delay(200);
            spinConveyor = 0;
            //chassis.setPose(-58, -58, 35);

            // Red Neg Corner
            chassis.moveToPose(-53, -8, 0, 3000);
            chassis.turnToHeading(180, def);
            chassis.moveToPose(-46, 20, 180, 3000, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntil(27);
            latch.set_value(true);
            pros::delay(100);
            
            chassis.turnToHeading(0, def);
            spinConveyor = 1;
            chassis.moveToPose(-53, 60, 0, 3000, {.maxSpeed = 50});
            chassis.turnToHeading(90, def);
            chassis.moveToPose(-68, 60, 90, 3000, {.forwards = false});
            chassis.turnToHeading(135, def);
            chassis.waitUntilDone();
            spinConveyor = 0;
            latch.set_value(false);
            
            /*chassis.turnToHeading(90, def);

            spinConveyor = 1;
            chassis.moveToPose(-23, 20, 90, def);
            pros::delay(800);
            


            chassis.moveToPose(-47, 47, 135, 3500, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 5});
            chassis.moveToPose(-71, 67, 135, 3500, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntilDone();
            latch.set_value(false);*/
            
            /* Old layout
            
            chassis.setPose(-62, -11.5, 90);
            chassis.follow(path1_txt, 5, 6000);
            chassis.waitUntilDone();

            chassis.moveToPose(-53, -51, 212, def, {.forwards = false});
            chassis.turnToHeading(90, def);
            chassis.follow(path2_txt, 5, 12000);
            chassis.waitUntilDone();

            chassis.moveToPose(54, -51, 133, def, {.forwards = false});
            chassis.turnToHeading(0, def);
            chassis.follow(path3_txt, 5, 11000);
            chassis.waitUntilDone();

            chassis.moveToPose(56, 55, 42, 1000, {.forwards = false});
            //chassis.moveToPose(66, 66, 42, 1000, {.minSpeed = 127});
            chassis.turnToHeading(270, def);
            chassis.follow(path4_txt, 5, 10000);
            chassis.waitUntilDone();

            chassis.turnToHeading(270, def);*/
            break;

        // Testing
        case 6:
            //latch.set_value(true);
            //conveyor = 120;
            //pros::delay(1000);

            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 20000);
            //chassis.moveToPose(0, 5, 0, 5000);
            
            //chassis.moveToPose(0, 12, 0, def, {.forwards = false, .minSpeed = 120});
            break;
        case 7:
            //latch.set_value(true);

            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(180, 5000);
            chassis.turnToHeading(0, 5000);
            chassis.turnToHeading(-90, 5000);
            chassis.turnToHeading(0, 5000);
            break;
        case 8:
            chassis.setPose(0, 0, 0);

            chassis.moveToPose(0, 25, 0, def);
            chassis.turnToHeading(90, def);

            chassis.moveToPose(25, 25, 90, def);
            chassis.turnToHeading(180, def);

            chassis.moveToPose(25, 0, 180, def);
            chassis.turnToHeading(270, def);

            chassis.moveToPose(0, 0, 270, def);
            chassis.turnToHeading(360, def);
            break;
        case 9:
            pros::delay(1000);
            latch.set_value(true);
            spinConveyor = 1;
            break;
    }
}

/**
 * Runs in driver control
 */

bool toggle = false;
bool toggle2 = false;
bool toggle3 = true;
bool toggle4 = false;
bool toggle5 = false;
bool toggle6 = false;

void opcontrol() {
    // set up
    detectBlockage = false;
    latch.set_value(true);
    elevation.set_value(false);

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX, 2.7);

        // Pneumatics (Press to activate)
        if (controller.get_digital(DIGITAL_L2)) {
            latch.set_value(false);
        } else {
            latch.set_value(true);
        }

        // Flag
        if (controller.get_digital_new_press(DIGITAL_L1)) {
            elevation.set_value(!toggle5);    // When false go to true and in reverse
            toggle5 = !toggle5;    // Flip the toggle to match piston state
        }

        // Primer

        /*if (controller.get_digital_new_press(DIGITAL_LEFT)) {
            primed = !primed;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            if (primed) {
                //int desired_position = -310;
                //arm.move_absolute(desired_position, 40); // Moves 100 units forward
                //while (!((arm.get_position() < desired_position+5) && (arm.get_position() > desired_position-5))) {
                //    pros::delay(2);
                //}
                //arm = -50;
                //pros::delay(500);
            } else {
                arm = 50;
                pros::delay(500);
                arm = 0;
            }
            
        }
        if (primed) {
            arm = -5;
        }*/

        // Lady Brown
        if (controller.get_digital(DIGITAL_LEFT)) {
            primed = true;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            arm.move(50);
        } else if (controller.get_digital(DIGITAL_DOWN)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            arm.move(-50);
        } else {
            if (primed) {
                arm.move(-5);
            } else {
                arm.move(0);
            }
        }
        if (controller.get_digital_new_press(DIGITAL_UP)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            spinConveyor = -1;
            arm.move(120);
            pros::delay(350);
            spinConveyor = 0;
            arm.move(0);
            //arm = 80;
            //pros::delay(300);
        }
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            arm.move(-120);
            pros::delay(200);
            arm.move(0);
        }

        // Elevation Mech
        if (controller.get_digital_new_press(DIGITAL_A)) {
            pto.set_value(!toggle6);
            toggle6 = !toggle6;
        }

        // Disable color sorting (inactive)
        if (controller.get_digital_new_press(DIGITAL_X)) {
            if (toggle3) {
                colorSorting = false;
                controller.rumble("..");
            } else {
                colorSorting = true;
                controller.rumble("-");
            }
            toggle3 = !toggle3;
        }

        // Conveyor motor (max is ±127)
        if (controller.get_digital(DIGITAL_R1)) {
            spinConveyor = -1;
        } else if (controller.get_digital(DIGITAL_R2)) {
            spinConveyor = 1;
        }

        // intake & conveyor toggle
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            toggle2 = !toggle2;
        }
        if (toggle2) {
            spinConveyor = 1;
        }

        if (!toggle2 && !controller.get_digital(DIGITAL_R1) && !controller.get_digital(DIGITAL_R2)) {
            spinConveyor = 0;
        }
        
        pros::delay(10);
    }
}
