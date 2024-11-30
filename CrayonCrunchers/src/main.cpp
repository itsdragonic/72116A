#include "main.h"
#include "lemlib/api.hpp"
#include "gif-pros/gifclass.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::Motor lF(-14, pros::E_MOTOR_GEARSET_06); //
pros::Motor lM(-12, pros::E_MOTOR_GEARSET_06); //
pros::Motor lB(13, pros::E_MOTOR_GEARSET_06); //
pros::Motor rF(5, pros::E_MOTOR_GEARSET_06); //
pros::Motor rM(19, pros::E_MOTOR_GEARSET_06); //
pros::Motor rB(-18, pros::E_MOTOR_GEARSET_06); //

// Motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Other Motors
pros::Motor arm(2, pros::E_MOTOR_GEARSET_36); // 5
pros::Motor conveyor(7);
pros::Motor intake(10);

// Pneumatics
pros::ADIDigitalOut latch('H');
pros::ADIDigitalOut arm2('A');
pros::ADIDigitalOut flag('E');

// Sensors
pros::Imu imu(21);
pros::Optical optical(4);

// Important Variables
bool alliance = true; // true means blue, false means red
int autonSide = 2; // 1 is positive, -1 is negative, 0 is skills
int autonRoute = 9;
bool colorSorting = true;
bool activated = false;

// Tracking Wheels

// horizontal tracking wheel (pros::Rotation). 2.00" diameter, 4.5" offset, back of the robot (negative)
pros::Rotation vertical1Enc(11, true);
pros::Rotation vertical2Enc(20);
pros::Rotation horizontalEnc(15);

lemlib::TrackingWheel vertical1(&vertical1Enc, lemlib::Omniwheel::NEW_275, -7); // 6.875
lemlib::TrackingWheel vertical2(&vertical2Enc, lemlib::Omniwheel::NEW_275, 7);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -4); // 4.5

// drivetrain settings
lemlib::Drivetrain drivetrain (
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    14, //track width
    lemlib::Omniwheel::NEW_275, // wheel diameter
    480, // drivetrain rpm
    2 // chase power is 2. If there were traction wheels, it would be 8
);

// lateral motion controller (1 tile = 23.8 Δx)
lemlib::ControllerSettings linearController (
    90, // proportional gain (kP) 500
    0, // integral gain (kI) 0 
    260, // derivative gain (kD) 100
    3, // anti windup 3
    1, // small error range, in inches 1
    50, // small error range timeout, in milliseconds 100
    3, // large error range, in inches 3
    100, // large error range timeout, in milliseconds 500
    20 // maximum acceleration (slew) 5
);

// angular motion controller
lemlib::ControllerSettings angularController (
    2, // proportional gain (kP) 1
    0, // integral gain (kI) 0 
    14.25, // derivative gain (kD) 19
    0, // anti windup 3
    0, // small error range, in degrees 1
    0, // small error range timeout, in milliseconds 100
    0, // large error range, in degrees 3
    0, // large error range timeout, in milliseconds 500
    0 // maximum acceleration (slew) 5
);

// Sensors for odometry
lemlib::OdomSensors sensors( 
    &vertical1, // vertical tracking wheel 1
    &vertical2, // vertical tracking wheel 2
    &horizontal, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

// Text
lv_obj_t * myLabel;
lv_obj_t * txtInfo;
lv_style_t labelStyle;

// Images
lv_obj_t * imgLogo;
lv_obj_t * imgLogo2;
lv_obj_t * imgLogo3;

// button 1
lv_obj_t * button1;
lv_obj_t * button1Label;

lv_style_t button1StyleREL; // released style
lv_style_t button1StylePR; // pressed style

// button 2
lv_obj_t * button2;
lv_obj_t * button2Label;

lv_style_t button2StyleREL; // released style
lv_style_t button2StylePR; // pressed style

// button 3
lv_obj_t * button3;
lv_obj_t * button3Label;

lv_style_t button3StyleREL; // released style
lv_style_t button3StylePR; // pressed style

// button 4
lv_obj_t * button4;
lv_obj_t * button4Label;

lv_style_t button4StyleREL; // released style
lv_style_t button4StylePR; // pressed style

LV_IMG_DECLARE(crayonlogo);
LV_IMG_DECLARE(crayonlogoBW);

char buffer[100];

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); // id usefull when there are multiple buttons

    if (id == 0) {
        if (alliance) {
            button1StyleREL.body.main_color = LV_COLOR_MAKE(255, 0, 0);
            button1StyleREL.body.grad_color = LV_COLOR_MAKE(100, 50, 50);
            lv_label_set_text(button1Label, "Red Team"); // sets label text
            alliance = false;
        } else {
            button1StyleREL.body.main_color = LV_COLOR_MAKE(0, 0, 255);
            button1StyleREL.body.grad_color = LV_COLOR_MAKE(50, 50, 100);
            lv_label_set_text(button1Label, "Blue Team"); // sets label text
            alliance = true;
        }
    }
    else if (id == 1) {
		sprintf(buffer, "Auton Route [+] Selected");
        autonSide = 1;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    else if (id == 2) {
		sprintf(buffer, "Auton Route [-] Selected");
        autonSide = -1;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    else if (id == 3) {
		sprintf(buffer, "Auton Route [Skills] Selected");
        autonSide = 0;
        labelStyle.text.color = LV_COLOR_LIME;
		lv_label_set_text(myLabel, buffer);
    }
    
    return LV_RES_OK;
}

int conveyorSpeed = 100;
int speedUp = -700;
int speedUp2 = 0;
int spinConveyor = 0;

int Clock = 100;
int counter = 0;

void conveyorChecking() {
    while (true) {
        conveyorSpeed = 100;

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
            conveyorSpeed = 127;
        }

        optical.set_led_pwm(100);
        if (colorSorting && !activated && optical.get_saturation() > 0.2) {
            // color sorting
            if (alliance && ((optical.get_hue() > 345) || (optical.get_hue() < 15))) {
                speedUp = 29;
                activated = true;
            }
            if (!alliance && ((optical.get_hue() > 210) && (optical.get_hue() < 230))) {
                speedUp = 29;
                activated = true;
            }
            
            // speed up
            if (!alliance && ((optical.get_hue() > 345) || (optical.get_hue() < 15))) {
                speedUp2 = 30;
            }
            if (alliance && ((optical.get_hue() > 210) && (optical.get_hue() < 230))) {
                speedUp2 = 30;
            }
        }

        // Spin conveyor
        switch (spinConveyor) {
            case 1:
                conveyor = conveyorSpeed;
                break;
            case -1:
                conveyor = -conveyorSpeed;
                break;
            case 0:
                conveyor = 0;
                break;
        }

        pros::delay(10);
    }
}

// Initialize screen and everything else
void initialize() {
    lemlib::infoSink()->setLowestLevel(lemlib::Level::DEBUG);
    chassis.calibrate(); // calibrate sensors
    pros::delay(3000);

    pros::Task conveyor_task(conveyorChecking);

    // Create a Tab view object
    lv_obj_t *tabview;
    lv_style_t tabstyle;
    tabview = lv_tabview_create(lv_scr_act(), NULL);

    // Add 3 tabs
    lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Main Menu");
    lv_obj_t *tab2 = lv_tabview_add_tab(tabview, "Settings");
    lv_obj_t *tab3 = lv_tabview_add_tab(tabview, "Debug Screen");

    // Images
    imgLogo = lv_img_create(tab1, NULL);
    lv_img_set_src(imgLogo, &crayonlogo);
    lv_obj_align(imgLogo, NULL, LV_ALIGN_IN_LEFT_MID, -20, -20);

    imgLogo2 = lv_img_create(tab2, NULL);
    lv_img_set_src(imgLogo2, &crayonlogoBW);
    lv_obj_align(imgLogo2, NULL, LV_ALIGN_CENTER, 0, 0);

    imgLogo3 = lv_img_create(tab3, NULL);
    lv_img_set_src(imgLogo3, &crayonlogoBW);
    lv_obj_align(imgLogo3, NULL, LV_ALIGN_CENTER, 0, 0);

    // BUTTON 1
    lv_style_copy(&button1StyleREL, &lv_style_plain);
    button1StyleREL.body.main_color = LV_COLOR_MAKE(0, 0, 255);
    button1StyleREL.body.grad_color = LV_COLOR_MAKE(50, 50, 100);
    button1StyleREL.body.radius = 5;
    button1StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&button1StylePR, &lv_style_plain);
    button1StylePR.body.main_color = LV_COLOR_MAKE(255, 255, 255);
    button1StylePR.body.grad_color = LV_COLOR_MAKE(200, 200, 200);
    button1StylePR.body.radius = 5;
    button1StylePR.body.shadow.width = 15;
    button1StylePR.text.color = LV_COLOR_MAKE(0, 0, 0);

    button1 = lv_btn_create(tab2, NULL); // create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button1, 0); // set button is to 0
    lv_btn_set_action(button1, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
    lv_btn_set_style(button1, LV_BTN_STYLE_REL, &button1StyleREL); // set the released style
    lv_btn_set_style(button1, LV_BTN_STYLE_PR, &button1StylePR); // set the pressed style
    lv_obj_set_size(button1, 150, 30); // set the button size
    lv_obj_align(button1, NULL, LV_ALIGN_IN_TOP_MID, 0, 10); // set the position to top mid

    button1Label = lv_label_create(button1, NULL); // create label and puts it inside of the button
    lv_label_set_text(button1Label, "Blue Team"); // sets label text

    // BUTTON 2
    lv_style_copy(&button2StyleREL, &lv_style_plain);
    button2StyleREL.body.main_color = LV_COLOR_MAKE(72, 101, 79); // normal color
    button2StyleREL.body.grad_color = LV_COLOR_MAKE(47, 76, 54); // extra gradient
    button2StyleREL.body.radius = 5;
    button2StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&button2StylePR, &lv_style_plain);
    button2StylePR.body.main_color = LV_COLOR_MAKE(78, 157, 96); // color when pressed
    button2StylePR.body.grad_color = LV_COLOR_MAKE(74, 152, 92); // extra gradient
    button2StylePR.body.radius = 5;
    button2StylePR.body.shadow.width = 15;
    button2StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    button2 = lv_btn_create(tab2, NULL); // create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button2, 1); // set button is to 0
    lv_btn_set_action(button2, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
    lv_btn_set_style(button2, LV_BTN_STYLE_REL, &button2StyleREL); // set the released style
    lv_btn_set_style(button2, LV_BTN_STYLE_PR, &button2StylePR); // set the pressed style
    lv_obj_set_size(button2, 150, 30); // set the button size
    lv_obj_align(button2, NULL, LV_ALIGN_CENTER, 0, -65); // set the position to top mid

    button2Label = lv_label_create(button2, NULL); // create label and puts it inside of the button
    lv_label_set_text(button2Label, "Auton ( + Pos)"); // sets label text

    // BUTTON 3
    lv_style_copy(&button3StyleREL, &lv_style_plain);
    button3StyleREL.body.main_color = LV_COLOR_MAKE(46, 44, 53);
    button3StyleREL.body.grad_color = LV_COLOR_MAKE(24, 23, 31);
    button3StyleREL.body.radius = 5;
    button3StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&button3StylePR, &lv_style_plain);
    button3StylePR.body.main_color = LV_COLOR_MAKE(62, 58, 76);
    button3StylePR.body.grad_color = LV_COLOR_MAKE(43, 40, 57);
    button3StylePR.body.radius = 5;
    button3StylePR.body.shadow.width = 15;
    button3StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    button3 = lv_btn_create(tab2, NULL); // create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button3, 2); // set button is to 0
    lv_btn_set_action(button3, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
    lv_btn_set_style(button3, LV_BTN_STYLE_REL, &button3StyleREL); // set the released style
    lv_btn_set_style(button3, LV_BTN_STYLE_PR, &button3StylePR); // set the pressed style
    lv_obj_set_size(button3, 150, 30); // set the button size
    lv_obj_align(button3, NULL, LV_ALIGN_CENTER, 0, -25); // set the position to top mid

    button3Label = lv_label_create(button3, NULL); // create label and puts it inside of the button
    lv_label_set_text(button3Label, "Auton ( - Neg)"); // sets label text

    // BUTTON 4
    lv_style_copy(&button4StyleREL, &lv_style_plain);
    button4StyleREL.body.main_color = LV_COLOR_MAKE(72, 101, 79); // normal color
    button4StyleREL.body.grad_color = LV_COLOR_MAKE(47, 76, 54); // extra gradient
    button4StyleREL.body.radius = 5;
    button4StyleREL.text.color = LV_COLOR_MAKE(255, 255, 255);

    lv_style_copy(&button4StylePR, &lv_style_plain);
    button4StylePR.body.main_color = LV_COLOR_MAKE(78, 157, 96); // color when pressed
    button4StylePR.body.grad_color = LV_COLOR_MAKE(74, 152, 92); // extra gradient
    button4StylePR.body.radius = 5;
    button4StylePR.body.shadow.width = 15;
    button4StylePR.text.color = LV_COLOR_MAKE(255, 255, 255);

    button4 = lv_btn_create(tab2, NULL); // create button, lv_scr_act() is deafult screen object
    lv_obj_set_free_num(button4, 3); // set button is to 0
    lv_btn_set_action(button4, LV_BTN_ACTION_CLICK, btn_click_action); // set function to be called on button click
    lv_btn_set_style(button4, LV_BTN_STYLE_REL, &button4StyleREL); // set the released style
    lv_btn_set_style(button4, LV_BTN_STYLE_PR, &button4StylePR); // set the pressed style
    lv_obj_set_size(button4, 150, 30); // set the button size
    lv_obj_align(button4, NULL, LV_ALIGN_CENTER, 0, 15); // set the position to top mid

    button4Label = lv_label_create(button4, NULL); // create label and puts it inside of the button
    lv_label_set_text(button4Label, "Auton Skills"); // sets label text

    // Labels
    myLabel = lv_label_create(tab3, NULL); // create label and puts it on the screen
    lv_style_copy(&labelStyle, &lv_style_plain_color);
    labelStyle.text.color = LV_COLOR_RED;
    lv_label_set_style(myLabel, &labelStyle);
    lv_label_set_text(myLabel, "Button has not been clicked yet"); // sets label text
    lv_obj_align(myLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10); // set the position to center


    txtInfo = lv_label_create(tab3, NULL); // create label and puts it on the screen
    lv_label_set_text(txtInfo, "null"); // sets label text
    lv_obj_align(txtInfo, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 35); // set the position to center

    // LemLib: thread to for brain screen and position logging

    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        
        while (true) {

            // Temperature (130, 140, 150, 160 °F)
            int motorTemps[6];
            motorTemps[0] = lF.get_temperature();
            motorTemps[1] = lM.get_temperature();
            motorTemps[2] = lB.get_temperature();
            motorTemps[3] = rF.get_temperature();
            motorTemps[4] = rM.get_temperature();
            motorTemps[5] = rB.get_temperature();

            int *maxElement = std::max_element(std::begin(motorTemps), std::end(motorTemps));

            // print info
            char txtBuffer[200];
            sprintf(txtBuffer, "X: %.2f \nY: %.2f \nTheta: %.2f \nHue: %.2f \nSaturation: %.2f \nTemperature: %.1f°F "SYMBOL_WARNING,
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, optical.get_hue(), optical.get_saturation(), static_cast<double>(*maxElement)*9/5+32);

		    lv_label_set_text(txtInfo, txtBuffer);

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(50);
        }
    });
};

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit (this needs to be put outside a function)
ASSET(example1_txt); // '.' replaced with "_" to make c++ happy

// acutally in use
ASSET(path1_txt);
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);

/**
 * Autonomous
 *
 */
void autonomous() {

    //chassis.moveToPose(0, 20, 0, 5000);
    //chassis.turnToHeading(90, 1000, {.minSpeed = 100});
    int def = 2000;
    int pickupTime = 2500;

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    latch.set_value(false);

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
            chassis.setPose(50, 35.8, 90);
            chassis.moveToPose(22.4, 22.3, 60, 3000, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            pros::delay(500);
            spinConveyor = -1;
            pros::delay(100);
            spinConveyor = 0;
            latch.set_value(true);
            intake = 100;
            pros::delay(500);
            spinConveyor = 1;
            pros::delay(1000);

            // grab rings
            chassis.moveToPose(24.4, 41.9, 350, def);
            pros::delay(3000);
            chassis.moveToPose(9.4, 49.6, 270, def);
            pros::delay(2500);
            chassis.turnToHeading(230, def);
            
            break;

        // Red, negative side
        case 2:
            chassis.setPose(50, -35.8, 90);
            chassis.moveToPose(22.4, -22.3, 120, 3000, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            pros::delay(100);
            spinConveyor = -1;
            pros::delay(100);
            spinConveyor = 0;
            latch.set_value(true);
            intake = 100;
            pros::delay(500);
            spinConveyor = 1;
            pros::delay(1000);

            // grab rings
            chassis.moveToPose(24.4, -41.9, 190, def);
            pros::delay(3000);
            chassis.moveToPose(9.4, -49.6, 270, def);
            pros::delay(2500);
            chassis.turnToHeading(300, def);
            
            break;
        
        // Red, positive side
        case 3:
            chassis.setPose(50, 35.8, 90);
            chassis.moveToPose(22.4, 22.3, 60, 3000, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            pros::delay(100);
            spinConveyor = -1;
            pros::delay(100);
            spinConveyor = 0;
            latch.set_value(true);
            intake = 100;
            pros::delay(500);
            spinConveyor = 1;
            pros::delay(1000);

            // grab ring & touch ladder
            chassis.moveToPose(24.4, 41.9, 350, def);
            pros::delay(3000);
            spinConveyor = 0;
            intake = 0;
            chassis.turnToHeading(170, def);
            chassis.moveToPose(17.7, 1.8, 190, def);
            
            break;

        // Blue, positive side
        case 4:
            chassis.setPose(50, -35.8, 90);
            chassis.moveToPose(22.4, -22.3, 120, 3000, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            pros::delay(100);
            spinConveyor = -1;
            pros::delay(100);
            spinConveyor = 0;
            latch.set_value(true);
            intake = 100;
            pros::delay(500);
            spinConveyor = 1;
            pros::delay(1000);

            // grab ring & touch ladder
            chassis.moveToPose(24.4, -41.9, 190, def);
            pros::delay(3000);
            intake = 0;
            spinConveyor = 0;
            chassis.turnToHeading(10, def);
            chassis.moveToPose(17.7, -1.8, 350, def);
            
            break;

        // Auton Skills
        case 5:
            chassis.setPose(-62, -11.5, 90);
            chassis.follow(path1_txt, 5, 7000);
            chassis.waitUntilDone();

            chassis.moveToPose(-53, -51, 212, def, {.forwards = false});
            chassis.turnToHeading(90, def);
            chassis.follow(path2_txt, 5, 13000);
            chassis.waitUntilDone();

            chassis.moveToPose(56, -54, 133, def, {.forwards = false});
            chassis.turnToHeading(0, def);
            chassis.follow(path3_txt, 5, 14000);
            chassis.waitUntilDone();

            chassis.moveToPose(52, 50, 42, def, {.forwards = false});
            chassis.turnToHeading(270, def);
            chassis.follow(path4_txt, 5, 30000);
            chassis.waitUntilDone();
            break;

        // Testing
        case 6:
            //latch.set_value(true);
            //intake = -100;
            //conveyor = 120;
            pros::delay(1000);

            chassis.setPose(0, 0, 0);
            chassis.moveToPose(0, 24, 0, def);
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
            intake = 100;
            break;
    }
}

/**
 * Driver control
 */

bool toggle = false;
bool toggle2 = false;
bool toggle3 = true;
bool toggle4 = false;
bool toggle5 = false;

void opcontrol() {
    // set up
    latch.set_value(true);
    arm2.set_value(false);
    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

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

        // Arm
        if (controller.get_digital_new_press(DIGITAL_L1)) {
            arm2.set_value(!toggle4);    // When false go to true and in reverse
            toggle4 = !toggle4;    // Flip the toggle to match piston state
        }

        // Flag
        if (controller.get_digital_new_press(DIGITAL_Y)) {
            flag.set_value(!toggle5);    // When false go to true and in reverse
            toggle5 = !toggle5;    // Flip the toggle to match piston state
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
            intake = -100;
        } else if (controller.get_digital(DIGITAL_R2)) {
            spinConveyor = 1;
            intake = 100;
        }

        // wall stake helper
        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            controller.rumble("-");
            chassis.setPose(0, 0, 0);
            chassis.moveToPose(0, 1, 0, 3000);
            chassis.waitUntilDone();
            arm2.set_value(true);
            pros::delay(2000);
            conveyor = 115;
            pros::delay(2000);
            arm2.set_value(false);
        }

        // intake & conveyor toggle
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            toggle2 = !toggle2;
        }
        if (toggle2) {
            intake = 100;
            spinConveyor = 1;
        }

        if (!toggle2 && !controller.get_digital(DIGITAL_R1) && !controller.get_digital(DIGITAL_R2)) {
            spinConveyor = 0;
            intake = 0;
        }
        
        pros::delay(10);
    }
}