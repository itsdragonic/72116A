#include "main.h"
#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "gif-pros/gifclass.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::Motor lF(12, pros::E_MOTOR_GEARSET_06); //
pros::Motor lM(-13, pros::E_MOTOR_GEARSET_06); //
pros::Motor lB(-11, pros::E_MOTOR_GEARSET_06); //
pros::Motor rF(-14, pros::E_MOTOR_GEARSET_06); //
pros::Motor rM(16, pros::E_MOTOR_GEARSET_06); //
pros::Motor rB(17, pros::E_MOTOR_GEARSET_06); //

// Motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group

// Other Motors
pros::Motor arm(-8, pros::E_MOTOR_GEARSET_36); // 5
pros::Motor conveyor(3);
pros::Motor intake(9);

// Pneumatics
pros::ADIDigitalOut latch('B');
pros::ADIDigitalOut colorSorter('A'); // LB
pros::ADIDigitalOut flag('C'); // PTO

// Sensors
pros::Imu imu(21);
pros::Optical optical(4);

// Important Variables
bool alliance = true; // true means blue, false means red
int autonSide = 2; // 1 is positive, -1 is negative, 0 is skills
int autonRoute = 6;

bool colorSorting = true;
bool activated = false;
bool detectBlockage = false;
bool touchLadder = false;
bool primed = false;

// Tracking Wheels

// horizontal tracking wheel (pros::Rotation). 2.00" diameter, 4.5" offset, back of the robot (negative)
pros::Rotation vertical1Enc(10, true);
pros::Rotation vertical2Enc(1);
pros::Rotation horizontalEnc(15);

lemlib::TrackingWheel vertical1(&vertical1Enc, 2.125, -4.6); // 7
lemlib::TrackingWheel vertical2(&vertical2Enc, 2.125, 4.6);
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.125, -5); // 4

// drivetrain settings
lemlib::Drivetrain drivetrain (
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    10.4, //track width
    lemlib::Omniwheel::NEW_275, // wheel diameter
    480, // drivetrain rpm
    8 // chase power is 2. If there are traction wheels, it should be 8
);

// lateral motion controller (1 tile = 23.8 Δx)
lemlib::ControllerSettings linearController (
    8, // proportional gain (kP) 500
    0, // integral gain (kI) 0 
    150, // derivative gain (kD) 100
    3, // anti windup 3
    1, // small error range, in inches 1
    100, // small error range timeout, in milliseconds 100
    3, // large error range, in inches 3
    500, // large error range timeout, in milliseconds 500
    5 // maximum acceleration (slew) 5
);

// angular motion controller
lemlib::ControllerSettings angularController (
    3, // proportional gain (kP) 2
    0, // integral gain (kI) 0 
    82, // derivative gain (kD) 14
    0, // anti windup 3
    0, // small error range, in degrees 1
    0, // small error range timeout, in milliseconds 100
    8, // large error range, in degrees 3
    0, // large error range timeout, in milliseconds 500
    5 // maximum acceleration (slew) 5
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
                conveyor = -80;
                pros::delay(500);
                conveyor = 0;
                Clock = 0;
            }
        }

        // Optical Sensing
        if (speedUp > -100) {
            speedUp --;
        }

        if (speedUp < 40 && speedUp > -40) {
            if (detectBlockage) {
                colorSorter.set_value(true);
            }
        }
        if (speedUp < 0 && speedUp > -40) {
            conveyorSpeed = -100;
        }
        if (speedUp < -80) {
            activated = false;
            if (detectBlockage) {
                colorSorter.set_value(false);
            }
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
                conveyor = conveyorSpeed;
                intake = conveyorSpeed;
                break;
            case -1:
                conveyor = -conveyorSpeed;
                intake = -conveyorSpeed;
                break;
            case 0:
                conveyor = 0;
                intake = 0;
                break;
        }

        pros::delay(10);
    }
}

static lv_res_t cb_release_action(lv_obj_t * cb)
{
    /*A check box is clicked*/
    //printf("%s state: %d\n", lv_cb_get_text(cb), lv_cb_is_checked(cb));
    //controller.print(0, 0, "%s - %d\n", lv_cb_get_text(cb), lv_cb_is_checked(cb));
    if (lv_cb_is_checked(cb)) {
        touchLadder = true;
    } else {
        touchLadder = false;
    }

    return LV_RES_OK;
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

    /*Create  border style*/
    static lv_style_t style_border;
    lv_style_copy(&style_border, &lv_style_pretty_color);
    style_border.glass = 1;
    style_border.body.empty = 1;

    /*Create a container*/
    lv_obj_t * cont;
    cont = lv_cont_create(tab2, NULL);
    lv_cont_set_layout(cont, LV_LAYOUT_COL_L);      /*Arrange the children in a column*/
    lv_cont_set_fit(cont, true, true);              /*Fit the size to the content*/
    lv_obj_set_style(cont, &style_border);

    /**************************
     * Create check boxes
     *************************/

    //Create check box
    lv_obj_t * cb;
    cb = lv_cb_create(cont, NULL);
    lv_cb_set_text(cb, "Touch Ladder");
    lv_cb_set_action(cb, cb_release_action);

    //Align the container to the middle
    lv_obj_align(cont, NULL, LV_ALIGN_IN_LEFT_MID, -5, -40);


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
                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, optical.get_hue(), arm.get_position(), static_cast<double>(*maxElement)*9/5+32);

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

// Pure Pursuit paths
ASSET(path1_txt); // '.' replaced with "_" to make c++ happy
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);

/**
 * Autonomous
 */
void autonomous() {

    //chassis.moveToPose(0, 20, 0, 5000);
    //chassis.turnToHeading(90, 1000, {.minSpeed = 100});
    int def = 1500;
    int pickupTime = 2500;
    detectBlockage = true;

    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    latch.set_value(false);
    flag.set_value(true);

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
            arm = -120;
            pros::delay(700);
            arm = 120;
            pros::delay(500);
            arm = 0;
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
                leftMotors = 40;
                rightMotors = 40;
                pros::delay(500);
                //leftMotors = 5;
                //rightMotors = 5;
                spinConveyor = 1;
                pros::delay(500);

                leftMotors = -40;
                rightMotors = -40;
                pros::delay(200);
                leftMotors = 40;
                rightMotors = 40;
                pros::delay(400);
                leftMotors = 5;
                rightMotors = 5;

                pros::delay(1000);
                leftMotors = -40;
                rightMotors = -40;
                pros::delay(100);
                leftMotors = 0;
                rightMotors = 0;
            }
            
            break;

        // Red, negative side
        case 2:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(-50, 11.2, 242);
            chassis.moveToPose(-69, 0, 240, 2000, {.maxSpeed = 47});
            pros::delay(900);
            arm = -120;
            pros::delay(700);
            arm = 120;
            pros::delay(500);
            arm = 0;
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
                leftMotors = 40;
                rightMotors = 40;
                pros::delay(500);
                //leftMotors = 5;
                //rightMotors = 5;
                spinConveyor = 1;
                pros::delay(500);

                leftMotors = -40;
                rightMotors = -40;
                pros::delay(200);
                leftMotors = 40;
                rightMotors = 40;
                pros::delay(400);
                leftMotors = 5;
                rightMotors = 5;

                pros::delay(1000);
                leftMotors = -40;
                rightMotors = -40;
                pros::delay(100);
                leftMotors = 0;
                rightMotors = 0;
            }
            
            break;
        
        // Red, positive side
        case 3:
            // alliance stake
            //pros::delay(3000);
            chassis.setPose(-50, -11.2, 298);
            chassis.moveToPose(-69, -2, 295, 2000, {.maxSpeed = 47});
            pros::delay(1000);
            arm = -120;
            pros::delay(600);
            arm = 0;
            pros::delay(500);
            chassis.moveToPose(-20, -24, 296, 2500, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntil(36);
            latch.set_value(true);
            arm = 120;
            pros::delay(1000);
            arm = 0;
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
            arm = -120;
            pros::delay(600);
            arm = 0;
            pros::delay(500);
            chassis.moveToPose(20, -24, 64, 2500, {.forwards = false, .maxSpeed = 70});
            chassis.waitUntil(36);
            latch.set_value(true);
            arm = 120;
            pros::delay(1000);
            arm = 0;
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
            arm = -120;
            pros::delay(600);
            arm = 120;
            pros::delay(400);
            arm = 0;
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
            //chassis.turnToHeading(90, 20000);
            chassis.moveToPose(0, 24, 0, 5000);
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
 * Driver control
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
    flag.set_value(true);

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
            flag.set_value(!toggle5);    // When false go to true and in reverse
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
            arm = -40;
        } else if (controller.get_digital(DIGITAL_DOWN)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            arm = 40;
        } else {
            if (primed) {
                arm = -5;
            } else {
                arm = 0;
            }
        }
        if (controller.get_digital_new_press(DIGITAL_UP)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            spinConveyor = -1;
            arm = -120;
            pros::delay(350);
            spinConveyor = 0;
            arm = 0;
            //arm = 80;
            //pros::delay(300);
        }
        if (controller.get_digital_new_press(DIGITAL_RIGHT)) {
            primed = false;
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            arm = 120;
            pros::delay(200);
            arm = 0;
        }

        // Elevation Mech
        if (controller.get_digital_new_press(DIGITAL_A)) {
            colorSorter.set_value(!toggle6);
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