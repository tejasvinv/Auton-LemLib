#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp" // pneumatics
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp" // tasks
#include "pros/llemu.hpp" // brain screen
#include "pros/imu.hpp" // inertial sensor
#include "pros/motors.hpp" // motor groups

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({8, -9, 10}, pros::MotorGearset::blue); // left motor group - ports 1, 2, 3 (not reversed)
pros::MotorGroup rightMotors({-4, 5, -6}, pros::MotorGearset::blue); // right motor group - ports 4, 5, 6 ( not reversed)

// motors
pros::Motor motor11 (11, pros::v5::MotorGears::green); // roller; not reversed
pros::Motor motor13 (-13, pros::v5::MotorGears::green); // roller; reversed
pros::Motor motor12 (12, pros::v5::MotorGears::green); // roller; not reversed
pros::Motor motor14 (-14, pros::v5::MotorGears::green); // roller; reversed

// sensors
pros::Imu imu(15); // Inertial Sensor on port 10
// pros::Rotation lb (12); // Rotation sensor on port 15
// pros::Optical colorSensor(5);

// Pneumatics
pros::adi::Pneumatics scraper ('A', false); // clamp on port A; starts off retracted
pros::adi::Pneumatics hood ('B', false); // doinker on port C; starts off retracted
bool pistonToggle = false; // toggle for pneumatics

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 12 inch track width
                              3.25,// using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            15, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
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
    // set position to x:0, y:0, heading:0
   chassis.setPose(0, 0, 0);
   // move 48" forwards
   
   /*
   chassis.moveToPose(-1.5, 34, 177, 3000, {.forwards = false, .maxSpeed = 85});
   chassis.waitUntilDone();
   pros::delay(500);
   scraper.extend();
   pros::delay(500);
   chassis.moveToPose(-1.5, 25, 177, 3000, {.maxSpeed = 127});  
   chassis.waitUntilDone();
   pros::delay(500);
   belt.move_velocity(600);
   pros::delay(2000);
   belt.move_velocity(0);
   pros::delay(500);
   //chassis.moveToPose(-1.5, 30, 177, 3000, {.forwards = false});
   */

   /*
   chassis.moveToPose(0, -27, 0, 3000, {.forwards = false, .maxSpeed = 60});
   chassis.waitUntilDone();
   pros::delay(300);
   scraper.extend();
   pros::delay(300);
   belt.move_velocity(600);
   pros::delay(1000);
   belt.move_velocity(0);
   pros::delay(300);
   chassis.turnToHeading(88, 2000);
   chassis.waitUntilDone();
   pros::delay(300);
   intake.move_velocity(200);
   belt.move_velocity(600);
   pros::delay(300);
   chassis.moveToPoint(18, -24, 2000);
   chassis.waitUntilDone();
   pros::delay(300);
   */

   /*
   intake.move_velocity(200);
   belt.move_velocity(600);
   pros::delay(1000);
   intake.move_velocity(0);
   belt.move_velocity(0);
   */

   // chassis.moveToPoint(0, 48, 10000, {.maxSpeed = 64}, false);
   // chassis.turnToHeading(90, 10000);
   // chassis.moveToPoint(0, 48, 10000);

   chassis.moveToPoint(-1.3, 0, 750, {.maxSpeed = 64}, false);
   // run intake
   chassis.moveToPoint(-1.3, 33, 2000, {.maxSpeed = 64}, false);
   pros::delay(1000);
   chassis.moveToPoint(-1.3, 15, 3000, {.maxSpeed = 64}, false);
   chassis.moveToPoint(-1.9, 27, 3000, {.maxSpeed = 64}, false);
   // score
   




      

   
   
}

/**
 * Runs in driver control
 */
 void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);

        /*
        // color sort
        int hue = colorSensor.get_hue();
        if (hue >= 200 && hue <= 250) { 
            belt.move_velocity(600);
            belt.move_velocity(0);
        } 
        */

        /*
        // clamp
        if (controller.get_digital(DIGITAL_B)){
            if(pistonToggle == false){
                scraper.extend();
                pros::delay(500);
                pistonToggle = true;

            }
            else{
                scraper.retract();
                pros::delay(500);
                pistonToggle = false;
            }
        }
        */

        // Roller Functions

        // Check if Button X is pressed
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            // longgoal
            // hood position = score
            motor11.move_velocity(200);
            motor12.move_velocity(200);
            motor13.move_velocity(200);
            motor14.move_velocity(200);
        } 
        // If X wasn't pressed, check if Y is pressed
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            // hoarder
            // hood position = hoard
            motor11.move_velocity(200);
            motor12.move_velocity(200);
            motor13.move_velocity(200);
            motor14.move_velocity(200);
        }
        // If neither X nor Y were pressed, check if A is pressed
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            // center top
            motor11.move_velocity(-200);
            motor12.move_velocity(200);
            motor13.move_velocity(200);
            motor14.move_velocity(200);
        }
        // If none of the above were pressed, check if B is pressed
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // center bottom
            motor11.move_velocity(0); // Explicitly stop motor11
            motor12.move_velocity(200);
            motor13.move_velocity(-200);
            motor14.move_velocity(-200);
        }
        // If none of the buttons are being pressed
        else {
            // Stop all motors
            motor11.move_velocity(0);
            motor12.move_velocity(0);
            motor13.move_velocity(0);
            motor14.move_velocity(0);
        }

        // Hood extensions
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // R1 extends Hood
            hood.extend();
        } else {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                // R2 retracts Hood
                hood.retract();
            }
        }

        // Scraper extensions
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            // L1 extends Scraper
            scraper.extend();
        } else {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
                // L2 retracts Scraper
                scraper.retract();
            }
        }


    }
}