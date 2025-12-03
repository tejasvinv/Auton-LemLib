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
#include "lemlib/asset.hpp" // asset manager

// include assets
ASSET(r_txt);

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DRIVETRAIN - 6 motors (Blue/600 RPM)
// Left side: ports 11, 12, 13 (reversed)
// Right side: ports 18, 19, 20 (normal)
pros::MotorGroup leftMotors({-11, -12, -13}, pros::v5::MotorGears::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::v5::MotorGears::blue);

// INTAKE MOTORS - 3 motors on same axle
pros::Motor front_intake(10, pros::v5::MotorGears::blue);        // Blue motor, port 10, normal
pros::Motor mid_intake(1, pros::v5::MotorGears::green);          // Green motor, port 1
pros::Motor top_intake(9, pros::v5::MotorGears::green);          // Green motor, port 9

// Pneumatics
pros::adi::Pneumatics scraper('A', false);
pros::adi::Pneumatics wings('B', false);

// Tracking wheels
pros::Rotation horizontal_encoder(17); 
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -5.75);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, 
                              &rightMotors, 
                              12,      // 12 inch track width
                              3.25,    // 3.25" omnis
                              600,     // drivetrain rpm is 600 (blue motors)
                              8        // horizontal drift
);

// Lateral motion controller
lemlib::ControllerSettings linearController(8, 0, 15, 3, 1, 100, 3, 500, 20);

// Angular controller
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);

// Sensors for odometry
lemlib::OdomSensors sensors(nullptr,                        // no vertical tracking wheel 1
                            nullptr,                        // no vertical tracking wheel 2
                            &horizontal_tracking_wheel,     // horizontal tracking wheel
                            nullptr,                        // no horizontal tracking wheel 2
                            nullptr                         // no inertial sensor
);

// Input curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// Create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "Initializing...");
    
    chassis.calibrate();
    pros::lcd::print(1, "Calibration done");
    pros::delay(500);

    // Brain screen task
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::clear();
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

/**  
 * Autonomous routine
 */
void autonomous() {
    chassis.setPose(-53.759, -8.49, 20);
    chassis.follow(r_txt, 15, 4000, false);

    // Run intake during path
    front_intake.move_velocity(600);
    mid_intake.move_velocity(600);
    top_intake.move_velocity(600);

    chassis.waitUntilDone();

    // Stop intake
    front_intake.move_velocity(0);
    mid_intake.move_velocity(0);
    top_intake.move_velocity(0);

    // Move to score position
    chassis.moveToPoint(-30, -40, 1500, {.forwards = false, .maxSpeed = 64});

    scraper.extend();
    pros::delay(500);
    scraper.retract();
    pros::delay(300);
    chassis.moveToPoint(-30, -40, 1500, {.forwards = false, .maxSpeed = 64});

}

/**
 * Driver control
 */
void opcontrol() {
    // Set brake modes
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    front_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    mid_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    top_intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    
    while (true) {
        // Get joystick positions
        // Left stick Y-axis: forward/backward
        // Right stick X-axis: left/right turning
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Drive with arcade control
        chassis.arcade(leftY, rightX);

        // INTAKE CONTROLS
        // R1: Spin all intakes forward
        // R2: Spin all intakes backward
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            front_intake.move_velocity(600);   // Blue motor max speed
            mid_intake.move_velocity(200);     // Green motor max speed
            top_intake.move_velocity(200);     // Green motor max speed
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            front_intake.move_velocity(-600);  // Reverse
            mid_intake.move_velocity(-200);    // Reverse
            top_intake.move_velocity(-200);    // Reverse
        }
        else {
            // Stop all intakes when no button pressed
            front_intake.move_velocity(0);
            mid_intake.move_velocity(0);
            top_intake.move_velocity(0);
        }

        // Pneumatics controls (keeping your original setup)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            scraper.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            scraper.retract();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            wings.extend();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            wings.retract();
        }
        
        // Delay to save resources
        pros::delay(20);
    }
}