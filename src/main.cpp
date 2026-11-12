#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups blue
pros::Motor left_blue_1(-3, pros::MotorGearset::blue);
pros::Motor left_blue_2(-10, pros::MotorGearset::blue);
pros::Motor right_blue_1(-12, pros::MotorGearset::blue);
pros::Motor right_blue_2(-20, pros::MotorGearset::blue);
// 5.5
pros::Motor left_green(-2, pros::MotorGearset::green);
pros::Motor right_green(-11, pros::MotorGearset::green);

pros::Motor ratchetClaw(13, pros::MotorGearset::green);
pros::Motor cascadeLeft(8, pros::MotorGearset::blue);
pros::Motor cascadeRight(19, pros::MotorGearset::blue);

pros::MotorGroup leftMotors({2, 3, 10});
pros::MotorGroup rightMotors({-11, -12, -20});
// Inertial Sensor on port 15
pros::Imu imu(15);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(6);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(5);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              600, // drivetrain rpm is 600
                              6 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
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
    pros::Task screenTask([&]() 
    {
        while (true) 
        {
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
void competition_initialize() 
{
  
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() 
{
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);  

}

// Voltage values in PROS range from -127 to 127
const int CLAW_CLOSE_POWER = 108;    // ~85% of 127
const int CLAW_RELEASE_POWER = 127;  // 100% of 127
const int CLAW_HOLD_POWER = 19;      // ~15% of 127

// Timing & Position Thresholds
const int RELEASE_DURATION_MS = 250; 
const double MAX_CLOSE_DEGREES = 360.0; 

enum ClawState { OPEN, CLOSING, LOCKED, RELEASING };
ClawState currentClawState = OPEN;
int releaseTimer = 0;

void updateRatchetClaw(bool closeButton, bool releaseButton) 
{
    // 1. Trigger Release (Overrides everything to unlatch)
    if (releaseButton && currentClawState != RELEASING) 
    {
        currentClawState = RELEASING;
        releaseTimer = 0;
    }

    // 2. State Machine Execution
    switch (currentClawState) 
    {
        case OPEN:
            if (closeButton) {
                currentClawState = CLOSING;
                ratchetClaw.tare_position(); // PROS equivalent of resetPosition()
            } else {
                ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                ratchetClaw.brake();
            }
            break;

        case CLOSING:
            if (closeButton && std::abs(ratchetClaw.get_position()) < MAX_CLOSE_DEGREES) {
                ratchetClaw.move(CLAW_CLOSE_POWER);
            } else {
                currentClawState = LOCKED;
            }
            break;

        case LOCKED:
            if (closeButton) {
                ratchetClaw.move(CLAW_CLOSE_POWER);
            } else {
                ratchetClaw.move(CLAW_HOLD_POWER);
            }
            break;

        case RELEASING:
            if (releaseTimer < RELEASE_DURATION_MS) {
                ratchetClaw.move(-CLAW_RELEASE_POWER); // Negative value reverses in PROS
                releaseTimer += 10; // Matches your 10ms pros::delay(10) loop
            } else {
                ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                ratchetClaw.brake();
                currentClawState = OPEN;
            }
            break;
    }
}


void opcontrol() {
  
    ratchetClaw.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  
    while (true) 
    {
        // get joystick positions
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // bind motors to joystick
        chassis.arcade(-leftX, -rightY); 

        bool btnClose = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool btnRelease = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        updateRatchetClaw(btnClose, btnRelease);
        
      
        int cascadePower = 0;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascadePower = 127;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
                   !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            cascadePower = -127;
        }
      
        cascadeLeft.move(cascadePower);
        cascadeRight.move(cascadePower);
        // delay to save resources
        pros::delay(10);
    }
}
