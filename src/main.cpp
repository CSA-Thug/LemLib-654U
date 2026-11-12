#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor ratchetClaw(13, pros::MotorGearset::green);

pros::MotorGroup cascadeMotors({8, 19}, pros::MotorGearset::blue);

pros::MotorGroup leftMotors({2, 3, 10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-11, -12, -20}, pros::MotorGearset::blue);
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
void initCascade() 
{
    cascadeMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascadeMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    cascadeMotors.tare_position();
}

void initialize() 
{
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    ratchetClaw.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    initCascade();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([]() 
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
const int CLAW_CLOSE_POWER = 108;    // ~85% power to close
const int CLAW_RELEASE_POWER = 127;  // 100% power to unlatch

// Timing & Position Thresholds
const uint32_t RELEASE_DURATION_MS = 250; // Safety timeout for opening
const uint32_t STALL_GRACE_MS = 100;      // Ignore low RPM during initial motor spin-up
const double STALL_VELOCITY_RPM = 5.0;    // Velocity threshold indicating claw hit an object
const double MIN_CLOSE_DEGREES = 60.0;    // Minimum travel to engage ratchet teeth
const double MAX_CLOSE_DEGREES = 360.0;   // Hard upper travel limit

enum ClawState { OPEN, CLOSING, LOCKED, RELEASING }; //
ClawState currentClawState = OPEN; //
uint32_t stateStartTime = 0;

void updateRatchetClaw(bool closeButton, bool releaseNewPress) 
{
    // 1. Trigger Release (Overrides everything to unlatch)
    if (releaseNewPress && currentClawState != RELEASING) 
    {
        currentClawState = RELEASING; //
        stateStartTime = pros::millis();
        ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); //
    }

    // 2. State Machine Execution
    switch (currentClawState) 
    {
        case OPEN: //
            if (closeButton) { //
                currentClawState = CLOSING; //
                stateStartTime = pros::millis();
                ratchetClaw.tare_position(); //
            } else {
                ratchetClaw.brake(); //
            }
            break;

        case CLOSING: //
        {
            double currentPos = std::abs(ratchetClaw.get_position()); //
            double currentVel = std::abs(ratchetClaw.get_actual_velocity());
            uint32_t elapsed = pros::millis() - stateStartTime;

            // Check if claw gripped an object and stalled after startup grace period
            bool isStalled = (elapsed > STALL_GRACE_MS) && (currentVel < STALL_VELOCITY_RPM);
            
            if (closeButton && currentPos < MAX_CLOSE_DEGREES && !isStalled) { //
                ratchetClaw.move(CLAW_CLOSE_POWER); //
            } else if (currentPos >= MIN_CLOSE_DEGREES) { //
                // Reached target angle or gripped object: lock the claw
                ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); //
                ratchetClaw.brake(); //
                currentClawState = LOCKED; //
            } else {
                // Button released too early before engaging ratchet: return to open
                ratchetClaw.brake(); //
                currentClawState = OPEN; //
            }
            break;
        }

        case LOCKED: //
            // Hold position firmly; avoid driving power into an already-stalled motor
            ratchetClaw.brake(); //
            break;

        case RELEASING: //
        {
            uint32_t elapsed = pros::millis() - stateStartTime;
            // Run backward until returned to starting 0 position OR safety timeout hits
            if (elapsed < RELEASE_DURATION_MS && ratchetClaw.get_position() > 0.0) { //
                ratchetClaw.move(-CLAW_RELEASE_POWER); //
            } else {
                ratchetClaw.brake(); //
                ratchetClaw.tare_position(); // Reset zero baseline for next close
                currentClawState = OPEN; //
            }
            break;
        }
    }
}


// --- CASCADE LIFT ---
const int CASCADE_MANUAL_POWER = 127;     // Voltage (-127 to 127) for manual up/down
const int CASCADE_DROP_VELOCITY = 450;    // RPM for auto-drop to ground (blue cartridge max is 600)
const double CASCADE_MIN_DEGREES = 0.0;   // Ground / stowed position
const double CASCADE_MAX_DEGREES = 1500.0; // Max height limit (adjust to your lift's physical max)

bool isCascadeDropping = false;

void updateCascade(bool upHeld, bool downHeld, bool dropNewPress) 
{
    double currentPos = cascadeMotors.get_position();

    // 1. Trigger Auto-Drop to ground level
    if (dropNewPress) {
        isCascadeDropping = true;
        cascadeMotors.move_absolute(CASCADE_MIN_DEGREES, CASCADE_DROP_VELOCITY);
        return;
    }

    // 2. Manual Override: Holding up or down cancels an active auto-drop
    if (upHeld || downHeld) {
        isCascadeDropping = false;

        if (upHeld && currentPos < CASCADE_MAX_DEGREES) {
            cascadeMotors.move(CASCADE_MANUAL_POWER);
        } 
        else if (downHeld && currentPos > CASCADE_MIN_DEGREES) {
            cascadeMotors.move(-CASCADE_MANUAL_POWER);
        } 
        else {
            // At soft limit: hold position
            cascadeMotors.brake();
        }
        return;
    }

    // 3. Handle ongoing Auto-Drop or Hold in place
    if (isCascadeDropping) {
        // Stop auto-drop once within 10 degrees of ground
        if (currentPos <= CASCADE_MIN_DEGREES + 10.0) {
            isCascadeDropping = false;
            cascadeMotors.brake();
        }
    } else {
        // Neither trigger is held and not dropping: hold current height
        cascadeMotors.brake();
    }
}


void opcontrol() {
  
    while (true) 
    {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // bind motors to joystick
        chassis.arcade(-leftY, -rightX); 

        //bool isDriving = (std::abs(leftY) > DRIVE_MOVE_THRESHOLD || std::abs(rightX) > DRIVE_MOVE_THRESHOLD);
      
        bool btnClose = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool btnRelease = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);

        updateRatchetClaw(btnClose, btnRelease);

        bool btnCascadeUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool btnCascadeDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool btnCascadeDrop = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        updateCascade(btnCascadeUp, btnCascadeDown, btnCascadeDrop);
      
        // delay to save resources
        pros::delay(10);
    }
}
