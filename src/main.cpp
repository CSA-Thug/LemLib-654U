#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor ratchetClaw(14, pros::MotorGearset::green);

pros::MotorGroup cascadeMotors({13, 4}, pros::MotorGearset::blue);

pros::MotorGroup leftMotors({11, 19, -20}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({2, -9, -10}, pros::MotorGearset::blue);
// Inertial Sensor on port 15
pros::Imu imu(15);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor on port 6, not reversed.
pros::Rotation horizontalEnc(6);
// vertical tracking wheel encoder. Rotation sensor on port 5, not reversed.
pros::Rotation verticalEnc(5);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset behind the robot center (negative).
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset left of the robot center (negative).
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              14, // track width in inches
                              lemlib::Omniwheel::NEW_275, // using 2.75" omni wheels
                              600, // drivetrain RPM
                              8 // horizontal drift value
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti-windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in ms
                                            3, // large error range, in inches
                                            500, // large error range timeout, in ms
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti-windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in ms
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in ms
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // secondary vertical tracking wheel, not used
                            &horizontal, // horizontal tracking wheel
                            nullptr, // secondary horizontal tracking wheel, not used
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output before drivetrain starts moving
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output before drivetrain starts moving
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
    const bool displayReady = pros::lcd::initialize();

    chassis.calibrate(); // calibrate sensors

    ratchetClaw.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    initCascade();
    if (!displayReady) return;

    lv_obj_t* const screen = lv_screen_active();

    lv_obj_t* const xLabel = lv_label_create(screen);
    lv_obj_t* const yLabel = lv_label_create(screen);
    lv_obj_t* const headingLabel = lv_label_create(screen);
    lv_label_set_text(xLabel, "X: 0.0 in");
    lv_label_set_text(yLabel, "Y: 0.0 in");
    lv_label_set_text(headingLabel, "Heading: 0.0 deg");
    lv_obj_set_pos(xLabel, 20, 70);
    lv_obj_set_pos(yLabel, 20, 100);
    lv_obj_set_pos(headingLabel, 20, 130);

    pros::Task screenTask([xLabel, yLabel, headingLabel]()
    {
        while (true) 
        {
            const lemlib::Pose pose = chassis.getPose();
            lv_label_set_text_fmt(xLabel, "X: %.1f in", pose.x);
            lv_label_set_text_fmt(yLabel, "Y: %.1f in", pose.y);
            lv_label_set_text_fmt(headingLabel, "Heading: %.1f deg", pose.theta);
            pros::delay(100);
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

// Example asset retained for autonomous path or future use.
ASSET(example_txt); // '.' replaced with "_" to make C++ happy.

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

// The intake runs while held; tune the bounded release travel to stop before the physical
// opening stoppers.
const int CLAW_INTAKE_POWER = 108;
const int CLAW_OPEN_SPEED_RPM = 127;
const int CLAW_INTAKE_DIRECTION = 1; // Change to -1 if the claw motor is reversed.
const double CLAW_OPEN_TRAVEL_DEGREES = 300.0;
const double CLAW_OPEN_TOLERANCE_DEGREES = 5.0;
const uint32_t CLAW_OPEN_TIMEOUT_MS = 750;
const uint32_t CLAW_STALL_GRACE_MS = 100;
const uint32_t CLAW_STALL_CONFIRM_MS = 100;
const double CLAW_STALL_VELOCITY_RPM = 5.0;

enum class ClawState { Idle, Intaking, Opening };
ClawState clawState = ClawState::Idle;
bool clawNeedsOpening = false;
bool clawIntakeInhibited = false;
bool clawIntakeStalled = false;
uint32_t clawStateStartTime = 0;
uint32_t clawStallStartTime = 0;
double clawOpenStartPosition = 0.0;

void updateRatchetClaw(bool intakeHeld, bool releaseNewPress)
{
    const uint32_t now = pros::millis();
    const double position = ratchetClaw.get_position();

    if (!intakeHeld) {
        clawIntakeInhibited = false;
        clawIntakeStalled = false;
        clawStallStartTime = 0;
    }

    if (clawState == ClawState::Opening) {
        const double openProgress =
            (clawOpenStartPosition - position) * CLAW_INTAKE_DIRECTION;
        const bool reachedOpenLimit =
            openProgress >= CLAW_OPEN_TRAVEL_DEGREES - CLAW_OPEN_TOLERANCE_DEGREES;
        const bool timedOut = now - clawStateStartTime >= CLAW_OPEN_TIMEOUT_MS;

        if (reachedOpenLimit || timedOut) {
            ratchetClaw.brake();
            clawState = ClawState::Idle;
            if (reachedOpenLimit) {
                clawNeedsOpening = false;
            }
            return;
        }
        return;
    }

    if (releaseNewPress && clawNeedsOpening) {
        clawState = ClawState::Opening;
        clawStateStartTime = now;
        clawIntakeInhibited = true;
        clawOpenStartPosition = position;
        const double openTargetPosition =
            position - CLAW_INTAKE_DIRECTION * CLAW_OPEN_TRAVEL_DEGREES;
        ratchetClaw.move_absolute(openTargetPosition, CLAW_OPEN_SPEED_RPM);
        return;
    }

    if (intakeHeld && !clawIntakeInhibited && !clawIntakeStalled) {
        if (clawState != ClawState::Intaking) {
            clawState = ClawState::Intaking;
            clawStateStartTime = now;
        }
        clawNeedsOpening = true;

        const double velocity = std::abs(ratchetClaw.get_actual_velocity());
        const uint32_t intakeElapsed = now - clawStateStartTime;
        if (intakeElapsed > CLAW_STALL_GRACE_MS && velocity < CLAW_STALL_VELOCITY_RPM) {
            if (clawStallStartTime == 0) {
                clawStallStartTime = now;
            }
        } else {
            clawStallStartTime = 0;
        }
        const bool stalled = clawStallStartTime != 0 &&
                             now - clawStallStartTime >= CLAW_STALL_CONFIRM_MS;
        if (stalled) {
            ratchetClaw.brake();
            clawState = ClawState::Idle;
            clawIntakeStalled = true;
            return;
        }

        ratchetClaw.move(CLAW_INTAKE_POWER * CLAW_INTAKE_DIRECTION);
        return;
    }

    if (clawState == ClawState::Intaking) {
        clawState = ClawState::Idle;
    }
    ratchetClaw.brake();
}

// Cascade lift control constants and state tracking.
const int CASCADE_MANUAL_VELOCITY = 450;
const int CASCADE_DROP_VELOCITY = 450;
const int CASCADE_RESET_POWER = -40; // Slow downward power for re-zeroing.
const double CASCADE_MIN_DEGREES = 0.0;
const double CASCADE_MAX_DEGREES = 1500.0;
const double CASCADE_SLOWDOWN_DEGREES = 150.0;
const double CASCADE_LIMIT_TOLERANCE_DEGREES = 5.0;
const uint32_t CASCADE_DROP_TIMEOUT_MS = 1500;
const uint32_t CASCADE_RESET_TIMEOUT_MS = 3000;
const uint32_t CASCADE_STALL_GRACE_MS = 150;
const uint32_t CASCADE_STALL_CONFIRM_MS = 100;
const double CASCADE_STALL_VELOCITY_RPM = 5.0;

bool isCascadeDropping = false;
uint32_t cascadeDropStartTime = 0;
uint32_t cascadeDropStallStartTime = 0;
bool isCascadeResetting = false;
bool cascadeResetComplete = false;
uint32_t cascadeResetStartTime = 0;
uint32_t cascadeResetStallStartTime = 0;

void updateCascade(bool upHeld, bool downHeld, bool dropNewPress, bool resetHeld)
{
    const uint32_t now = pros::millis();
    const double currentPos = cascadeMotors.get_position();
    const double currentVel = std::abs(cascadeMotors.get_actual_velocity());

    if (!resetHeld) {
        isCascadeResetting = false;
        cascadeResetComplete = false;
        cascadeResetStallStartTime = 0;
    } else {
        isCascadeDropping = false;
        if (cascadeResetComplete) {
            cascadeMotors.brake();
            return;
        }
        if (!isCascadeResetting) {
            isCascadeResetting = true;
            cascadeResetStartTime = now;
            cascadeResetStallStartTime = 0;
        }

        const uint32_t elapsed = now - cascadeResetStartTime;
        if (elapsed >= CASCADE_RESET_TIMEOUT_MS) {
            cascadeMotors.brake();
            isCascadeResetting = false;
            cascadeResetComplete = true;
            return;
        }

        if (elapsed > CASCADE_STALL_GRACE_MS && currentVel < CASCADE_STALL_VELOCITY_RPM) {
            if (cascadeResetStallStartTime == 0) {
                cascadeResetStallStartTime = now;
            } else if (now - cascadeResetStallStartTime >= CASCADE_STALL_CONFIRM_MS) {
                cascadeMotors.brake();
                cascadeMotors.tare_position();
                isCascadeResetting = false;
                cascadeResetComplete = true;
                return;
            }
        } else {
            cascadeResetStallStartTime = 0;
        }

        cascadeMotors.move(CASCADE_RESET_POWER);
        return;
    }

    if (dropNewPress) {
        isCascadeDropping = true;
        cascadeDropStartTime = now;
        cascadeDropStallStartTime = 0;
        cascadeMotors.move_absolute(CASCADE_MIN_DEGREES, CASCADE_DROP_VELOCITY);
        return;
    }

    if (upHeld || downHeld) {
        isCascadeDropping = false;
        cascadeDropStallStartTime = 0;

        if (upHeld == downHeld) {
            cascadeMotors.brake();
            return;
        }

        const double distanceToLimit = upHeld
            ? CASCADE_MAX_DEGREES - currentPos
            : currentPos - CASCADE_MIN_DEGREES;
        if (distanceToLimit <= CASCADE_LIMIT_TOLERANCE_DEGREES) {
            cascadeMotors.brake();
            return;
        }

        const double speedScale = std::min(1.0, distanceToLimit / CASCADE_SLOWDOWN_DEGREES);
        const int velocity = static_cast<int>(CASCADE_MANUAL_VELOCITY * speedScale);
        cascadeMotors.move_velocity(upHeld ? velocity : -velocity);
        return;
    }

    if (isCascadeDropping) {
        const uint32_t elapsed = now - cascadeDropStartTime;
        const bool reachedBottom = currentPos <= CASCADE_MIN_DEGREES + 10.0;
        const bool timedOut = elapsed >= CASCADE_DROP_TIMEOUT_MS;
        bool stalled = false;

        if (elapsed > CASCADE_STALL_GRACE_MS && currentVel < CASCADE_STALL_VELOCITY_RPM) {
            if (cascadeDropStallStartTime == 0) {
                cascadeDropStallStartTime = now;
            } else {
                stalled = now - cascadeDropStallStartTime >= CASCADE_STALL_CONFIRM_MS;
            }
        } else {
            cascadeDropStallStartTime = 0;
        }

        if (reachedBottom || timedOut || stalled) {
            isCascadeDropping = false;
            cascadeMotors.brake();
        }
        return;
    }

    cascadeMotors.brake();
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
        bool btnCascadeReset = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        updateCascade(btnCascadeUp, btnCascadeDown, btnCascadeDrop, btnCascadeReset);
      
        // delay to save resources
        pros::delay(10);
    }
}
