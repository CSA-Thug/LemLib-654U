#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h"
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

//pros::Rotation verticalEnc(5);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset behind the robot center (negative).
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset left of the robot center (negative).

//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

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
lemlib::OdomSensors sensors(nullptr, // no vertical tracking wheel
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
