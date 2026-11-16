#include "robot/hardware.hpp"

namespace robot::hardware
{

// V5 smart-port assignments; negative motor ports reverse the motor direction.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor ratchetClaw(13, pros::MotorGearset::green);

pros::MotorGroup cascadeMotors({8, 19}, pros::MotorGearset::blue);

pros::MotorGroup leftMotors({2, 3, 10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-11, -12, -20}, pros::MotorGearset::blue);

pros::Imu imu(15);

namespace
{
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    14,
    lemlib::Omniwheel::NEW_275,
    600,
    8
);

lemlib::ControllerSettings linearController(
    10,
    0,
    3,
    3,
    1,
    100,
    3,
    500,
    20
);

lemlib::ControllerSettings angularController(
    2,
    0,
    10,
    3,
    1,
    100,
    3,
    500,
    0
);

lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
} // namespace

lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

} // namespace robot::hardware
