#pragma once

#include "lemlib/api.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace robot::hardware
{

// Shared robot devices; port assignments and chassis configuration live in hardware.cpp.
extern pros::Controller controller;
extern pros::Motor ratchetClaw;
extern pros::MotorGroup cascadeMotors;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern lemlib::Chassis chassis;

} // namespace robot::hardware
