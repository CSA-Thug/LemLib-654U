#include "main.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

extern pros::MotorGroup cascadeMotors;

void initCascade()
{
    cascadeMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    cascadeMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    cascadeMotors.tare_position();
}

// Cascade lift control constants and state tracking.
const int CASCADE_MANUAL_VELOCITY = 450;
const int CASCADE_DROP_VELOCITY = 400;
const double CASCADE_MIN_DEGREES = 0.0;
const double CASCADE_MAX_DEGREES = 2000.0;
const double CASCADE_SLOWDOWN_DEGREES = 150.0;
const double CASCADE_LIMIT_TOLERANCE_DEGREES = 5.0;
const uint32_t CASCADE_DROP_TIMEOUT_MS = 1500;
const uint32_t CASCADE_STALL_GRACE_MS = 150;
const uint32_t CASCADE_STALL_CONFIRM_MS = 100;
const double CASCADE_STALL_VELOCITY_RPM = 5.0;

bool isCascadeDropping = false;
uint32_t cascadeDropStartTime = 0;
uint32_t cascadeDropStallStartTime = 0;

void updateCascade(bool upHeld, bool downHeld, bool dropNewPress)
{
    const uint32_t now = pros::millis();
    const double currentPos = cascadeMotors.get_position();
    const double currentVel = std::abs(cascadeMotors.get_actual_velocity());

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