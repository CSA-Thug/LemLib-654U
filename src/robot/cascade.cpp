#include "main.h"
#include "robot/cascade.hpp"
#include "robot/hardware.hpp"

#include "lemlib/api.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot
{
namespace
{
const int CASCADE_MANUAL_VELOCITY = 450;
const int CASCADE_DROP_VELOCITY = 450;
const int CASCADE_RESET_POWER = -40;
const double CASCADE_MIN_DEGREES = 0.0;
const double CASCADE_MAX_DEGREES = 1500.0;
const double CASCADE_SLOWDOWN_DEGREES = 150.0;
const double CASCADE_LIMIT_TOLERANCE_DEGREES = 5.0;
const std::uint32_t CASCADE_DROP_TIMEOUT_MS = 1500;
const std::uint32_t CASCADE_RESET_TIMEOUT_MS = 3000;
const std::uint32_t CASCADE_STALL_GRACE_MS = 150;
const std::uint32_t CASCADE_STALL_CONFIRM_MS = 100;
const double CASCADE_STALL_VELOCITY_RPM = 5.0;

bool isCascadeDropping = false;
std::uint32_t cascadeDropStartTime = 0;
std::uint32_t cascadeDropStallStartTime = 0;
std::uint32_t activeCascadeDropTimeoutMs = CASCADE_DROP_TIMEOUT_MS;
bool isCascadeResetting = false;
bool cascadeResetComplete = false;
bool cascadeResetSucceeded = false;
std::uint32_t cascadeResetStartTime = 0;
std::uint32_t cascadeResetStallStartTime = 0;
int activeCascadeResetPower = CASCADE_RESET_POWER;
std::uint32_t activeCascadeResetTimeoutMs = CASCADE_RESET_TIMEOUT_MS;

void startCascadeDrop(int velocityRpm, std::uint32_t timeoutMs)
{
    isCascadeDropping = true;
    cascadeDropStartTime = pros::millis();
    cascadeDropStallStartTime = 0;
    activeCascadeDropTimeoutMs = timeoutMs;
    hardware::cascadeMotors.move_absolute(CASCADE_MIN_DEGREES, velocityRpm);
}

void startCascadeReset(int resetPower, std::uint32_t timeoutMs)
{
    isCascadeResetting = true;
    cascadeResetComplete = false;
    cascadeResetSucceeded = false;
    cascadeResetStartTime = pros::millis();
    cascadeResetStallStartTime = 0;
    activeCascadeResetPower = resetPower;
    activeCascadeResetTimeoutMs = timeoutMs;
    lemlib::telemetrySink()->info("Cascade reset started");
}
} // namespace

void updateCascade(bool upHeld, bool downHeld, bool dropNewPress, bool resetHeld)
{
    const std::uint32_t now = pros::millis();
    const double currentPos = hardware::cascadeMotors.get_position();
    const double currentVel = std::abs(hardware::cascadeMotors.get_actual_velocity());

    if (!resetHeld) {
        isCascadeResetting = false;
        cascadeResetComplete = false;
        cascadeResetSucceeded = false;
        cascadeResetStallStartTime = 0;
    } else {
        isCascadeDropping = false;
        if (cascadeResetComplete) {
            hardware::cascadeMotors.brake();
            return;
        }
        if (!isCascadeResetting) {
            startCascadeReset(CASCADE_RESET_POWER, CASCADE_RESET_TIMEOUT_MS);
        }

        const std::uint32_t elapsed = now - cascadeResetStartTime;
        if (elapsed >= activeCascadeResetTimeoutMs) {
            hardware::cascadeMotors.brake();
            isCascadeResetting = false;
            cascadeResetComplete = true;
            cascadeResetSucceeded = false;
            lemlib::telemetrySink()->warn("Cascade reset timed out without detecting the lower stop");
            return;
        }

        // A brief low-velocity period at the lower stop is used to establish zero.
        if (elapsed > CASCADE_STALL_GRACE_MS && currentVel < CASCADE_STALL_VELOCITY_RPM) {
            if (cascadeResetStallStartTime == 0) {
                cascadeResetStallStartTime = now;
            } else if (now - cascadeResetStallStartTime >= CASCADE_STALL_CONFIRM_MS) {
                hardware::cascadeMotors.brake();
                hardware::cascadeMotors.tare_position();
                isCascadeResetting = false;
                cascadeResetComplete = true;
                cascadeResetSucceeded = true;
                lemlib::telemetrySink()->info("Cascade encoder zeroed at the lower stop");
                return;
            }
        } else {
            cascadeResetStallStartTime = 0;
        }

        hardware::cascadeMotors.move(activeCascadeResetPower);
        return;
    }

    if (dropNewPress) {
        startCascadeDrop(CASCADE_DROP_VELOCITY, CASCADE_DROP_TIMEOUT_MS);
        return;
    }

    if (upHeld || downHeld) {
        isCascadeDropping = false;
        cascadeDropStallStartTime = 0;

        if (upHeld == downHeld) {
            hardware::cascadeMotors.brake();
            return;
        }

        const double distanceToLimit = upHeld
            ? CASCADE_MAX_DEGREES - currentPos
            : currentPos - CASCADE_MIN_DEGREES;
        if (distanceToLimit <= CASCADE_LIMIT_TOLERANCE_DEGREES) {
            hardware::cascadeMotors.brake();
            return;
        }

        // Reduce speed near either software limit to avoid hitting the hard stop.
        const double speedScale = std::min(1.0, distanceToLimit / CASCADE_SLOWDOWN_DEGREES);
        const int velocity = static_cast<int>(CASCADE_MANUAL_VELOCITY * speedScale);
        hardware::cascadeMotors.move_velocity(upHeld ? velocity : -velocity);
        return;
    }

    if (isCascadeDropping) {
        const std::uint32_t elapsed = now - cascadeDropStartTime;
        const bool reachedBottom = currentPos <= CASCADE_MIN_DEGREES + 10.0;
        const bool timedOut = elapsed >= activeCascadeDropTimeoutMs;
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
            hardware::cascadeMotors.brake();
            if (timedOut && !reachedBottom) {
                lemlib::telemetrySink()->warn("Cascade drop timed out before reaching the bottom");
            } else if (stalled && !reachedBottom) {
                lemlib::telemetrySink()->warn("Cascade drop stalled before reaching the bottom");
            }
        }
        return;
    }

    hardware::cascadeMotors.brake();
}

bool runCascadeDrop(int velocityRpm, std::uint32_t timeoutMs)
{
    if (velocityRpm <= 0 || timeoutMs == 0) {
        lemlib::telemetrySink()->warn("Cascade drop not started: velocity and timeout must be positive");
        return false;
    }
    if (isCascadeDropping || isCascadeResetting) {
        lemlib::telemetrySink()->warn("Cascade drop not started: cascade is already moving");
        return false;
    }

    startCascadeDrop(velocityRpm, timeoutMs);
    while (isCascadeDropping) {
        pros::delay(10);
        updateCascade(false, false, false, false);
    }
    return cascadeAtBottom();
}

bool runCascadeReset(int resetPower, std::uint32_t timeoutMs)
{
    if (resetPower >= 0 || resetPower < -127 || timeoutMs == 0) {
        lemlib::telemetrySink()->warn("Cascade reset not started: power must be negative and timeout positive");
        return false;
    }
    if (isCascadeDropping || isCascadeResetting) {
        lemlib::telemetrySink()->warn("Cascade reset not started: cascade is already moving");
        return false;
    }

    updateCascade(false, false, false, false);
    startCascadeReset(resetPower, timeoutMs);
    while (isCascadeResetting) {
        updateCascade(false, false, false, true);
        if (isCascadeResetting) pros::delay(10);
    }
    return cascadeResetSucceeded;
}

bool runCascadeToPosition(double positionDegrees, int velocityRpm,
                          std::uint32_t timeoutMs, double toleranceDegrees)
{
    if (!std::isfinite(positionDegrees) || positionDegrees < CASCADE_MIN_DEGREES ||
        positionDegrees > CASCADE_MAX_DEGREES || velocityRpm <= 0 ||
        timeoutMs == 0 || !std::isfinite(toleranceDegrees) || toleranceDegrees <= 0.0) {
        lemlib::telemetrySink()->warn("Cascade move not started: invalid position, velocity, timeout, or tolerance");
        return false;
    }
    if (isCascadeDropping || isCascadeResetting) {
        lemlib::telemetrySink()->warn("Cascade move not started: cascade is already moving");
        return false;
    }

    // This bounded wait is intended for autonomous; driver control remains loop-based.
    moveCascadeTo(positionDegrees, velocityRpm);
    const std::uint32_t startTime = pros::millis();
    std::uint32_t stallStartTime = 0;
    while (pros::millis() - startTime < timeoutMs) {
        const std::uint32_t now = pros::millis();
        if (cascadeAtPosition(positionDegrees, toleranceDegrees)) {
            hardware::cascadeMotors.brake();
            return true;
        }

        if (now - startTime > CASCADE_STALL_GRACE_MS &&
            cascadeVelocityRpm() < CASCADE_STALL_VELOCITY_RPM) {
            if (stallStartTime == 0) {
                stallStartTime = now;
            } else if (now - stallStartTime >= CASCADE_STALL_CONFIRM_MS) {
                hardware::cascadeMotors.brake();
                lemlib::telemetrySink()->warn("Cascade move stalled before reaching its target");
                return false;
            }
        } else {
            stallStartTime = 0;
        }
        pros::delay(10);
    }

    hardware::cascadeMotors.brake();
    lemlib::telemetrySink()->warn("Cascade move timed out before reaching its target");
    return false;
}

bool cascadeDropActive() { return isCascadeDropping; }

bool cascadeAtBottom()
{
    return hardware::cascadeMotors.get_position() <= CASCADE_MIN_DEGREES + 10.0;
}

double cascadePositionDegrees() { return hardware::cascadeMotors.get_position(); }

double cascadeVelocityRpm()
{
    return std::abs(hardware::cascadeMotors.get_actual_velocity());
}

bool cascadeAtPosition(double positionDegrees, double toleranceDegrees)
{
    return std::abs(cascadePositionDegrees() - positionDegrees) <= toleranceDegrees;
}

void moveCascadeTo(double positionDegrees, int velocityRpm)
{
    hardware::cascadeMotors.move_absolute(positionDegrees, velocityRpm);
}

void stopCascade()
{
    isCascadeDropping = false;
    cascadeDropStallStartTime = 0;
    hardware::cascadeMotors.brake();
}

} // namespace robot
