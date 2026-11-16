#include "main.h"
#include "robot/hardware.hpp"
#include "robot/ratchet_claw.hpp"

#include "lemlib/api.hpp"

#include <cmath>
#include <cstdint>

namespace robot
{
namespace
{
const int CLAW_INTAKE_POWER = 108;
const int CLAW_OPEN_SPEED_RPM = 127;
const int CLAW_INTAKE_DIRECTION = 1;
const double CLAW_OPEN_TRAVEL_DEGREES = 300.0;
const double CLAW_OPEN_TOLERANCE_DEGREES = 5.0;
const std::uint32_t CLAW_OPEN_TIMEOUT_MS = 750;
const std::uint32_t CLAW_STALL_GRACE_MS = 100;
const std::uint32_t CLAW_STALL_CONFIRM_MS = 100;
const double CLAW_STALL_VELOCITY_RPM = 5.0;

enum class ClawState
{
    Idle,
    Intaking,
    Opening
};

ClawState clawState = ClawState::Idle;
bool clawNeedsOpening = false;
bool clawIntakeInhibited = false;
bool clawIntakeStalled = false;
std::uint32_t clawStateStartTime = 0;
std::uint32_t clawStallStartTime = 0;
double clawOpenStartPosition = 0.0;
double activeClawOpenTravelDegrees = CLAW_OPEN_TRAVEL_DEGREES;
int activeClawOpenSpeedRpm = CLAW_OPEN_SPEED_RPM;
std::uint32_t activeClawOpenTimeoutMs = CLAW_OPEN_TIMEOUT_MS;
} // namespace

namespace
{
void updateRatchetClawWithParameters(bool intakeHeld, bool releaseNewPress,
                                     int intakePower, double releaseTravelDegrees,
                                     int releaseSpeedRpm, std::uint32_t releaseTimeoutMs)
{
    const std::uint32_t now = pros::millis();
    const double position = hardware::ratchetClaw.get_position();

    if (!intakeHeld) {
        clawIntakeInhibited = false;
        clawIntakeStalled = false;
        clawStallStartTime = 0;
    }

    if (clawState == ClawState::Opening) {
        // Use measured travel, rather than only the motor target, to confirm release.
        const double openProgress =
            (clawOpenStartPosition - position) * CLAW_INTAKE_DIRECTION;
        const bool reachedOpenLimit =
            openProgress >= activeClawOpenTravelDegrees - CLAW_OPEN_TOLERANCE_DEGREES;
        const bool timedOut = now - clawStateStartTime >= activeClawOpenTimeoutMs;

        if (reachedOpenLimit || timedOut) {
            hardware::ratchetClaw.brake();
            clawState = ClawState::Idle;
            if (reachedOpenLimit) {
                clawNeedsOpening = false;
                lemlib::telemetrySink()->info("Claw release completed");
            } else {
                lemlib::telemetrySink()->warn("Claw release timed out before reaching its target");
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
        activeClawOpenTravelDegrees = releaseTravelDegrees;
        activeClawOpenSpeedRpm = releaseSpeedRpm;
        activeClawOpenTimeoutMs = releaseTimeoutMs;
        const double openTargetPosition =
            position - CLAW_INTAKE_DIRECTION * activeClawOpenTravelDegrees;
        hardware::ratchetClaw.move_absolute(openTargetPosition, activeClawOpenSpeedRpm);
        return;
    }

    if (intakeHeld && !clawIntakeInhibited && !clawIntakeStalled) {
        if (clawState != ClawState::Intaking) {
            clawState = ClawState::Intaking;
            clawStateStartTime = now;
        }
        clawNeedsOpening = true;

        const double velocity = std::abs(hardware::ratchetClaw.get_actual_velocity());
        const std::uint32_t intakeElapsed = now - clawStateStartTime;
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
            hardware::ratchetClaw.brake();
            clawState = ClawState::Idle;
            clawIntakeStalled = true;
            lemlib::telemetrySink()->warn("Claw intake stopped after a stall");
            return;
        }

        hardware::ratchetClaw.move(intakePower * CLAW_INTAKE_DIRECTION);
        return;
    }

    if (clawState == ClawState::Intaking) {
        clawState = ClawState::Idle;
    }
    hardware::ratchetClaw.brake();
}
} // namespace

void updateRatchetClaw(bool intakeHeld, bool releaseNewPress)
{
    updateRatchetClawWithParameters(intakeHeld, releaseNewPress, CLAW_INTAKE_POWER,
                                    CLAW_OPEN_TRAVEL_DEGREES, CLAW_OPEN_SPEED_RPM,
                                    CLAW_OPEN_TIMEOUT_MS);
}

bool runRatchetClawClose(int closePower, std::uint32_t timeoutMs)
{
    if (closePower <= 0 || closePower > 127 || timeoutMs == 0 ||
        clawState == ClawState::Opening) {
        lemlib::telemetrySink()->warn("Claw close not started: invalid parameters or release already active");
        return false;
    }

    updateRatchetClaw(false, false);
    const std::uint32_t startTime = pros::millis();
    while (pros::millis() - startTime < timeoutMs) {
        updateRatchetClawWithParameters(true, false, closePower,
                                        CLAW_OPEN_TRAVEL_DEGREES, CLAW_OPEN_SPEED_RPM,
                                        CLAW_OPEN_TIMEOUT_MS);
        // On this ratchet claw, a sustained stall indicates it has closed on its load.
        if (clawIntakeStalled) return true;
        if (clawState != ClawState::Intaking) return false;
        pros::delay(10);
    }

    hardware::ratchetClaw.brake();
    clawState = ClawState::Idle;
    clawIntakeInhibited = true;
    lemlib::telemetrySink()->warn("Claw close timed out before reaching its stall limit");
    return false;
}

bool runRatchetClawRelease(double travelDegrees, int speedRpm,
                           std::uint32_t timeoutMs)
{
    if (!startRatchetClawRelease(travelDegrees, speedRpm, timeoutMs)) return false;
    while (clawState == ClawState::Opening) {
        updateRatchetClaw(false, false);
        pros::delay(10);
    }
    return !clawNeedsOpening;
}

bool ratchetClawNeedsOpening() { return clawNeedsOpening; }

bool ratchetClawIsOpening() { return clawState == ClawState::Opening; }

bool startRatchetClawRelease()
{
    return startRatchetClawRelease(CLAW_OPEN_TRAVEL_DEGREES, CLAW_OPEN_SPEED_RPM,
                                   CLAW_OPEN_TIMEOUT_MS);
}

bool startRatchetClawRelease(double travelDegrees, int speedRpm,
                             std::uint32_t timeoutMs)
{
    if (!std::isfinite(travelDegrees) || travelDegrees <= 0.0 ||
        speedRpm <= 0 || timeoutMs == 0) {
        lemlib::telemetrySink()->warn("Claw release not started: invalid travel, speed, or timeout");
        return false;
    }
    if (clawState == ClawState::Opening) return false;
    clawNeedsOpening = true;
    updateRatchetClawWithParameters(false, true, CLAW_INTAKE_POWER,
                                    travelDegrees, speedRpm, timeoutMs);
    return clawState == ClawState::Opening;
}

void stopRatchetClaw()
{
    hardware::ratchetClaw.brake();
    if (clawState == ClawState::Opening) clawState = ClawState::Idle;
}

} // namespace robot
