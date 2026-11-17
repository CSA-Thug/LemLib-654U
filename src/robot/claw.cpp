#include "main.h"

#include <cmath>
#include <cstdint>

extern pros::Motor ratchetClaw;

// The intake runs while held; tune the bounded release travel to stop before the physical
// opening stoppers.
const int CLAW_INTAKE_POWER = 108;
const int CLAW_OPEN_SPEED_RPM = 80;
const int CLAW_INTAKE_DIRECTION = 1; // Change to -1 if the claw motor is reversed.
const double CLAW_OPEN_TRAVEL_DEGREES = 275.0;
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