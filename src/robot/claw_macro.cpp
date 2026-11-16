#include "robot/cascade.hpp"
#include "robot/claw_macro.hpp"
#include "robot/hardware.hpp"
#include "robot/ratchet_claw.hpp"

#include "lemlib/api.hpp"

#include <cmath>
#include <cstdint>

namespace robot
{
namespace
{
enum class MacroState
{
    Idle,
    Dropping,
    Retracting,
    Complete,
    Failed,
    Cancelled
};

constexpr double CASCADE_BOTTOM_DEGREES = 0.0;
// Start opening before the cascade reaches bottom so the held pin has downward momentum.
constexpr double RELEASE_HEIGHT_ABOVE_BOTTOM_DEGREES = 80.0;
constexpr int RETRACT_VELOCITY_RPM = 450;
constexpr double RETRACT_POSITION_TOLERANCE_DEGREES = 5.0;
constexpr std::uint32_t RETRACT_TIMEOUT_MS = 3000;
constexpr std::uint32_t STALL_GRACE_MS = 150;
constexpr std::uint32_t STALL_CONFIRM_MS = 100;
constexpr double STALL_VELOCITY_RPM = 5.0;

MacroState state = MacroState::Idle;
double startingCascadePosition = 0.0;
std::uint32_t retractStartTime = 0;
std::uint32_t retractStallStartTime = 0;
bool releaseStarted = false;
} // namespace

bool startClawMacro()
{
    if (clawMacroActive() ||
        cascadeDropActive() ||
        ratchetClawIsOpening()) {
        lemlib::telemetrySink()->warn(
            "Claw macro not started: cascade is already dropping or claw release is active"
        );
        state = MacroState::Failed;
        return false;
    }

    startingCascadePosition = cascadePositionDegrees();
    retractStartTime = 0;
    retractStallStartTime = 0;
    releaseStarted = false;
    state = MacroState::Dropping;
    updateCascade(false, false, true, false);
    lemlib::telemetrySink()->info("Claw macro started");
    return true;
}

void updateClawMacro()
{
    if (!clawMacroActive()) return;

    updateRatchetClaw(false, false);

    if (state == MacroState::Dropping) {
        updateCascade(false, false, false, false);

        if (!releaseStarted &&
            cascadePositionDegrees() <=
                CASCADE_BOTTOM_DEGREES + RELEASE_HEIGHT_ABOVE_BOTTOM_DEGREES) {
            if (!startRatchetClawRelease()) {
                stopCascade();
                state = MacroState::Failed;
                lemlib::telemetrySink()->warn("Claw macro could not start ratchet release");
                return;
            }
            releaseStarted = true;
        }

        if (cascadeDropActive()) return;
        if (!cascadeAtBottom()) {
            stopRatchetClaw();
            state = MacroState::Failed;
            lemlib::telemetrySink()->warn("Claw macro cascade drop did not reach the bottom");
            return;
        }

        if (!releaseStarted) {
            if (!startRatchetClawRelease()) {
                stopCascade();
                state = MacroState::Failed;
                lemlib::telemetrySink()->warn("Claw macro could not start ratchet release");
                return;
            }
            releaseStarted = true;
        }
        if (ratchetClawIsOpening()) return;
        if (ratchetClawNeedsOpening()) {
            state = MacroState::Failed;
            lemlib::telemetrySink()->warn("Claw macro ratchet release timed out");
            return;
        }

        if (cascadeAtPosition(startingCascadePosition, RETRACT_POSITION_TOLERANCE_DEGREES)) {
            state = MacroState::Complete;
            lemlib::telemetrySink()->info("Claw macro completed");
            return;
        }

        // "Retract" means return to the cascade height at macro start.
        moveCascadeTo(startingCascadePosition, RETRACT_VELOCITY_RPM);
        retractStartTime = pros::millis();
        retractStallStartTime = 0;
        state = MacroState::Retracting;
        return;
    }

    if (state == MacroState::Retracting) {
        if (cascadeAtPosition(startingCascadePosition, RETRACT_POSITION_TOLERANCE_DEGREES)) {
            stopCascade();
            state = MacroState::Complete;
            lemlib::telemetrySink()->info("Claw macro completed");
            return;
        }

        const std::uint32_t now = pros::millis();
        if (now - retractStartTime > STALL_GRACE_MS &&
            cascadeVelocityRpm() < STALL_VELOCITY_RPM) {
            if (retractStallStartTime == 0) {
                retractStallStartTime = now;
            } else if (now - retractStallStartTime >= STALL_CONFIRM_MS) {
                stopCascade();
                state = MacroState::Failed;
                lemlib::telemetrySink()->warn("Claw macro cascade retraction stalled");
                return;
            }
        } else {
            retractStallStartTime = 0;
        }

        if (now - retractStartTime >= RETRACT_TIMEOUT_MS) {
            stopCascade();
            state = MacroState::Failed;
            lemlib::telemetrySink()->warn("Claw macro cascade retraction timed out");
        }
    }
}

void cancelClawMacro()
{
    if (!clawMacroActive()) return;
    stopCascade();
    stopRatchetClaw();
    state = MacroState::Cancelled;
    lemlib::telemetrySink()->info("Claw macro cancelled");
}

bool clawMacroActive()
{
    return state == MacroState::Dropping || state == MacroState::Retracting;
}

bool runClawMacro()
{
    if (!startClawMacro()) return false;

    while (clawMacroActive()) {
        updateClawMacro();
        pros::delay(10);
    }

    return state == MacroState::Complete;
}

} // namespace robot
