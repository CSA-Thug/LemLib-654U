#pragma once

#include <cstdint>

namespace robot
{

// Call once per driver-control loop with the current close/release button states.
void updateRatchetClaw(bool closeHeld, bool releaseNewPress);

// Blocking autonomous actions return true only when the claw reaches its goal.
// Close power uses PROS's 0-127 scale; release travel uses degrees and RPM.
bool runRatchetClawClose(int closePower, std::uint32_t timeoutMs);
bool runRatchetClawRelease(double travelDegrees, int speedRpm,
                           std::uint32_t timeoutMs);

// State queries and non-blocking release controls for coordinated routines.
bool ratchetClawNeedsOpening();
bool ratchetClawIsOpening();
bool startRatchetClawRelease();
bool startRatchetClawRelease(double travelDegrees, int speedRpm,
                             std::uint32_t timeoutMs);
void stopRatchetClaw();

} // namespace robot
