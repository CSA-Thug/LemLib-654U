#pragma once

#include <cstdint>

namespace robot
{

// Call once per driver-control loop with the current button states.
void updateCascade(bool upHeld, bool downHeld, bool dropNewPress, bool resetHeld);

// Blocking autonomous actions return true only when the requested motion succeeds.
// Positions and tolerances are motor degrees; velocity is motor RPM.
bool runCascadeDrop(int velocityRpm, std::uint32_t timeoutMs);
bool runCascadeReset(int resetPower, std::uint32_t timeoutMs);
bool runCascadeToPosition(double positionDegrees, int velocityRpm,
                          std::uint32_t timeoutMs, double toleranceDegrees);

// Read-only state helpers shared by driver logic, autonomous routes, and macros.
bool cascadeDropActive();
bool cascadeAtBottom();
double cascadePositionDegrees();
double cascadeVelocityRpm();
bool cascadeAtPosition(double positionDegrees, double toleranceDegrees);

// Low-level target/stop operations for coordinated non-blocking routines.
void moveCascadeTo(double positionDegrees, int velocityRpm);
void stopCascade();

} // namespace robot
