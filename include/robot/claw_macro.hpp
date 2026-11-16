#pragma once

namespace robot
{

// Non-blocking scoring macro: drop the cascade, release the claw, then return
// the cascade to the position it had when the macro started.
bool startClawMacro();
void updateClawMacro();
void cancelClawMacro();
bool clawMacroActive();

// Blocking wrapper for use from an autonomous route.
bool runClawMacro();

} // namespace robot
