#pragma once

namespace robot
{

// Implementations of the PROS lifecycle callbacks, separated from main.cpp.
void initialize();
void autonomous();
void opcontrol();

} // namespace robot
