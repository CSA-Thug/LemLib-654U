#include "robot/hardware.hpp"
#include "robot/claw_macro.hpp"
#include "robot/program.hpp"

// Example asset retained for autonomous path or future use.
ASSET(example_txt);

namespace robot
{

void autonomous()
{
    hardware::chassis.moveToPose(20, 15, 90, 4000);
    runClawMacro();
}

} // namespace robot
