


#include "main.h"
#include "lemlib/api.hpp"

extern lemlib::Chassis chassis;

// Example asset retained for autonomous path or future use.
ASSET(example_txt); // '.' replaced with "_" to make C++ happy.

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous()
{
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
}
