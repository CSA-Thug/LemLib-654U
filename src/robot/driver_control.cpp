#include "main.h"
#include "robot/cascade.hpp"
#include "robot/claw_macro.hpp"
#include "robot/hardware.hpp"
#include "robot/ratchet_claw.hpp"
#include "robot/program.hpp"

namespace robot
{

void opcontrol()
{
    while (true) {
        const int leftY = hardware::controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int rightX = hardware::controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        hardware::chassis.arcade(-leftY, -rightX);

        const bool btnClose = hardware::controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        const bool btnRelease =
            hardware::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);

        const bool btnCascadeUp = hardware::controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        const bool btnCascadeDown = hardware::controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        const bool btnCascadeDrop =
            hardware::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        const bool btnCascadeReset = hardware::controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        const bool startMacro =
            hardware::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
        // Any direct mechanism command takes priority over the shared scoring macro.
        const bool mechanismOverride =
            btnClose || btnRelease || btnCascadeUp || btnCascadeDown || btnCascadeDrop ||
            btnCascadeReset;

        bool macroOwnsMechanisms = false;
        if (clawMacroActive()) {
            if (mechanismOverride) {
                cancelClawMacro();
            } else {
                updateClawMacro();
                macroOwnsMechanisms = true;
            }
        } else if (startMacro && !mechanismOverride && startClawMacro()) {
            updateClawMacro();
            macroOwnsMechanisms = true;
        }

        if (!macroOwnsMechanisms) {
            updateRatchetClaw(btnClose, btnRelease);
            updateCascade(btnCascadeUp, btnCascadeDown, btnCascadeDrop, btnCascadeReset);
        }

        pros::delay(10);
    }
}

} // namespace robot
