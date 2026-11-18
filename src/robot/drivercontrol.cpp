#include "main.h"

extern pros::Controller controller;
extern lemlib::Chassis chassis;

void opcontrol()
{
    while (true) 
    {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // bind motors to joystick
        chassis.arcade(-leftY, -rightX); 

        //bool isDriving = (std::abs(leftY) > DRIVE_MOVE_THRESHOLD || std::abs(rightX) > DRIVE_MOVE_THRESHOLD);
      
        bool btnClose = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool btnRelease = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2);
        updateRatchetClaw(btnClose, btnRelease);

        bool btnCascadeUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool btnCascadeDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool btnCascadeDrop = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
        updateCascade(btnCascadeUp, btnCascadeDown, btnCascadeDrop);
      
        // delay to save resources
        pros::delay(10);
    }
}
