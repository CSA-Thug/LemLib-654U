#include "main.h"
#include "robot/hardware.hpp"
#include "robot/program.hpp"

#include "Omega.h"
#include "lemlib/api.hpp"
#include "liblvgl/lvgl.h"

namespace robot
{

namespace
{
void initializeCascade()
{
    hardware::cascadeMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    hardware::cascadeMotors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    hardware::cascadeMotors.tare_position();
}
} // namespace

void initialize()
{
    const bool displayReady = pros::lcd::initialize();
    if (!displayReady) {
        lemlib::telemetrySink()->error("Failed to initialize the Brain display");
    }

    hardware::chassis.calibrate();

    hardware::ratchetClaw.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    hardware::ratchetClaw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    initializeCascade();
    if (!displayReady) return;

    lv_obj_t* const screen = lv_screen_active();
    lv_obj_t* const omegaImage = lv_image_create(screen);
    lv_image_set_src(omegaImage, &Omega);
    lv_image_set_scale(omegaImage, 102);
    lv_obj_center(omegaImage);

    lv_obj_t* const xLabel = lv_label_create(screen);
    lv_obj_t* const yLabel = lv_label_create(screen);
    lv_obj_t* const headingLabel = lv_label_create(screen);
    lv_label_set_text(xLabel, "X: 0.0 in");
    lv_label_set_text(yLabel, "Y: 0.0 in");
    lv_label_set_text(headingLabel, "Heading: 0.0 deg");
    lv_obj_set_pos(xLabel, 20, 70);
    lv_obj_set_pos(yLabel, 20, 100);
    lv_obj_set_pos(headingLabel, 20, 130);
    lv_obj_add_flag(xLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(yLabel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(headingLabel, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t* const modeHint = lv_label_create(screen);
    lv_label_set_text(modeHint, "X: Pose");
    lv_obj_align(modeHint, LV_ALIGN_BOTTOM_RIGHT, -10, -8);

    pros::Task screenTask([omegaImage, xLabel, yLabel, headingLabel, modeHint]() {
        bool showPose = false;
        while (true) {
            if (hardware::controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                showPose = !showPose;
                if (showPose) {
                    lv_obj_add_flag(omegaImage, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_remove_flag(xLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_remove_flag(yLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_remove_flag(headingLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_label_set_text(modeHint, "X: Omega");
                } else {
                    lv_obj_remove_flag(omegaImage, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(xLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(yLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_obj_add_flag(headingLabel, LV_OBJ_FLAG_HIDDEN);
                    lv_label_set_text(modeHint, "X: Pose");
                }
            }

            const lemlib::Pose pose = hardware::chassis.getPose();
            lv_label_set_text_fmt(xLabel, "X: %.1f in", pose.x);
            lv_label_set_text_fmt(yLabel, "Y: %.1f in", pose.y);
            lv_label_set_text_fmt(headingLabel, "Heading: %.1f deg", pose.theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", pose);
            pros::delay(100);
        }
    });
    (void)screenTask;
}

} // namespace robot
