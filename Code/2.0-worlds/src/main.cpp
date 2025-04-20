/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "setup.h"

#define AUTON_TYPE 0 // 0 for skills, 1 for qual rush, 2 for quali safe, 3 for elims, 4 for not auton

static pros::Task* init_task = nullptr;

void initialize() {
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

    for (int port : {dt_left.get_port(0), dt_left.get_port(1), dt_left.get_port(2), 
                     dt_right.get_port(0), dt_right.get_port(1), dt_right.get_port(2), 
                     intake_motor.get_port(), arm_motors.get_port(0), arm_motors.get_port(1)}) {
        if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none || pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined) { 
            master.rumble("---"); 
        }
    }
    
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0

    optical_sensor.set_integration_time(10);
    optical_sensor.set_led_pwm(100);

    arm_controller.init();

    init_task = new pros::Task([&]() {
        while (pros::competition::is_disabled()) {
            if (selector.get_value() < 100) {
                sideColor = color::blue;
                intake_controller.setState(Intake::SortState::BLUE);
                indicator.set_value(true);
            } else {
                sideColor = color::red;
                intake_controller.setState(Intake::SortState::RED);
                indicator.set_value(false);
            }
            
            pros::delay(100);
        }
    });

    telemetryInit();

    pros::delay(500);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // pidtune();

    if (AUTON_TYPE == 0) {
        skills();
    } else if (AUTON_TYPE == 1) { // quali rush
        if (sideColor == red){
            qual_rush_pos_red();
        } else if (sideColor == blue){
            qual_rush_pos_blue();
        } 
    } else if (AUTON_TYPE == 2) { // quali safe
        qual_safe_pos_blue();
    } else if (AUTON_TYPE == 3) { // elims rush
        if (sideColor == red){
            elims_rush_pos_red();
        } else if (sideColor == blue){
            elims_rush_pos_blue();
        }
    }

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}

void opcontrol() {
    if (arm_controller.getTargetPosition() != Arm::position::CUSTOM) {// only if not not touching laddder at end of match
        arm_controller.moveTo(Arm::position::RETRACT, true);
    }
    master.clear();
    
    //partner.print(0, 0, "op start"); // 0-2 0-14

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_task(drive);
	pros::Task intake_task(intake);
    pros::Task topmech_task(topmech);
    pros::Task piston_task(piston);
    ledsetup();
}