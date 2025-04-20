/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "setup.h"

#define AUTON_SLOT 6

/* -------- ROBOT PROGRAMS --------
1 - awp pos (allience stake + mogo of 3 (doinker grab 2))
2 - new non-awp pos (mogo of 1 + mogo of 3 (doinker grab 2))
3 - old non-awp pos (mogo of 1 + mogo of 2)
4 - awp neg (allience stake + mogo of 3 (doinker grab 2))
5 - 
6 - skills
7 - 
8 - doom :)
*/

static pros::Task* init_task = nullptr;

void initialize() {
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

    // for (int port : {dt_left.get_port(0), dt_left.get_port(1), dt_left.get_port(2), 
    //                  dt_right.get_port(0), dt_right.get_port(1), dt_right.get_port(2), 
    //                  intake_motor.get_port(), arm_motors.get_port(0), arm_motors.get_port(1)}) {
    //     if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none || pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined) { 
    //         master.rumble("---"); 
    //     }
    // }
    
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

    if (AUTON_SLOT == 6) { skills(); }  

    // qualis
    if (AUTON_SLOT == 1) { // awp (allience stake + mogo of 3 (doinker grab 2)) end at ladder
        if (sideColor == red){
            qual_1_pos_red();
        } else if (sideColor == blue){
            qual_1_pos_blue();
        } 
    } else if (AUTON_SLOT == 2) { // new non-awp (mogo of 1 + mogo of 3 (doinker grab 2)) end at ladder
        if (sideColor == red){
            qual_2_pos_red();
        } else if (sideColor == blue){
            qual_2_pos_blue();
        }
    } else if (AUTON_SLOT == 3) { // old non-awp (mogo of 1 + mogo of 2) end at ladder
        if (sideColor == red){
            qual_3_pos_red();
        } else if (sideColor == blue){
            qual_3_pos_blue();
        }
    } else if (AUTON_SLOT == 4) { // awp neg (allience stake + mogo of 3 (doinker grab 2)) end at ladder
        if (sideColor == red){
            qual_1_neg_red();
        } else if (sideColor == blue){
            qual_1_neg_blue();
        }
    } 

    // elims
    /*
    if (AUTON_SLOT == 1) { // awp (allience stake + mogo of 3 (doinker grab 2)) end at mid
        if (sideColor == red){
            elim_1_pos_red();
        } else if (sideColor == blue){
            elim_1_pos_blue();
        }
    } else if (AUTON_SLOT == 2) { // new non-awp (mogo of 1 + mogo of 3 (doinker grab 2)) end at mid
        if (sideColor == red){
            elim_2_pos_red();
        } else if (sideColor == blue){
            elim_2_pos_blue();
        }
    } else if (AUTON_SLOT == 3) { // old non-awp (mogo of 1 + mogo of 2) end at mid
        if (sideColor == red){
            elim_3_pos_red();
        } else if (sideColor == blue){
            elim_3_pos_blue();
        }
    } else if (AUTON_SLOT == 4) { // awp neg (allience stake + mogo of 3 (doinker grab 2)) end at mid
        if (sideColor == red){
            elim_1_neg_red();
        } else if (sideColor == blue){
            elim_1_neg_blue();
        }
    } 
    */

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
    // ledsetup();
}