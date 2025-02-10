/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "setup.h"

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
    
    pros::Task screen_telemetry_task(screenTelemetry);
    pros::Task sd_telemetry_task(sdTelemetry);

    pros::Task selection([&]() {
        while (pros::competition::is_disabled()) {
            if (selector.get_value() < 100) {
                sideColor = color::blue;
                indicator.set_value(true);
            } else {
                sideColor = color::red;
                indicator.set_value(false);
            }
            pros::delay(100);
        }
        switch (sideColor) {
            case color::blue: intake_controller.setState(Intake::SortState::BLUE); break;
            case color::red: intake_controller.setState(Intake::SortState::RED); break;
            default: intake_controller.setState(Intake::SortState::OFF); break;
        }
    });

    optical_sensor.set_integration_time(20);
    optical_sensor.set_led_pwm(100);

    arm_controller.init();

    pros::delay(500);
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // uncommment the auton you want to run

    // pidtune();
    // skills();

    //quali
    if (sideColor == red){
        qual_pos_blue();
    } else if (sideColor == blue){
       qual_pos_blue();
    }

    // elims
    // if (sideColor == red){
    //     elims_pos_red();
    // } else if (sideColor == blue){
    //     elims_pos_blue();
    // }
}

void opcontrol() {
    master.clear();
    
    // console.println("Running op...");
    //partner.print(0, 0, "op start"); // 0-2 0-14

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_task(drive);
	pros::Task intake_task(intake);
    pros::Task topmech_task(topmech);
    pros::Task piston_task(piston);
    ledsetup();
}