/*
  __   ___  ___   ___   _  _ 
 /  \ / __)/ _ \ / _ \ / )( \
(_/ /(___ \\__  )\__  )\ \/ /
 (__)(____/(___/ (___/  \__/ 

VBF Robotics 

*/

#include "main.h" 
#include "autonomous.h"
#include "setup.h"

void initialize() {

    // partner.clear();
    //partner.print(0, 0, "init start"); // 0-2 0-14
    chassis.calibrate(); // calibrate the chassis
    arm_rotational_sensor.reset(); // reset the arm sensor

    for (int port : {dt_left.get_port(0), dt_left.get_port(1), dt_left.get_port(2), 
                     dt_right.get_port(0), dt_right.get_port(1), dt_right.get_port(2), 
                     intake_motor.get_port(), arm_motor.get_port()}) {
        if (pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::none || pros::v5::Device::get_plugged_type(port) == pros::v5::DeviceType::undefined) { 
            master.rumble("---"); 
        }
    }
    
    chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    pros::lcd::initialize(); // initialize brain screen
    
    pros::Task positionprint([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            partner.print(0, 0, "%.1f %.1f %.1f", pose.x, pose.y, pose.theta); // 0-2 0-14
            pros::delay(50);
            partner.print(1, 0, "%.1f %s", arm_controller.getAngle(), arm_controller.isInPosition() ? "stopped" : " moving"); // 0-2 0-14
            pros::delay(50);
            partner.print(2, 0, "%s %s ej:%s", sideColor == red ? "red" : "blue", ejectEnabled ? "on" : "off"); // 0-2 0-14
            // pros::delay(50);
            // pros::lcd::print(3,"opotical distance: %i", optical_sensor.get_proximity());
            pros::delay(500);
        }
    });
    pros::lcd::initialize(); // initialize brain screen
    pros::Task screen([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose(); // get chassis position
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Theta: %f", pose.theta);
            pros::delay(50);
        }
    });

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
    });

    pros::lcd::print(0, "%s %s auton", sideColor == color::red ? "red" : "blue"); // 0-2 0-14

    vision_sensor.set_signature(1, &REDSIG);
    vision_sensor.set_signature(2, &BLUESIG);

    //optical_sensor.set_integration_time(20);

    pros::delay(500);

    //partner.print(0, 0, "init done"); // 0-2 0-14
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    
    // partner.print(0, 0, "auton start"); // 0-2 0-14
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    //pidtune();

    //skills();

    //soloAWP_left_neg();
    elims_right();

    if (sideColor == red){
        //soloAWP_right_pos(); // red
        //soloAWP_left_neg(); // red
        //elims_right(); // red
    } else if (sideColor == blue){
        //soloAWP_right_neg(); // blue
        //soloAWP_left_pos(); // blue
        //elims_left(); // blue
    }

}

// void printDistance(){
//     while (true){
//         pros::lcd::print(0,"distance: %i", optical_sensor.get_proximity());
//         pros::delay(500);
//     }
// }

void opcontrol() {
    master.clear();
    // console.println("Running op...");
    partner.print(0, 0, "op start"); // 0-2 0-14
    arm_controller.moveToAngle(16);
    arm_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	pros::Task drive_thread(drive);
	pros::Task intake_thread(intake);
    pros::Task topmech_thread(topmech);
    pros::Task piston_thread(piston);
    ledsetup();
}