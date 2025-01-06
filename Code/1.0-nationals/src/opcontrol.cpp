#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "setup.h"
#include "arm.h"

int sgn (float number) { return 1 ? number >= 0 : -1; }

//bool reverse_mode = false;
bool ejectEnabled = true;

float driveCurve(float input, float scale) {
    if (scale != 0) {
        return int((powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * input);
    }
    return std::round(input);
}

std::pair<float, float> arcade(int throttle, int turn, float curveGain = 0, float desaturateBias = 0.75) {
    throttle = driveCurve(throttle, curveGain);
    turn = driveCurve(turn, 1);

    //printf("%d %d %d %d \n", throttle, turn, newThrottle, newTurn);

    
    float leftPower = throttle + turn;
    float rightPower = throttle - turn;
    return std::make_pair(leftPower, rightPower);
}

void drive() {
	while (true) {
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { reverse_mode = !reverse_mode; }
        //if (reverse_mode){power *= -1;}

		auto [left, right] = arcade(power, turn, 4.2);
        // auto [left, right] = curvature(power, turn, 7.2);

        //printf("Hello %f %f \n", left, right);
	    dt_left.move(left);
	    dt_right.move(right);

		pros::delay(20 ? CONTROLLER_MODE == bluetooth : 50);
	}
}

//bool fix = true;

// void colorSortOptical() {
//     if (((sideColor == color::blue && optical_sensor.get_hue() < 40) || //red
//         (sideColor == color::red && optical_sensor.get_hue() > 70)) //blue
//         && optical_sensor.get_proximity() > 100 && ejectEnabled) {

//         double initPos = intake_motor.get_position();

//         lemlib::PID intakePID(5, // kP
//                         0.01, // kI
//                         20, // kD
//                         5, // integral anti windup range
//                         false); // don't reset integral when sign of error flips

//         while ((initPos + 480) - intake_motor.get_position() > 10) {
//             intake_motor.move_velocity(600);
//             pros::delay(10);
//         }
        
//         intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//         intake_motor.brake();
//         pros::delay(50);
//         intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//     } 
// }

void colorSortVision() {
    pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);

    intake_motor.move_velocity(600);

    if (((sideColor == color::blue && rtn.signature == 2) || //red
        (sideColor == color::red && rtn.signature == 1)) //blue
        && ejectEnabled && rtn.height > 150) {

        double initPos = intake_motor.get_position();

        while ((initPos + 720) - intake_motor.get_position() > 10) {
            pros::delay(10);
        }
        
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        intake_motor.brake();
        pros::delay(200);
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
}

void redirect() {
    pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);

    intake_motor.move_velocity(400);

    if ((rtn.signature == 2 || //red
        rtn.signature == 1) //blue
        && rtn.height > 150) {

        double initPos = intake_motor.get_position();

        while ((initPos + 500) - intake_motor.get_position() > 10) {
            pros::delay(10);
        }
        
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        intake_motor.brake();
        pros::delay(100);
        // // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

        intake_motor.move_relative(-1200, 300);
        pros::delay(1000);

        // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } 
    pros::delay(10);

    // if (((sideColor == color::blue && rtn.signature == 2) || //red
    //     (sideColor == color::red && rtn.signature == 1)) //blue
    //     && ejectEnabled && rtn.height > 100) {

    //     double initPos = intake_motor.get_position();

    //     lemlib::PID intakePID(8, 0, 0, 1, false);

    //     while ((initPos + 360) - intake_motor.get_position() > 10) {
    //         intake_motor.move_velocity(intakePID.update((initPos + 480) - intake_motor.get_position()));
    //         pros::delay(10);
    //     }
        
    //     intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //     intake_motor.brake();
    //     pros::delay(50);
    //     intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    //     intake_motor.move_velocity(-400);
    //     pros::delay(200);
    // } 

    // intake_motor.move_velocity(600);
    // intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // while (optical_sensor.get_proximity() < 240){
    //     pros::delay(10);
    //     if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) == true) {break;}
    // }
    // pros::delay(95);
    // intake_motor.brake();
    // intake_motor.move_velocity(0);
    // pros::delay(500);
}

//int prevUnjam = 0;

// void unjam() {
//     while (true) {
//         pros::delay(300);
//         if ((std::abs(intake_motor.get_actual_velocity()) < std::abs((0.1*intake_motor.get_target_velocity()))) && (pros::millis() - prevUnjam >= 2000)) {
//             intake_motor.move_velocity(-600);
//             pros::delay(300);
//             intake_motor.move_velocity(600);
//             pros::delay(200);
//             prevUnjam = pros::millis();
//         }
//     }
// }

void intake () {
    bool printLoop = true;
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    //pros::Task unjamthread(unjam);

    while (true) {
        if(!ejectEnabled && printLoop) {master.print(0,0, "Color sort: off");} else if (printLoop) {master.print(0, 0, "Color sort: %s", colorToString(sideColor));}
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            master.clear_line(0);
            ejectEnabled = !ejectEnabled;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) { sideColor == red ? sideColor = color::blue : sideColor = color::red;}
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move_velocity(600);
            if (ejectEnabled) {colorSortVision();}
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move_velocity(-400);
        } else {
            intake_motor.move_velocity(0);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { redirect(); }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
        printLoop = !printLoop;
    }
}

void piston(){
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            doinker_solenoid.toggle();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clamp_solenoid.toggle();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            intake_solenoid.toggle();
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void topmech() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {arm_controller.changeAngle(-10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {arm_controller.changeAngle(10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {arm_controller.home();}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {arm_controller.apartment();}
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
        //on vexnet: 30ms, on bluetooth, delay 50ms
        //double power = partner.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        //arm_motor.move(power);
    }
}