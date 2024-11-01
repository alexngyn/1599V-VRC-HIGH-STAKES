#include "pros/motors.h"
#include "setup.h"
//#include "gui.h"
#include "arm.h"

int sgn (float number) { return 1 ? number >= 0 : -1 ; }

bool reverse_mode = false;
bool ejectEnabled = true;

float driveCurve(float input, float scale) {
    if (scale != 0) {
        return int((powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * input);
    }
    return std::round(input);
}

std::pair<float, float> arcade(int throttle, int turn, float curveGain = 0, float desaturateBias = 0.75) {
    throttle = driveCurve(throttle, curveGain);
    turn = driveCurve(turn, 7.2);

    //printf("%d %d %d %d \n", throttle, turn, newThrottle, newTurn);

    
    float leftPower = throttle + turn;
    float rightPower = throttle - turn;
    return std::make_pair(leftPower, rightPower);
}

void drive() {
	while (true) {
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { reverse_mode = !reverse_mode; }
        if (reverse_mode){power *= -1;}

		auto [left, right] = arcade(power, turn, 4.2);
        // auto [left, right] = curvature(power, turn, 7.2);

        //printf("Hello %f %f \n", left, right);
	    dt_left.move(left);
	    dt_right.move(right);

		pros::delay(20 ? CONTROLLER_MODE == bluetooth : 50);
	}
}

//bool fix = true;

void colorSort() {
    if (((sideColor == blue && optical_sensor.get_hue() < 40 && optical_sensor.get_hue() > 0) || //red
         (sideColor == red && optical_sensor.get_hue() < 225 && optical_sensor.get_hue() > 160)) 
         && optical_sensor.get_proximity() > 100 && arm_controller.getAngle() < -200 && ejectEnabled) { //blue
        //pros::delay(200);
        intake_motor.move_velocity(600);
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        while (optical_sensor.get_proximity() > 200)
           pros::delay(10); 
        while (optical_sensor.get_proximity() > 75)
           pros::delay(2); 
        intake_motor.move_velocity(0);
        pros::delay(1000);
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } 
    
}

void intake () {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            ejectEnabled = !ejectEnabled;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move_velocity(600);
            colorSort();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move_velocity(-400);
        } else {
            intake_motor.move_velocity(0);
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void clamp() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clamp_solenoid.toggle();
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void topmech() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {arm_controller.changeAngle(-10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {arm_controller.changeAngle(10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {arm_controller.home();};
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
         //on vexnet: 30ms, on bluetooth, delay 50ms
        //double power = partner.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        //arm_motor.move(power);
    }
}