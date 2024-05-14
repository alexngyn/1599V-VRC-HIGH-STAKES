#include "main.h"

void initialize() {
	arm.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    pros::lcd::initialize(); // initialize brain screen
	printf("init");
}

bool reversey = false;

float driveCurve(float input, float scale) {
    if (scale != 0) {
        return (powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) * (1 - powf(2.718, -(scale / 10)))) * input;
    }
    return input;
}

std::pair<float, float> arcade(int throttle, int turn, float curveGain = 0) {
    turn *= 0.8;
    float leftPower = driveCurve(throttle + turn, curveGain);
    float rightPower = driveCurve(throttle - turn, curveGain);
    return std::make_pair(leftPower, rightPower);
}

std::pair<float, float> curvature(int throttle, int turn, float curveGain) {
    if (abs(throttle) < 4) {
        return arcade(throttle, turn, curveGain);;
    }

    float leftPower = throttle + (std::abs(throttle) * turn) / 127.0;
    float rightPower = throttle - (std::abs(throttle) * turn) / 127.0;

    leftPower = driveCurve(leftPower, curveGain);
    rightPower = driveCurve(rightPower, curveGain);

	return std::make_pair(leftPower, rightPower);
}

void drive() {
	while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) { reversey = !reversey; }

		double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
	    
		auto [left, right] = arcade(power, turn, 7.2);
        // auto [left, right] = curvature(power, turn, 7.2);

        if (reversey){ left *= -1; right *= -1; }

	    dt_left.move(left);
	    dt_right.move(right);

		pros::delay(10 ? CONTROLLER_MODE == bluetooth : 50);
	}
}

void intake () {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake_motor.move_velocity(200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_motor.move_velocity(-200);
        } else {
            intake_motor.move_velocity(0);
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void arms () {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            arm.move_velocity(50);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            arm.move_velocity(-50);
        } else {
            arm.move_velocity(0);
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void opcontrol() {
    dt_left.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    dt_right.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	pros::Task drive_thread(drive);
    pros::Task arms_thread(arms);
	pros::Task intake_thread(intake);
}
