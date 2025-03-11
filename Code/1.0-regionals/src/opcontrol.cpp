#include "arm.h"
#include "setup.h"

int sgn (float number) { return 1 ? number >= 0 : -1; }

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


    float leftPower = throttle + turn;
    float rightPower = throttle - turn;
    return std::make_pair(leftPower, rightPower);
}

void drive() {
	while (true) {
        double power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);


		auto [left, right] = arcade(power, turn, 4.2);

	    dt_left.move(left);
	    dt_right.move(right);

		pros::delay(20 ? CONTROLLER_MODE == bluetooth : 50);
	}
}

void intake () {
    intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            intake_controller.toggleState();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_controller.set(Intake::IntakeState::INTAKING);
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_controller.set(Intake::IntakeState::OUTTAKE);
        } else if (!(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))) {
            intake_controller.set(Intake::IntakeState::STOPPED);
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void piston(){
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            doinker_solenoid.toggle();
        }
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     clamp_solenoid.toggle();
        // }
        if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && clamp_solenoid.is_extended()) {
            clamp_solenoid.retract();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && !clamp_solenoid.is_extended()) { 
            clamp_solenoid.extend(); 
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            intake_solenoid.toggle();
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

void topmech() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {arm_controller.changeAngle(-8);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {arm_controller.changeAngle(8);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {arm_controller.moveTo(Arm::position::RETRACT);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {arm_controller.moveTo(Arm::position::CLIMB);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {arm_controller.togglePosition(Arm::position::INTAKE, 
                                                                                                                    Arm::position::SCORE_NEUTRAL);}

        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
        //on vexnet: 30ms, on bluetooth, delay 50ms
    }
}