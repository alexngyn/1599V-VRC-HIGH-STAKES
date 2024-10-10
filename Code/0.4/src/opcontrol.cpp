#include "pros/motors.h"
#include "setup.h"
#include "gui.h"
#include "arm.h"

int sgn (float number) { return 1 ? number >= 0 : -1 ; }

bool reverse_mode = false;

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

/*
std::pair<float, float> arcade(int throttle, int turn, float curveGain = 0) {
    turn *= 1;
    float leftPower = driveCurve(throttle + turn, curveGain);
    float rightPower = driveCurve(throttle - turn, curveGain);
    return std::make_pair(leftPower, rightPower);
}
*/

/*
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
*/

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

        
        // printf("%f %f %f %f %f %f \n", dt_left.get_temperature(0), 
        //                                dt_left.get_temperature(1),
        //                                dt_left.get_temperature(2), 
        //                                dt_right.get_temperature(0), 
        //                                dt_right.get_temperature(1),
        //                                dt_right.get_temperature(2));
       //if (clamp_solenoid.is_extended())
	}
}

//bool fix = true;

void colorSort() {
    const int sortState = 1; //0 = off, 1 = blue, 2 = red

    if (((sortState == 1 && optical_sensor.get_hue() < 40 && optical_sensor.get_hue() > 0) || //red
         (sortState == 2 && optical_sensor.get_hue() < 225 && optical_sensor.get_hue() > 160)) && optical_sensor.get_proximity() > 100 ) { //blue
        //pros::delay(200);
        intake_motor.move_velocity(600);
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        while (optical_sensor.get_proximity() > 200   )
           pros::delay(10); 
        while (optical_sensor.get_proximity() > 75   )
           pros::delay(2); 
        intake_motor.move_velocity(0);
        pros::delay(1000);
        intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    } 
    
}

void intake () {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move_velocity(600);
            colorSort();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_motor.move_velocity(-400);
        } else {
            intake_motor.move_velocity(0);
        }
        //printf("%f \n", optical_sensor.get_hue() );
        //if (intake_motor.get_efficiency() < 10 && fix) {
            //intake_motor.move_relative(-360, 200);
            //pros::delay(500); //300ms per 360deg
        //}
        // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        //     fix = !fix;
        // }
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
        //printf("%f \n", arm_rotational_sensor.get_position()*0.01);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {arm_controller.changeAngle(-10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {arm_controller.changeAngle(10);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {arm_controller.home();};
        //double power = partner.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        //arm_motor.move(power);
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }
}

/*

Arm ptoArm(
    std::make_unique<pros::Motor>(3, pros::v5::MotorGears::blue),
    std::make_unique<pros::Rotation>(6),
    -0.25,
    lemlib::PID {10, 0, 20, 20, true},
    -20
);

void setPTO(bool state) {
    isPtoActive = state;
    if (state) {
        ptoPiston.extend();
        activeArm = &ptoArm;
        arm.connect();
        arm.moveToAngle(arm.getAngle());
    } else {
        ptoPiston.retract();
        activeArm = &arm;
        arm.disconnect();
    }
}

void pto() {
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            isPtoActive = !isPtoActive;
            if (isPtoActive) {
                ptoPiston.extend();
                activeArm = &ptoArm;
                arm.connect();
                arm.moveToAngle(arm.getAngle());
            } else {
                ptoPiston.retract();
                activeArm = &arm;
                arm.disconnect();
            }
        }
        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
    }

}

int slewControl(int desiredVoltage, int previousVoltage, int slewRate, int timestep){
    if(desiredVoltage != previousVoltage){
        if (desiredVoltage - previousVoltage > slewRate * timestep) {
            return previousVoltage + slewRate * timestep;
        }
        if (desiredVoltage - previousVoltage < -slewRate * timestep)
        {
            return previousVoltage - slewRate * timestep;
        }
    }
    return desiredVoltage;
}
*/