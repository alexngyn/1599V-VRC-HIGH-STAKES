#include "arm.h"
#include "setup.h"

int sgn (float number) { return 1 ? number >= 0 : -1; }

bool ejectEnabled = true;

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
        //else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {arm_controller.moveTo(Arm::position::CLIMB);}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {arm_controller.togglePosition(Arm::position::INTAKE, 
                                                                                                                    Arm::position::SCORE_NEUTRAL);}

        pros::delay(30 ? CONTROLLER_MODE == bluetooth : 50);
        //on vexnet: 30ms, on bluetooth, delay 50ms
    }
}