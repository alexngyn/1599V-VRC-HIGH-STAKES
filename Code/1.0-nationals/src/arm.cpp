#include "arm.h"

/**
 * @brief Construct a new Arm object
 *
 * @param motors The motor group that controls the arm
 * @param rotation The rotation sensor that measures the arm's angle
 * @param pid The PID constants for the arm
 */

Arm::Arm(pros::MotorGroup* motors, pros::Rotation* rotation, lemlib::PID pid)
    : motors(motors),
      rotation(rotation),
      pid(pid) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD, 0);
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD, 1);
}    

void Arm::moveTo(double angle) {
    this->targetAngle = angle;
}

void Arm::changeAngle(double deltaAngle) {
    this->moveTo(this->targetAngle + deltaAngle);
}

double Arm::getAngle() {
    return this->rotation->get_position() * 0.01;
}

double Arm::getTargetAngle() {
    return this->rotation->get_position() * 0.01;
}

void Arm::home() {
    if (targetAngle == RETRACT) {this->moveTo(INTAKE);}
    else if (targetAngle == INTAKE) {this->moveTo(UP);}
    else if (targetAngle == UP) {this->moveTo(SCORE_NEUTRAL);}
    else if (targetAngle == SCORE_NEUTRAL) {this->moveTo(SCORE_ALLIANCE);}
    else if (targetAngle == SCORE_ALLIANCE) {this->moveTo(RETRACT);}
}