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

void Arm::moveTo(double angle, bool async) {
    this->targetPosition = position::CUSTOM;
    this->targetAngle = angle;
    if (!async) { waitUntilDone(); }
}

void Arm::moveTo(position pos, bool async) {
    this->targetPosition = pos;
    if (!async) { waitUntilDone(); }
}

void Arm::changeAngle(double deltaAngle) {
    this->moveTo(this->targetAngle + deltaAngle);
}

double Arm::getAngle() {
    return this->rotation->get_position() * 0.01;
}

double Arm::getTargetAngle() {
    return this->targetAngle;
}

Arm::position Arm::getTargetPosition() {
    return this->targetPosition;
}

void Arm::waitUntilDone() {
    while (this->currentState == Arm::state::MOVING) {
        pros::delay(10);
    }
}

double Arm::angleStringToAngle() {
    switch (this->targetPosition) {
        case position::RETRACT: return 10;
        case position::INTAKE: return 30;
        case position::UP: return 80;
        case position::SCORE_NEUTRAL: return 140;
        case position::SCORE_ALLIANCE: return 240;
        default: return targetAngle;
    }
}

void Arm::home() {
    switch (this->targetPosition) {
        case position::RETRACT: targetPosition = position::INTAKE; break;
        case position::INTAKE: targetPosition = position::UP; break;
        case position::UP: targetPosition = position::SCORE_NEUTRAL; break;
        case position::SCORE_NEUTRAL: targetPosition = position::SCORE_ALLIANCE; break;
        case position::SCORE_ALLIANCE: targetPosition = position::RETRACT; break;
        default: targetPosition = position::RETRACT; break;
    }
}