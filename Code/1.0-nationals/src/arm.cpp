#include "arm.h"
#include <cstddef>

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
        case position::RETRACT: return -340;
        case position::INTAKE: return -310;
        case position::UP: return -220;
        case position::SCORE_NEUTRAL: return -180;
        case position::SCORE_ALLIANCE: return -150;
        default: return targetAngle;
    }
}

void Arm::togglePosition(position position1, position position2, 
                         position position3,position position4, position position5) {
    if (this->targetPosition == position1) {
        this->targetPosition = position2;
    } else if (this->targetPosition == position2) {
        if (position3 != position::CUSTOM) {this->targetPosition = position3;}
        else {this->targetPosition = position1;}
    } else if (this->targetPosition == position3) {
        if (position4 != position::CUSTOM) {this->targetPosition = position4;}
        else {this->targetPosition = position1;}
    } else if (this->targetPosition == position4) {
        if (position5 != position::CUSTOM) {this->targetPosition = position5;}
        else {this->targetPosition = position1;}
    } else {
        this->targetPosition = position1;
    }
}