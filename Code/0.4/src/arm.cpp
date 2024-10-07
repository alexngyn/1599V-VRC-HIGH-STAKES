#include "arm.h"

Arm::Arm(std::unique_ptr<pros::Motor> motor,
         std::unique_ptr<pros::Rotation> rot, double ratio,
         lemlib::PID pid, int rpm)
    : motor(std::move(motor)),
      rot(std::move(rot)),
      ratio(ratio),
      PID(pid),
      rpm(rpm) {
    this->reset();
}

void Arm::reset() {}

void Arm::moveToAngle(double angle) {
    double height = angleToHeight(angle);
//    if (height > 30.25 + 4.5 || height < 8 || this->currState == Arm::state::INACTIVE) return;
    if (angle > 55 || angle < -55 || this->currState == Arm::state::INACTIVE) return;
    this->targetAngle = angle;
}

void Arm::moveToHeight(double height) {
    this->moveToAngle(heightToAngle(height));
}

void Arm::changeAngle(double deltaAngle) {
    this->moveToAngle(this->targetAngle + deltaAngle);
}

void Arm::changeHeight(double deltaHeight) {
    this->moveToHeight(angleToHeight(this->targetAngle) + deltaHeight);
}

void Arm::disconnect() {
    this->currState = Arm::state::INACTIVE;
}

void Arm::connect() {
    this->motor->move(0);
    this->currState = Arm::state::HOLD;
}

void Arm::descore() {
    this->currState = Arm::state::DESCORE;
}

double Arm::getAngle() {
    return this->rot->get_position() * 0.01 * this->ratio;
}