#include "arm.h"

Arm::Arm(std::unique_ptr<pros::Motor> motor,
         std::unique_ptr<pros::Rotation> rotSensor, double ratio,
         double length, double heightOffset,
         lemlib::PID pid)
    : motor(std::move(motor)),
      rotSensor(std::move(rotSensor)),
      ratio(ratio),
      PID(pid),
      heightOffset(heightOffset) {
    this->reset();
}

void Arm::reset() {}

void Arm::moveToAngle(double angle) {
    double height = angleToHeight(angle);
//    if (height > 30.25 + 4.5 || height < 8 || this->currState == Arm::state::INACTIVE) return;
    if (angle > 55 || angle < -55) return;
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

void Arm::home() {
    if (getAngle() < 20) {this->moveToAngle(0);}
    else {this->moveToAngle(90);};
}

double Arm::getAngle() {
    return this->rotSensor->get_position() * 0.01 * this->ratio;
}