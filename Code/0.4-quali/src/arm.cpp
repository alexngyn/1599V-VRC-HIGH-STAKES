#include "arm.h"

Arm::Arm(pros::Motor motor,
         pros::Rotation rotSensor, double ratio,
         lemlib::PID pid)
    : motor(std::move(motor)),
      rotSensor(std::move(rotSensor)),
      ratio(ratio),
      PID(pid) {
    this->reset();
}

void Arm::reset() {}

void Arm::moveToAngle(double angle) {
    this->targetAngle = angle;
}

void Arm::changeAngle(double deltaAngle) {
    this->moveToAngle(this->targetAngle + deltaAngle);
}

void Arm::home() {
    if (getAngle() < 30) {this->moveToAngle(90);}
    else {this->moveToAngle(12);};
}

double Arm::getAngle() {
    return this->rotSensor.get_position() * this->ratio * 0.01;
}

void Arm::apartment() {
    if (getAngle()<30) {this->moveToAngle(120);}
    else {this->moveToAngle(12);}
}