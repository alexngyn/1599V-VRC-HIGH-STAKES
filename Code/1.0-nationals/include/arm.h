#pragma once

#include "api.h"  // IWYU pragma: keep
//#include "setup.h"

#define targetAngleInit 16

class Arm {
    public:
        Arm(pros::Motor motor,
            pros::Rotation rotSensor, double ratio,
            lemlib::PID pid);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD
        };

        void moveToAngle(double angle);
        void home();
        void changeAngle(double deltaAngle);
        void apartment();

        double getAngle();
        bool isInPosition() { return this->currState == Arm::state::HOLD; }

    private:

        pros::Motor motor;
        pros::Rotation rotSensor;
        double ratio;

        lemlib::PID PID;
        double targetAngle = targetAngleInit;
        float UpwardGain = 2.0;
        float DownwardGain = 1.5;

        Arm::state currState = Arm::state::HOLD;

        pros::Task task = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                pros::delay(10);
                double error = lemlib::angleError(this->targetAngle, this->getAngle(), false);

                //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, error);

                if (std::fabs(error) <= 4) {
                    this->currState = Arm::state::HOLD;
                } else {
                    this->currState = Arm::state::MOVING;
                }

                if (this->currState == Arm::state::MOVING) {
                    double vel = this->PID.update(error);

                    if (vel > 0) { vel *= UpwardGain; } else {vel *= DownwardGain; }

                    //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, vel);
                     this->motor.move(vel);
                } else if (this->currState == Arm::state::HOLD) {
                    this->motor.move(0);
                }
            }
        }};;
};