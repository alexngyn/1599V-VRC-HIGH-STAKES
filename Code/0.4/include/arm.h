#pragma once
#include "api.h"  // IWYU pragma: keep

class Arm {
    public:
        Arm(std::unique_ptr<pros::Motor> motor,
            std::unique_ptr<pros::Rotation> rotSensor, double ratio,
            double length, double heightOffset,
            lemlib::PID pid);
        ~Arm() {this->task.remove();};
        //double angleOffset = 0;
        double length, heightOffset;

        enum class state {
            MOVING,
            HOLD
        };

        void reset();

        double angleToHeight(double angle) { return 18.5 + 12.5*std::sin(angle); }
        double heightToAngle(double height) { return std::asin((height - 18.5) / 12.5); }

        double getHeight() { return angleToHeight(this->getAngle()); }
        void moveToAngle(double angle);
        void moveToHeight(double height);
        void changeAngle(double deltaAngle);
        void changeHeight(double deltaHeight);
        void home();

        double getAngle();
        bool isInPosition() { return this->currState == Arm::state::HOLD; }

    private:

        std::unique_ptr<pros::Motor> motor;
        std::unique_ptr<pros::Rotation> rotSensor;
        double ratio;

        lemlib::PID PID;
        double targetAngle = 0;

        Arm::state currState = Arm::state::HOLD;

        pros::Task task = pros::Task {[&] {
            while (true) {
                pros::delay(10);
                double error = lemlib::angleError(this->getAngle(), this->targetAngle, false);

//                std::printf("Arm: %f | %f\n", this->getAngle(), this->targetAngle);

                if (std::fabs(error) <= 4) {
                    this->currState = Arm::state::HOLD;
                } else {
                    this->currState = Arm::state::MOVING;
                }

                if (this->currState == Arm::state::MOVING) {
                    double vel = this->PID.update(error);
                    this->motor->move(vel);
                } else if (this->currState == Arm::state::HOLD) {
                    this->motor->move(0);
                }
            }
        }};;
};