#pragma once
#include "api.h"  // IWYU pragma: keep

class Arm {
    public:
        Arm(std::unique_ptr<pros::Motor> motor,
            std::unique_ptr<pros::Rotation> enc, double ratio,
            lemlib::PID pid, int rpm);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD,
            DESCORE,
            STOP,
            INACTIVE
        };

        double angleOffset = 0;
//        bool autonMovement = true;

        void reset();

        static double angleToHeight(double angle) { return 18.5 + 12.5*std::sin(angle); }
        static double heightToAngle(double height) { return std::asin((height - 18.5) / 12.5); }

        double getHeight() { return angleToHeight(this->getAngle()); }
        void moveToAngle(double angle);
        void moveToHeight(double height);
        void changeAngle(double deltaAngle);
        void changeHeight(double deltaHeight);
        void disconnect();
        void connect();

        void descore();

        double getAngle();
        bool isInPosition() { return this->currState == Arm::state::HOLD; }
    private:

        std::unique_ptr<pros::Motor> motor;
        std::unique_ptr<pros::Rotation> rot;
        double ratio;

        lemlib::PID PID;
        int rpm;
        double targetAngle = -50;

        Arm::state currState = Arm::state::INACTIVE;

        pros::Task task = pros::Task {[&] {
            while (true) {
                pros::delay(10);
                double error = lemlib::angleError(this->getAngle(), this->targetAngle + angleOffset, false);

//                std::printf("Arm: %f | %f\n", this->getAngle(), this->targetAngle);

                if (this->currState == Arm::state::INACTIVE) continue;

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