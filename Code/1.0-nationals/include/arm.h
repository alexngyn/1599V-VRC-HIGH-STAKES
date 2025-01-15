#pragma once

#include "api.h"  // IWYU pragma: keep
//#include "setup.h"

class Arm {
    public:
        Arm(pros::MotorGroup* motors, pros::Rotation* rotation, lemlib::PID pid);
        ~Arm() {this->task.remove();};

        enum class state {
            MOVING,
            HOLD
        };

        enum class position {
            RETRACT,
            INTAKE,
            UP,
            SCORE_NEUTRAL,
            SCORE_ALLIANCE,
            CUSTOM
        };

        void moveTo(double angle, bool async = true);
        void moveTo(position angle, bool async = true);
        void changeAngle(double deltaAngle);
        double getAngle();
        double angleStringToAngle();
        double getTargetAngle();
        position getTargetPosition();
        void home();
        void waitUntilDone();

    private:
        pros::Rotation* rotation;
        pros::MotorGroup* motors;
        lemlib::PID pid;

        double targetAngle = 0; // init angle
        position targetPosition = position::RETRACT;
        //float UpwardGain = 2.0;
        //float DownwardGain = 1.5;

        Arm::state currentState = Arm::state::HOLD;

        pros::Task task = pros::Task {[&] {
            while (true) {                                                                                                                                                              
                pros::delay(10);
                double error = lemlib::angleError(this->targetAngle, this->getAngle());

                //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, error);

                if (std::fabs(error) <= 8) {
                    this->currentState = Arm::state::HOLD;
                } else {
                    this->currentState = Arm::state::MOVING;
                }

                if (this->currentState == Arm::state::MOVING) {
                    double vel = this->pid.update(error);

                    //if ((vel > 0 && this -> getAngle() < 180) || (vel < 0 && this -> getAngle() > 180)) { vel *= UpwardGain; } else {vel *= DownwardGain; }

                    //std::printf("Arm: %f | %f | %f \n", this->getAngle(), this->targetAngle, vel);
                     this->motors->move(vel);
                } else if (this->currentState == Arm::state::HOLD) {
                    this->motors->move(0);
                }
            }
        }};;
};